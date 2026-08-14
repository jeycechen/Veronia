#include "display.h"

#include <stdlib.h>

#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "platform/spi_bus_manager.h"

namespace {

constexpr char TAG[] = "display";
constexpr int DRAW_BUFFER_LINES = 20;
constexpr size_t DRAW_BUFFER_PIXELS = VERONIA_DISPLAY_WIDTH * DRAW_BUFFER_LINES;
constexpr size_t SPI_MAX_TRANSFER_SIZE = 16 * 1024;

SemaphoreHandle_t display_mutex = nullptr;
esp_lcd_panel_io_handle_t panel_io = nullptr;
esp_lcd_panel_handle_t panel = nullptr;
uint16_t *draw_buffer = nullptr;
bool initialized = false;

bool ensure_mutex()
{
    if (display_mutex == nullptr) {
        display_mutex = xSemaphoreCreateMutex();
    }
    return display_mutex != nullptr;
}

esp_err_t fill_rectangle(int x, int y, int width, int height, uint16_t color)
{
    if (x < 0 || y < 0 || width <= 0 || height <= 0 ||
        x + width > VERONIA_DISPLAY_WIDTH || y + height > VERONIA_DISPLAY_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t index = 0; index < DRAW_BUFFER_PIXELS; ++index) {
        draw_buffer[index] = color;
    }

    int current_y = y;
    int remaining_lines = height;
    while (remaining_lines > 0) {
        const int line_count = remaining_lines > DRAW_BUFFER_LINES ? DRAW_BUFFER_LINES : remaining_lines;
        const esp_err_t result = esp_lcd_panel_draw_bitmap(panel, x, current_y,
                                                            x + width, current_y + line_count,
                                                            draw_buffer);
        if (result != ESP_OK) {
            return result;
        }
        current_y += line_count;
        remaining_lines -= line_count;
    }
    return ESP_OK;
}

} // namespace

extern "C" esp_err_t veronia_display_init(void)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    if (initialized) {
        xSemaphoreGive(display_mutex);
        return ESP_OK;
    }

    esp_err_t result = veronia_spi_bus_acquire(VERONIA_DISPLAY_SPI_HOST,
                                               VERONIA_DISPLAY_PIN_MOSI,
                                               VERONIA_DISPLAY_PIN_MISO,
                                               VERONIA_DISPLAY_PIN_SCLK,
                                               SPI_MAX_TRANSFER_SIZE);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus acquisition failed: %s", esp_err_to_name(result));
        xSemaphoreGive(display_mutex);
        return result;
    }

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num = VERONIA_DISPLAY_PIN_CS;
    io_config.dc_gpio_num = VERONIA_DISPLAY_PIN_DC;
    io_config.spi_mode = 0;
    io_config.pclk_hz = 20 * 1000 * 1000;
    io_config.trans_queue_depth = 1;
    io_config.lcd_cmd_bits = 8;
    io_config.lcd_param_bits = 8;

    result = esp_lcd_new_panel_io_spi(VERONIA_DISPLAY_SPI_HOST, &io_config, &panel_io);
    if (result == ESP_OK) {
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = VERONIA_DISPLAY_PIN_RST;
        panel_config.color_space = ESP_LCD_COLOR_SPACE_RGB;
        panel_config.bits_per_pixel = 16;
        result = esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel);
    }
    if (result == ESP_OK) {
        result = esp_lcd_panel_reset(panel);
    }
    if (result == ESP_OK) {
        result = esp_lcd_panel_init(panel);
    }
    if (result == ESP_OK) {
        result = esp_lcd_panel_disp_on_off(panel, true);
    }

    draw_buffer = static_cast<uint16_t *>(heap_caps_malloc(DRAW_BUFFER_PIXELS * sizeof(uint16_t),
                                                             MALLOC_CAP_DMA));
    if (result == ESP_OK && draw_buffer == nullptr) {
        result = ESP_ERR_NO_MEM;
    }
    if (result != ESP_OK) {
        if (panel != nullptr) {
            esp_lcd_panel_del(panel);
            panel = nullptr;
        }
        if (panel_io != nullptr) {
            esp_lcd_panel_io_del(panel_io);
            panel_io = nullptr;
        }
        veronia_spi_bus_release(VERONIA_DISPLAY_SPI_HOST);
        xSemaphoreGive(display_mutex);
        return result;
    }

    initialized = true;
    ESP_LOGI(TAG, "ILI9341 initialized (%dx%d)", VERONIA_DISPLAY_WIDTH, VERONIA_DISPLAY_HEIGHT);
    xSemaphoreGive(display_mutex);
    return ESP_OK;
}

extern "C" esp_err_t veronia_display_deinit(void)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    if (!initialized) {
        xSemaphoreGive(display_mutex);
        return ESP_OK;
    }

    esp_lcd_panel_disp_on_off(panel, false);
    esp_lcd_panel_del(panel);
    esp_lcd_panel_io_del(panel_io);
    free(draw_buffer);
    panel = nullptr;
    panel_io = nullptr;
    draw_buffer = nullptr;
    initialized = false;
    const esp_err_t result = veronia_spi_bus_release(VERONIA_DISPLAY_SPI_HOST);
    xSemaphoreGive(display_mutex);
    return result;
}

extern "C" bool veronia_display_is_ready(void)
{
    return initialized;
}

extern "C" esp_err_t veronia_display_fill(uint16_t rgb565_color)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(display_mutex, portMAX_DELAY);
    const esp_err_t result = initialized
        ? fill_rectangle(0, 0, VERONIA_DISPLAY_WIDTH, VERONIA_DISPLAY_HEIGHT, rgb565_color)
        : ESP_ERR_INVALID_STATE;
    xSemaphoreGive(display_mutex);
    return result;
}

extern "C" esp_err_t veronia_display_draw_rgb565_bitmap(int x, int y, int width, int height,
                                                          const uint16_t *pixels)
{
    if (pixels == nullptr || x < 0 || y < 0 || width <= 0 || height <= 0 ||
        x + width > VERONIA_DISPLAY_WIDTH || y + height > VERONIA_DISPLAY_HEIGHT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(display_mutex, portMAX_DELAY);
    const esp_err_t result = initialized
        ? esp_lcd_panel_draw_bitmap(panel, x, y, x + width, y + height, pixels)
        : ESP_ERR_INVALID_STATE;
    xSemaphoreGive(display_mutex);
    return result;
}

extern "C" esp_err_t veronia_display_show_test_pattern(void)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }
    xSemaphoreTake(display_mutex, portMAX_DELAY);
    esp_err_t result = initialized ? ESP_OK : ESP_ERR_INVALID_STATE;
    if (result == ESP_OK) result = fill_rectangle(0, 0, VERONIA_DISPLAY_WIDTH, 80, 0xF800);
    if (result == ESP_OK) result = fill_rectangle(0, 80, VERONIA_DISPLAY_WIDTH, 80, 0x07E0);
    if (result == ESP_OK) result = fill_rectangle(0, 160, VERONIA_DISPLAY_WIDTH, 80, 0x001F);
    if (result == ESP_OK) result = fill_rectangle(0, 240, VERONIA_DISPLAY_WIDTH, 80, 0xFFFF);
    xSemaphoreGive(display_mutex);
    return result;
}
