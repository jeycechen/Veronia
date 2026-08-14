#include "platform/spi_bus_manager.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

SemaphoreHandle_t mutex = nullptr;
spi_host_device_t active_host = SPI_HOST_MAX;
int active_mosi = -1;
int active_miso = -1;
int active_sclk = -1;
size_t active_max_transfer_size = 0;
unsigned int client_count = 0;

bool ensure_mutex()
{
    if (mutex == nullptr) {
        mutex = xSemaphoreCreateMutex();
    }
    return mutex != nullptr;
}

} // namespace

extern "C" esp_err_t veronia_spi_bus_acquire(spi_host_device_t host, int mosi_pin,
                                               int miso_pin, int sclk_pin,
                                               size_t max_transfer_size)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);
    if (client_count != 0) {
        const bool matching_bus = active_host == host && active_mosi == mosi_pin &&
                                  active_miso == miso_pin && active_sclk == sclk_pin &&
                                  active_max_transfer_size >= max_transfer_size;
        if (!matching_bus) {
            xSemaphoreGive(mutex);
            return ESP_ERR_INVALID_STATE;
        }
        ++client_count;
        xSemaphoreGive(mutex);
        return ESP_OK;
    }

    spi_bus_config_t config = {};
    config.mosi_io_num = mosi_pin;
    config.miso_io_num = miso_pin;
    config.sclk_io_num = sclk_pin;
    config.quadwp_io_num = -1;
    config.quadhd_io_num = -1;
    config.max_transfer_sz = max_transfer_size;

    const esp_err_t result = spi_bus_initialize(host, &config, SPI_DMA_CH_AUTO);
    if (result == ESP_OK) {
        active_host = host;
        active_mosi = mosi_pin;
        active_miso = miso_pin;
        active_sclk = sclk_pin;
        active_max_transfer_size = max_transfer_size;
        client_count = 1;
    }
    xSemaphoreGive(mutex);
    return result;
}

extern "C" esp_err_t veronia_spi_bus_release(spi_host_device_t host)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(mutex, portMAX_DELAY);
    if (client_count == 0 || active_host != host) {
        xSemaphoreGive(mutex);
        return ESP_ERR_INVALID_STATE;
    }

    --client_count;
    esp_err_t result = ESP_OK;
    if (client_count == 0) {
        result = spi_bus_free(active_host);
        if (result == ESP_OK) {
            active_host = SPI_HOST_MAX;
            active_mosi = -1;
            active_miso = -1;
            active_sclk = -1;
            active_max_transfer_size = 0;
        } else {
            client_count = 1;
        }
    }
    xSemaphoreGive(mutex);
    return result;
}
