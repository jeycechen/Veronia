#include "sd_storage.h"

#include <stdio.h>
#include <string.h>

#include "driver/sdspi_host.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "platform/spi_bus_manager.h"
#include "sdmmc_cmd.h"

namespace {

constexpr char TAG[] = "sd_storage";
constexpr size_t MAX_FULL_PATH_LENGTH = 256;

SemaphoreHandle_t storage_mutex = nullptr;
sdmmc_card_t *card = nullptr;
bool mounted = false;

bool ensure_mutex()
{
    if (storage_mutex == nullptr) {
        storage_mutex = xSemaphoreCreateMutex();
    }
    return storage_mutex != nullptr;
}

bool is_safe_relative_path(const char *path)
{
    return path != nullptr && path[0] != '\0' && path[0] != '/' &&
           strstr(path, "..") == nullptr;
}

esp_err_t make_full_path(const char *path, char *full_path, size_t full_path_size)
{
    if (!is_safe_relative_path(path)) {
        return ESP_ERR_INVALID_ARG;
    }

    const int path_length = snprintf(full_path, full_path_size, "%s/%s",
                                     VERONIA_SD_MOUNT_POINT, path);
    if (path_length < 0 || static_cast<size_t>(path_length) >= full_path_size) {
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

esp_err_t write_file(const char *path, const void *data, size_t data_size, const char *mode)
{
    if ((data == nullptr && data_size != 0) || mode == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    if (!mounted) {
        xSemaphoreGive(storage_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[MAX_FULL_PATH_LENGTH];
    esp_err_t result = make_full_path(path, full_path, sizeof(full_path));
    if (result == ESP_OK) {
        FILE *file = fopen(full_path, mode);
        if (file == nullptr) {
            ESP_LOGE(TAG, "Unable to open %s", full_path);
            result = ESP_FAIL;
        } else {
            const size_t written = data_size == 0 ? 0 : fwrite(data, 1, data_size, file);
            if (written != data_size || fflush(file) != 0) {
                ESP_LOGE(TAG, "Unable to write %s", full_path);
                result = ESP_FAIL;
            }
            if (fclose(file) != 0 && result == ESP_OK) {
                result = ESP_FAIL;
            }
        }
    }
    xSemaphoreGive(storage_mutex);
    return result;
}

} // namespace

extern "C" esp_err_t veronia_sd_storage_mount(void)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    if (mounted) {
        xSemaphoreGive(storage_mutex);
        return ESP_OK;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VERONIA_SD_SPI_HOST;

    esp_err_t result = veronia_spi_bus_acquire(static_cast<spi_host_device_t>(host.slot),
                                               VERONIA_SD_PIN_MOSI, VERONIA_SD_PIN_MISO,
                                               VERONIA_SD_PIN_SCLK, 16 * 1024);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(result));
        xSemaphoreGive(storage_mutex);
        return result;
    }
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = host.slot;
    slot_config.gpio_cs = VERONIA_SD_PIN_CS;

    esp_vfs_fat_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 5;
    mount_config.allocation_unit_size = 16 * 1024;

    result = esp_vfs_fat_sdspi_mount(VERONIA_SD_MOUNT_POINT, &host, &slot_config,
                                     &mount_config, &card);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(result));
        veronia_spi_bus_release(static_cast<spi_host_device_t>(host.slot));
        card = nullptr;
        xSemaphoreGive(storage_mutex);
        return result;
    }

    mounted = true;
    ESP_LOGI(TAG, "Mounted SD card at %s", VERONIA_SD_MOUNT_POINT);
    sdmmc_card_print_info(stdout, card);
    xSemaphoreGive(storage_mutex);
    return ESP_OK;
}

extern "C" esp_err_t veronia_sd_storage_unmount(void)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    if (!mounted) {
        xSemaphoreGive(storage_mutex);
        return ESP_OK;
    }

    esp_vfs_fat_sdcard_unmount(VERONIA_SD_MOUNT_POINT, card);
    card = nullptr;
    mounted = false;
    const esp_err_t result = veronia_spi_bus_release(VERONIA_SD_SPI_HOST);
    xSemaphoreGive(storage_mutex);
    return result;
}

extern "C" bool veronia_sd_storage_is_mounted(void)
{
    return mounted;
}

extern "C" esp_err_t veronia_sd_storage_read_file(const char *path, void *buffer,
                                                    size_t buffer_size, size_t *out_size)
{
    if (buffer == nullptr || buffer_size == 0 || out_size == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_size = 0;
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    if (!mounted) {
        xSemaphoreGive(storage_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    char full_path[MAX_FULL_PATH_LENGTH];
    esp_err_t result = make_full_path(path, full_path, sizeof(full_path));
    if (result == ESP_OK) {
        FILE *file = fopen(full_path, "rb");
        if (file == nullptr) {
            ESP_LOGE(TAG, "Unable to open %s", full_path);
            result = ESP_ERR_NOT_FOUND;
        } else {
            *out_size = fread(buffer, 1, buffer_size, file);
            if (ferror(file)) {
                ESP_LOGE(TAG, "Unable to read %s", full_path);
                result = ESP_FAIL;
            }
            if (fclose(file) != 0 && result == ESP_OK) {
                result = ESP_FAIL;
            }
        }
    }
    xSemaphoreGive(storage_mutex);
    return result;
}

extern "C" esp_err_t veronia_sd_storage_write_file(const char *path, const void *data,
                                                     size_t data_size)
{
    return write_file(path, data, data_size, "wb");
}

extern "C" esp_err_t veronia_sd_storage_append_file(const char *path, const void *data,
                                                      size_t data_size)
{
    return write_file(path, data, data_size, "ab");
}
