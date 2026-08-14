#ifndef VERONIA_SD_STORAGE_H
#define VERONIA_SD_STORAGE_H

#include <stdbool.h>
#include <stddef.h>
#include "driver/spi_master.h"
#include "esp_err.h"

// SPI SD card wiring. SD storage and the display can share a SPI bus when
// host/MOSI/MISO/SCLK match; each device must use a distinct CS pin.
#ifndef VERONIA_SD_SPI_HOST
#define VERONIA_SD_SPI_HOST SPI3_HOST
#endif

#ifndef VERONIA_SD_PIN_MOSI
#define VERONIA_SD_PIN_MOSI 12
#endif

#ifndef VERONIA_SD_PIN_MISO
#define VERONIA_SD_PIN_MISO 13
#endif

#ifndef VERONIA_SD_PIN_SCLK
#define VERONIA_SD_PIN_SCLK 11
#endif

#ifndef VERONIA_SD_PIN_CS
#define VERONIA_SD_PIN_CS 14
#endif

#ifndef VERONIA_SD_MOUNT_POINT
#define VERONIA_SD_MOUNT_POINT "/sdcard"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Mount the FAT filesystem on the SPI-connected SD card. */
esp_err_t veronia_sd_storage_mount(void);

/** Flush, unmount, and release the SPI bus owned by this component. */
esp_err_t veronia_sd_storage_unmount(void);

bool veronia_sd_storage_is_mounted(void);

/**
 * Read a file below VERONIA_SD_MOUNT_POINT.
 * path must be a relative path, e.g. "settings/config.json".
 * The function does not append a '\0' terminator; out_size receives bytes read.
 */
esp_err_t veronia_sd_storage_read_file(const char *path, void *buffer,
                                       size_t buffer_size, size_t *out_size);

/** Create or replace a file below VERONIA_SD_MOUNT_POINT. */
esp_err_t veronia_sd_storage_write_file(const char *path, const void *data,
                                        size_t data_size);

/** Append data to a file below VERONIA_SD_MOUNT_POINT. */
esp_err_t veronia_sd_storage_append_file(const char *path, const void *data,
                                         size_t data_size);

#ifdef __cplusplus
}
#endif

#endif // VERONIA_SD_STORAGE_H
