#ifndef VERONIA_SPI_BUS_MANAGER_H
#define VERONIA_SPI_BUS_MANAGER_H

#include <stddef.h>
#include "driver/spi_master.h"
#include "esp_err.h"

// Acquires a shared SPI bus. All clients of one host must pass identical pins.
esp_err_t veronia_spi_bus_acquire(spi_host_device_t host, int mosi_pin, int miso_pin,
                                  int sclk_pin, size_t max_transfer_size);

// Releases one acquisition. The bus is freed after the final client releases it.
esp_err_t veronia_spi_bus_release(spi_host_device_t host);

#endif // VERONIA_SPI_BUS_MANAGER_H
