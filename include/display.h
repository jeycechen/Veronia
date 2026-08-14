#ifndef VERONIA_DISPLAY_H
#define VERONIA_DISPLAY_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"

// ILI9341 SPI wiring. These default values match the existing Wokwi diagram.
// SD storage can share this bus only when its SPI host and MOSI/MISO/SCLK values match.
#ifndef VERONIA_DISPLAY_SPI_HOST
#define VERONIA_DISPLAY_SPI_HOST SPI3_HOST
#endif

#ifndef VERONIA_DISPLAY_PIN_MOSI
#define VERONIA_DISPLAY_PIN_MOSI 12
#endif

#ifndef VERONIA_DISPLAY_PIN_MISO
#define VERONIA_DISPLAY_PIN_MISO 13
#endif

#ifndef VERONIA_DISPLAY_PIN_SCLK
#define VERONIA_DISPLAY_PIN_SCLK 11
#endif

#ifndef VERONIA_DISPLAY_PIN_CS
#define VERONIA_DISPLAY_PIN_CS 9
#endif

#ifndef VERONIA_DISPLAY_PIN_DC
#define VERONIA_DISPLAY_PIN_DC 10
#endif

#ifndef VERONIA_DISPLAY_PIN_RST
#define VERONIA_DISPLAY_PIN_RST -1
#endif

#define VERONIA_DISPLAY_WIDTH 240
#define VERONIA_DISPLAY_HEIGHT 320

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t veronia_display_init(void);
esp_err_t veronia_display_deinit(void);
bool veronia_display_is_ready(void);

esp_err_t veronia_display_fill(uint16_t rgb565_color);
esp_err_t veronia_display_draw_rgb565_bitmap(int x, int y, int width, int height,
                                              const uint16_t *pixels);
esp_err_t veronia_display_show_test_pattern(void);

#ifdef __cplusplus
}
#endif

#endif // VERONIA_DISPLAY_H
