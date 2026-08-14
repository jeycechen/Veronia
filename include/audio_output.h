#ifndef VERONIA_AUDIO_OUTPUT_H
#define VERONIA_AUDIO_OUTPUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// I2S output wiring for an external DAC or I2S amplifier (for example MAX98357A).
#ifndef VERONIA_AUDIO_PIN_BCLK
#define VERONIA_AUDIO_PIN_BCLK 4
#endif

#ifndef VERONIA_AUDIO_PIN_WS
#define VERONIA_AUDIO_PIN_WS 5
#endif

#ifndef VERONIA_AUDIO_PIN_DOUT
#define VERONIA_AUDIO_PIN_DOUT 6
#endif

#ifndef VERONIA_AUDIO_SAMPLE_RATE_HZ
#define VERONIA_AUDIO_SAMPLE_RATE_HZ 44100
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize I2S TX as 16-bit signed, stereo, Philips-format PCM. */
esp_err_t veronia_audio_output_init(uint32_t sample_rate_hz);
esp_err_t veronia_audio_output_deinit(void);
bool veronia_audio_output_is_ready(void);

/**
 * Write interleaved signed-16-bit stereo PCM frames to the output.
 * samples is [left0, right0, left1, right1, ...].
 * TODO: Add a decoder layer for files read from SD storage (for example WAV/MP3)
 *       and feed its decoded PCM frames into this function.
 */
esp_err_t veronia_audio_output_write_frames(const int16_t *samples, size_t frame_count,
                                            uint32_t timeout_ms, size_t *frames_written);

/** Play a synchronous sine-wave tone; useful for alerts and hardware testing. */
esp_err_t veronia_audio_output_play_tone(uint32_t frequency_hz, uint32_t duration_ms,
                                         uint8_t volume_percent);

#ifdef __cplusplus
}
#endif

#endif // VERONIA_AUDIO_OUTPUT_H
