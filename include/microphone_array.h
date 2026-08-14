#ifndef VERONIA_MICROPHONE_ARRAY_H
#define VERONIA_MICROPHONE_ARRAY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Microphone positions in millimetres, relative to the array centre.
// The defaults describe an equilateral three-microphone array with a 35 mm radius.
#ifndef VERONIA_MIC0_X_MM
#define VERONIA_MIC0_X_MM 35.0f
#endif
#ifndef VERONIA_MIC0_Y_MM
#define VERONIA_MIC0_Y_MM 0.0f
#endif
#ifndef VERONIA_MIC1_X_MM
#define VERONIA_MIC1_X_MM -17.5f
#endif
#ifndef VERONIA_MIC1_Y_MM
#define VERONIA_MIC1_Y_MM 30.31f
#endif
#ifndef VERONIA_MIC2_X_MM
#define VERONIA_MIC2_X_MM -17.5f
#endif
#ifndef VERONIA_MIC2_Y_MM
#define VERONIA_MIC2_Y_MM -30.31f
#endif

typedef struct {
    bool valid;
    // Angle from the mic0 axis, counter-clockwise, in [-pi, pi].
    float azimuth_rad;
    // Mean normalized cross-correlation of the two TDOA estimates, [0, 1].
    float confidence;
    float mic1_minus_mic0_delay_samples;
    float mic2_minus_mic0_delay_samples;
} veronia_sound_direction_t;

typedef void (*veronia_sound_direction_callback_t)(const veronia_sound_direction_t *direction,
                                                   void *context);

// TODO: Connect an offline wake-word engine here (for example ESP-SR) and invoke
// this callback only after the engine confirms the configured wake word.
typedef void (*veronia_wake_word_callback_t)(void *context);

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the three-microphone TDOA localization algorithm. */
esp_err_t veronia_microphone_array_init(uint32_t sample_rate_hz);
bool veronia_microphone_array_is_ready(void);

void veronia_microphone_array_set_direction_callback(veronia_sound_direction_callback_t callback,
                                                      void *context);
void veronia_microphone_array_set_wake_word_callback(veronia_wake_word_callback_t callback,
                                                      void *context);

/**
 * Submit synchronous, interleaved signed-16-bit PCM from all three microphones.
 * samples layout: [mic0_0, mic1_0, mic2_0, mic0_1, mic1_1, mic2_1, ...].
 * A TDM/I2S/PDM capture backend should call this function for each audio block.
 * TODO: Implement that capture backend after the microphone model and wiring are chosen.
 */
esp_err_t veronia_microphone_array_submit_frames(const int16_t *samples, size_t frame_count,
                                                 veronia_sound_direction_t *out_direction);

/** Temporary integration point for a future wake-word engine. */
void veronia_microphone_array_report_wake_word_detected(void);

#ifdef __cplusplus
}
#endif

#endif // VERONIA_MICROPHONE_ARRAY_H
