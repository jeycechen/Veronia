#include "microphone_array.h"

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

constexpr float SPEED_OF_SOUND_M_PER_S = 343.0f;
constexpr float MIN_CORRELATION = 0.35f;
constexpr float MIN_DIRECTION_VECTOR_LENGTH = 0.2f;

struct Point {
    float x;
    float y;
};

SemaphoreHandle_t microphone_mutex = nullptr;
uint32_t sample_rate_hz = 0;
bool initialized = false;
veronia_sound_direction_callback_t direction_callback = nullptr;
void *direction_callback_context = nullptr;
veronia_wake_word_callback_t wake_word_callback = nullptr;
void *wake_word_callback_context = nullptr;

constexpr Point MIC0 = {VERONIA_MIC0_X_MM / 1000.0f, VERONIA_MIC0_Y_MM / 1000.0f};
constexpr Point MIC1 = {VERONIA_MIC1_X_MM / 1000.0f, VERONIA_MIC1_Y_MM / 1000.0f};
constexpr Point MIC2 = {VERONIA_MIC2_X_MM / 1000.0f, VERONIA_MIC2_Y_MM / 1000.0f};

bool ensure_mutex()
{
    if (microphone_mutex == nullptr) {
        microphone_mutex = xSemaphoreCreateMutex();
    }
    return microphone_mutex != nullptr;
}

float distance_between(Point first, Point second)
{
    const float x = first.x - second.x;
    const float y = first.y - second.y;
    return sqrtf(x * x + y * y);
}

float find_delay_samples(const int16_t *samples, size_t frame_count, size_t mic_index,
                         int max_lag, float *best_correlation)
{
    float best_score = -1.0f;
    int best_lag = 0;

    for (int lag = -max_lag; lag <= max_lag; ++lag) {
        const size_t start_frame = lag < 0 ? static_cast<size_t>(-lag) : 0;
        const size_t end_frame = lag > 0 ? frame_count - static_cast<size_t>(lag) : frame_count;
        double cross = 0.0;
        double reference_energy = 0.0;
        double microphone_energy = 0.0;

        for (size_t frame = start_frame; frame < end_frame; ++frame) {
            const float reference = static_cast<float>(samples[frame * 3]);
            const size_t delayed_frame = static_cast<size_t>(static_cast<int>(frame) + lag);
            const float microphone = static_cast<float>(samples[delayed_frame * 3 + mic_index]);
            cross += reference * microphone;
            reference_energy += reference * reference;
            microphone_energy += microphone * microphone;
        }

        const double energy = reference_energy * microphone_energy;
        if (energy <= 0.0) {
            continue;
        }
        const float score = static_cast<float>(cross / sqrt(energy));
        if (score > best_score) {
            best_score = score;
            best_lag = lag;
        }
    }

    *best_correlation = best_score < 0.0f ? 0.0f : best_score;
    return static_cast<float>(best_lag);
}

veronia_sound_direction_t locate_sound(const int16_t *samples, size_t frame_count)
{
    veronia_sound_direction_t direction = {};
    const float longest_baseline = fmaxf(distance_between(MIC0, MIC1), distance_between(MIC0, MIC2));
    const int max_lag = static_cast<int>(ceilf(longest_baseline * sample_rate_hz /
                                                SPEED_OF_SOUND_M_PER_S)) + 1;
    if (frame_count <= static_cast<size_t>(max_lag * 2 + 1)) {
        return direction;
    }

    float correlation_10 = 0.0f;
    float correlation_20 = 0.0f;
    const float delay_10 = find_delay_samples(samples, frame_count, 1, max_lag, &correlation_10);
    const float delay_20 = find_delay_samples(samples, frame_count, 2, max_lag, &correlation_20);
    direction.mic1_minus_mic0_delay_samples = delay_10;
    direction.mic2_minus_mic0_delay_samples = delay_20;
    direction.confidence = (correlation_10 + correlation_20) * 0.5f;
    if (direction.confidence < MIN_CORRELATION) {
        return direction;
    }

    const float baseline_10_x = MIC1.x - MIC0.x;
    const float baseline_10_y = MIC1.y - MIC0.y;
    const float baseline_20_x = MIC2.x - MIC0.x;
    const float baseline_20_y = MIC2.y - MIC0.y;
    const float determinant = baseline_10_x * baseline_20_y - baseline_10_y * baseline_20_x;
    if (fabsf(determinant) < 0.000001f) {
        return direction;
    }

    const float delay_10_s = delay_10 / static_cast<float>(sample_rate_hz);
    const float delay_20_s = delay_20 / static_cast<float>(sample_rate_hz);
    const float projection_10 = -SPEED_OF_SOUND_M_PER_S * delay_10_s;
    const float projection_20 = -SPEED_OF_SOUND_M_PER_S * delay_20_s;
    float direction_x = (projection_10 * baseline_20_y - baseline_10_y * projection_20) / determinant;
    float direction_y = (baseline_10_x * projection_20 - projection_10 * baseline_20_x) / determinant;
    const float length = sqrtf(direction_x * direction_x + direction_y * direction_y);
    if (length < MIN_DIRECTION_VECTOR_LENGTH) {
        return direction;
    }

    direction_x /= length;
    direction_y /= length;
    direction.azimuth_rad = atan2f(direction_y, direction_x);
    direction.valid = true;
    return direction;
}

} // namespace

extern "C" esp_err_t veronia_microphone_array_init(uint32_t new_sample_rate_hz)
{
    if (new_sample_rate_hz == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(microphone_mutex, portMAX_DELAY);
    if (initialized && sample_rate_hz != new_sample_rate_hz) {
        xSemaphoreGive(microphone_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    sample_rate_hz = new_sample_rate_hz;
    initialized = true;
    xSemaphoreGive(microphone_mutex);
    return ESP_OK;
}

extern "C" bool veronia_microphone_array_is_ready(void)
{
    return initialized;
}

extern "C" void veronia_microphone_array_set_direction_callback(
    veronia_sound_direction_callback_t callback, void *context)
{
    if (!ensure_mutex()) {
        return;
    }
    xSemaphoreTake(microphone_mutex, portMAX_DELAY);
    direction_callback = callback;
    direction_callback_context = context;
    xSemaphoreGive(microphone_mutex);
}

extern "C" void veronia_microphone_array_set_wake_word_callback(
    veronia_wake_word_callback_t callback, void *context)
{
    if (!ensure_mutex()) {
        return;
    }
    xSemaphoreTake(microphone_mutex, portMAX_DELAY);
    wake_word_callback = callback;
    wake_word_callback_context = context;
    xSemaphoreGive(microphone_mutex);
}

extern "C" esp_err_t veronia_microphone_array_submit_frames(const int16_t *samples,
                                                              size_t frame_count,
                                                              veronia_sound_direction_t *out_direction)
{
    if (samples == nullptr || out_direction == nullptr || frame_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(microphone_mutex, portMAX_DELAY);
    if (!initialized) {
        xSemaphoreGive(microphone_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    const veronia_sound_direction_t direction = locate_sound(samples, frame_count);
    *out_direction = direction;
    const veronia_sound_direction_callback_t callback = direction_callback;
    void *const callback_context = direction_callback_context;
    xSemaphoreGive(microphone_mutex);

    if (direction.valid && callback != nullptr) {
        callback(&direction, callback_context);
    }
    return ESP_OK;
}

extern "C" void veronia_microphone_array_report_wake_word_detected(void)
{
    if (!ensure_mutex()) {
        return;
    }
    xSemaphoreTake(microphone_mutex, portMAX_DELAY);
    const veronia_wake_word_callback_t callback = wake_word_callback;
    void *const callback_context = wake_word_callback_context;
    xSemaphoreGive(microphone_mutex);

    if (callback != nullptr) {
        callback(callback_context);
    }
}
