#include "audio_output.h"

#include <math.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace {

constexpr char TAG[] = "audio_output";
constexpr float TWO_PI = 6.28318530718f;
constexpr size_t TONE_BUFFER_FRAME_COUNT = 128;
constexpr uint32_t WAIT_FOREVER = UINT32_MAX;

SemaphoreHandle_t audio_mutex = nullptr;
i2s_chan_handle_t tx_channel = nullptr;
uint32_t active_sample_rate_hz = 0;

bool ensure_mutex()
{
    if (audio_mutex == nullptr) {
        audio_mutex = xSemaphoreCreateMutex();
    }
    return audio_mutex != nullptr;
}

TickType_t timeout_to_ticks(uint32_t timeout_ms)
{
    if (timeout_ms == WAIT_FOREVER) {
        return portMAX_DELAY;
    }
    const TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    return timeout_ms != 0 && ticks == 0 ? 1 : ticks;
}

esp_err_t write_frames_locked(const int16_t *samples, size_t frame_count,
                              uint32_t timeout_ms, size_t *frames_written)
{
    if (samples == nullptr || frames_written == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame_count > SIZE_MAX / (2 * sizeof(int16_t))) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t bytes_written = 0;
    const esp_err_t result = i2s_channel_write(tx_channel, samples,
                                               frame_count * 2 * sizeof(int16_t),
                                               &bytes_written, timeout_to_ticks(timeout_ms));
    *frames_written = bytes_written / (2 * sizeof(int16_t));
    return result;
}

} // namespace

extern "C" esp_err_t veronia_audio_output_init(uint32_t sample_rate_hz)
{
    if (sample_rate_hz == 0 || !ensure_mutex()) {
        return sample_rate_hz == 0 ? ESP_ERR_INVALID_ARG : ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    if (tx_channel != nullptr) {
        const esp_err_t result = active_sample_rate_hz == sample_rate_hz ? ESP_OK : ESP_ERR_INVALID_STATE;
        xSemaphoreGive(audio_mutex);
        return result;
    }

    i2s_chan_config_t channel_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel_config.dma_desc_num = 6;
    channel_config.dma_frame_num = 240;

    esp_err_t result = i2s_new_channel(&channel_config, &tx_channel, nullptr);
    if (result == ESP_OK) {
        i2s_std_config_t standard_config = {
            .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                             I2S_SLOT_MODE_STEREO),
            .gpio_cfg = {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = VERONIA_AUDIO_PIN_BCLK,
                .ws = VERONIA_AUDIO_PIN_WS,
                .dout = VERONIA_AUDIO_PIN_DOUT,
                .din = I2S_GPIO_UNUSED,
                .invert_flags = {},
            },
        };
        result = i2s_channel_init_std_mode(tx_channel, &standard_config);
    }
    if (result == ESP_OK) {
        result = i2s_channel_enable(tx_channel);
    }
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed: %s", esp_err_to_name(result));
        if (tx_channel != nullptr) {
            i2s_del_channel(tx_channel);
            tx_channel = nullptr;
        }
        xSemaphoreGive(audio_mutex);
        return result;
    }

    active_sample_rate_hz = sample_rate_hz;
    ESP_LOGI(TAG, "I2S output initialized: %lu Hz, BCLK=%d WS=%d DOUT=%d",
             static_cast<unsigned long>(sample_rate_hz), VERONIA_AUDIO_PIN_BCLK,
             VERONIA_AUDIO_PIN_WS, VERONIA_AUDIO_PIN_DOUT);
    xSemaphoreGive(audio_mutex);
    return ESP_OK;
}

extern "C" esp_err_t veronia_audio_output_deinit(void)
{
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    if (tx_channel == nullptr) {
        xSemaphoreGive(audio_mutex);
        return ESP_OK;
    }

    esp_err_t result = i2s_channel_disable(tx_channel);
    if (result == ESP_OK) {
        result = i2s_del_channel(tx_channel);
    }
    if (result == ESP_OK) {
        tx_channel = nullptr;
        active_sample_rate_hz = 0;
    }
    xSemaphoreGive(audio_mutex);
    return result;
}

extern "C" bool veronia_audio_output_is_ready(void)
{
    return tx_channel != nullptr;
}

extern "C" esp_err_t veronia_audio_output_write_frames(const int16_t *samples, size_t frame_count,
                                                         uint32_t timeout_ms, size_t *frames_written)
{
    if (samples == nullptr || frame_count == 0 || frames_written == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    *frames_written = 0;
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    const esp_err_t result = tx_channel == nullptr
        ? ESP_ERR_INVALID_STATE
        : write_frames_locked(samples, frame_count, timeout_ms, frames_written);
    xSemaphoreGive(audio_mutex);
    return result;
}

extern "C" esp_err_t veronia_audio_output_play_tone(uint32_t frequency_hz, uint32_t duration_ms,
                                                      uint8_t volume_percent)
{
    if (frequency_hz == 0 || duration_ms == 0 || volume_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!ensure_mutex()) {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(audio_mutex, portMAX_DELAY);
    if (tx_channel == nullptr) {
        xSemaphoreGive(audio_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    if (frequency_hz > active_sample_rate_hz / 2) {
        xSemaphoreGive(audio_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    int16_t tone_buffer[TONE_BUFFER_FRAME_COUNT * 2] = {};
    uint64_t remaining_frames = (static_cast<uint64_t>(active_sample_rate_hz) * duration_ms) / 1000;
    float phase = 0.0f;
    const float phase_step = TWO_PI * static_cast<float>(frequency_hz) /
                             static_cast<float>(active_sample_rate_hz);
    const float amplitude = 32767.0f * static_cast<float>(volume_percent) / 100.0f;
    esp_err_t result = ESP_OK;

    while (remaining_frames != 0 && result == ESP_OK) {
        const size_t frame_count = remaining_frames > TONE_BUFFER_FRAME_COUNT
            ? TONE_BUFFER_FRAME_COUNT : static_cast<size_t>(remaining_frames);
        float sample_phase = phase;
        for (size_t index = 0; index < frame_count; ++index) {
            const int16_t sample = static_cast<int16_t>(sinf(sample_phase) * amplitude);
            tone_buffer[index * 2] = sample;
            tone_buffer[index * 2 + 1] = sample;
            sample_phase += phase_step;
            if (sample_phase >= TWO_PI) {
                sample_phase -= TWO_PI;
            }
        }

        size_t frames_written = 0;
        result = write_frames_locked(tone_buffer, frame_count, WAIT_FOREVER, &frames_written);
        if (frames_written == 0 && result == ESP_OK) {
            result = ESP_FAIL;
        }
        phase = fmodf(phase + phase_step * static_cast<float>(frames_written), TWO_PI);
        remaining_frames -= frames_written;
    }

    xSemaphoreGive(audio_mutex);
    return result;
}
