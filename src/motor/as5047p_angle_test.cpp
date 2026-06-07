#include "motor/angleencoder.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor/pins.h"

static const char *TAG = "encoder_task";
static constexpr uint8_t MOTOR_POLE_PAIRS = 7;
static constexpr int AS5047P_SPI_MODE = 1;
static constexpr int AS5047P_SPI_CLOCK_HZ = 100 * 1000;

extern "C" void veronia_encoder_task(void *pvParameters)
{
    (void)pvParameters;

    AngleEncoder encoder(MOTOR_POLE_PAIRS);
    if (!encoder.init(SPI2_HOST, MOSI_SDA_ENCODER, MISO_ENCODER, SCL_ENCODER, EN_CS_ENCODER,
                      AS5047P_SPI_MODE, AS5047P_SPI_CLOCK_HZ)) {
        ESP_LOGE(TAG, "AS5047P init failed");
        vTaskDelete(nullptr);
        return;
    }

    int64_t last_time_us = esp_timer_get_time();
    uint32_t sample_count = 0;
    while (true) {
        const int64_t now_us = esp_timer_get_time();
        const float dt = (float)(now_us - last_time_us) * 1e-6f;
        last_time_us = now_us;

        if (encoder.update(dt)) {
            ESP_LOGI(TAG, "raw=%u mech=%.2f deg elec=%.2f rad vel=%.2f rad/s ef=%d resp=0x%04X cmd=0x%04X",
                     encoder.get_raw_data(),
                     encoder.get_mechanical_degrees(),
                     encoder.get_electrical_angle(),
                     encoder.get_velocity(),
                     encoder.get_last_error_bit(),
                     encoder.get_last_response(),
                     encoder.get_last_command());
            if (encoder.get_last_error_bit() && ((sample_count % 10) == 0)) {
                encoder.read_error_flags();
                ESP_LOGW(TAG, "AS5047P EF bit set, ERRFL=0x%04X", encoder.get_last_error_flags());
            }
        } else {
            ESP_LOGW(TAG, "AS5047P read failed, err=0x%04X", encoder.get_last_error_flags());
        }

        sample_count++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
