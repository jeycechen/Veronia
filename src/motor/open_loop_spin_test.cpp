/*
 * @Author: ChenCalm cklnuaa@163.com
 * @Date: 2026-06-04 23:14:03
 * @LastEditors: ChenCalm cklnuaa@163.com
 * @LastEditTime: 2026-06-07 11:27:39
 * @FilePath: \Veronia\src\motor\open_loop_spin_test.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "esp_log.h"
#include "esp_simplefoc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor/pins.h"

static const char *TAG = "motor_openloop";

static constexpr int MOTOR_POLE_PAIRS = 7;
static constexpr float POWER_SUPPLY_VOLTAGE = 12.0f;
static constexpr float DRIVER_VOLTAGE_LIMIT = 3.0f;
static constexpr float MOTOR_VOLTAGE_LIMIT = 2.0f;
static constexpr float OPEN_LOOP_TARGET_VELOCITY = 2.0f;

extern "C" void veronia_motor_task(void *pvParameters)
{
    (void)pvParameters;

    ESP_LOGI(TAG, "Starting 6PWM open-loop motor test");
    ESP_LOGI(TAG, "UH=%d UL=%d VH=%d VL=%d WH=%d WL=%d EN=%d",
             TMC_UH, TMC_UL, TMC_VH, TMC_VL, TMC_WH, TMC_WL, EN_MOTOR);

    BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
    BLDCDriver6PWM driver = BLDCDriver6PWM(
        TMC_UH, TMC_UL,
        TMC_VH, TMC_VL,
        TMC_WH, TMC_WL,
        EN_MOTOR);

    SimpleFOCDebug::enable();
    Serial.begin(115200);

    driver.voltage_power_supply = POWER_SUPPLY_VOLTAGE;
    driver.voltage_limit = DRIVER_VOLTAGE_LIMIT;
    driver.dead_zone = 0.04f;

    if (driver.init() != 1) {
        ESP_LOGE(TAG, "BLDCDriver6PWM init failed, motor output disabled");
        vTaskDelete(nullptr);
        return;
    }

    motor.linkDriver(&driver);
    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
    const TickType_t loop_delay = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
    motor.velocity_limit = 10.0f;

    motor.init();
    driver.enable();

    ESP_LOGI(TAG, "Open-loop running: target velocity %.2f rad/s, voltage limit %.2f V",
             OPEN_LOOP_TARGET_VELOCITY, MOTOR_VOLTAGE_LIMIT);

    uint32_t tick = 0;
    while (true) {
        motor.move(OPEN_LOOP_TARGET_VELOCITY);

        if ((tick++ % 1000) == 0) {
            ESP_LOGI(TAG, "open-loop tick, core=%d", xPortGetCoreID());
        }

        vTaskDelay(pdMS_TO_TICKS(1));
        vTaskDelay(loop_delay);
    }
}