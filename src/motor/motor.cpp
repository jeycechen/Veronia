/*
 * @Author: 陈开龙 cklnuaa@163.com
 * @Date: 2026-04-21 10:00:50
 * @LastEditors: 陈开龙 cklnuaa@163.com
 * @LastEditTime: 2026-05-16 21:00:06
 * @FilePath: /Veronia/src/motor/motor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stdio.h"
static const int spiClk = 1000000; // 400KHz

class MotorCtrl {
    public:
    void init() {};
    MotorCtrl() {};
};

void veronia_motor_task(void *pvParameters) {
    MotorCtrl motorCtrl;
    motorCtrl.init();

    while (1) {
        // 这里可以添加电机控制逻辑，例如根据 target_velocity 调整 motor 的速度
        // motor.move(target_velocity);
        printf(">>> Motor control task running on core %d <<<\n", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次
    }
}
