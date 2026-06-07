/*
 * @Author: 陈开龙 cklnuaa@163.com
 * @Date: 2026-04-20 22:19:37
 * @LastEditors: ChenCalm cklnuaa@163.com
 * @LastEditTime: 2026-06-02 22:22:54
 * @FilePath: /Veronia/main/main.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "veronia_ui.h"
#include "motor.h"

static const char *TAG = "Veronia_Test";

void task_on_core(void *pvParameters) {
    int core_id = (int)pvParameters;
    while (1) {
        // 使用 printf 强制测试，不受 ESP_LOG 级别限制
        printf(">>> Hello from Core %d <<<\n", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

extern "C" void app_main(void) {
    printf("--- app_main Start ---\n");

    // 创建任务并固定到核心 0
    // xTaskCreatePinnedToCore(task_on_core, "Task0", 4096, (void *)0, 10, NULL, 0);

    // 创建任务并固定到核心 1
    // xTaskCreatePinnedToCore(veronia_ui_task, "veronia ui task", 4096, (void *)1, 10, NULL, 1);

    xTaskCreatePinnedToCore(veronia_motor_task, "verinia motor task", 4096, (void *)1, 10, NULL, 1);
    // xTaskCreatePinnedToCore(veronia_encoder_task, "as5047p task", 4096, (void *)1, 10, NULL, 1);

    // xTaskCreatePinnedToCore(veronia_smart_knob_task, "veronia_smart_knob_task", 4096, (void *)1, 10, NULL, 1);
    
    
    while (1) {
        ESP_LOGI(TAG, "Main task is still alive on core %d", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
