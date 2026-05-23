/*
 * @Author: 陈开龙 cklnuaa@163.com
 * @Date: 2026-04-25 19:45:37
 * @LastEditors: 陈开龙 cklnuaa@163.com
 * @LastEditTime: 2026-05-16 18:30:16
 * @FilePath: /Veronia/src/veronia_ui.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
extern "C" { // 这里是为了编译cpp的时候引用c风格的符号能够找到，所以c文件的头文件也需要extern c
#include "stdio.h"
#include "veronia_ui.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
}

class VeroniaUI {
    public:
        VeroniaUI() {};
        void init() {};
};

void veronia_ui_task(void *pvParameters) {
    VeroniaUI ui;
    ui.init();

    int core_id = (int)pvParameters;
    while (1) {
        // 使用 printf 强制测试，不受 ESP_LOG 级别限制
        printf(">>> Hello Veronia face >_< %d <<<\n", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // lv_timer_handler(); /* Handle LVGL tasks */
    // vTaskDelay(pdMS_TO_TICKS(5)); /* Short delay for the RTOS scheduler */
}