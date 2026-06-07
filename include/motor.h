/*
 * @Author: 陈开龙 cklnuaa@163.com
 * @Date: 2026-04-21 10:00:59
 * @LastEditors: 陈开龙 cklnuaa@163.com
 * @LastEditTime: 2026-05-16 20:57:18
 * @FilePath: /Veronia/include/motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef VERONIA_MOTOR_H
#define VERONIA_MOTOR_H

#define MOTOR_POLE_PRIRS    7

#ifdef __cplusplus
extern "C" {
#endif

#include "config.h"
// #include <SimpleFOC.h>

void veronia_motor_task(void *pvParameters);
void veronia_encoder_task(void *pvParameters);
void veronia_smart_knob_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
