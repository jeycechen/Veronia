/*
 * @Author: ChenCalm cklnuaa@163.com
 * @Date: 2026-05-23 18:20:32
 * @LastEditors: ChenCalm cklnuaa@163.com
 * @LastEditTime: 2026-05-24 13:14:08
 * @FilePath: \Veronia\src\motor\motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"
#define PI  3.14159

typedef struct MotorConfig {
    // 可以运动的个数
    int32_t num_positions;        
    // 位置
    int32_t position;             
    // 位置宽度弧度 或者是每一步的度数
    float position_width_radians; 
    // 正常旋转时的制动强度
    float detent_strength_unit;  
    // 超出界限后的制动强度
    float endstop_strength_unit;  
    // 每一步弧度的放大值
    float snap_point; 
    // 描述符            
    char descriptor[50];          
}MotorConfig;


static MotorConfig motor_configs[] = {
    [MOTOR_UNBOUND_FINE_DETENTS] = {
        0,
        0,
        1 * PI / 180,
        2,
        1,
        1.1,
        "Fine values\nWith detents", //任意运动的控制  有阻尼 类似于机械旋钮
    },
    [MOTOR_UNBOUND_NO_DETENTS] = {
        0,
        0,
        1 * PI / 180,
        0,
        0.1,
        1.1,
        "Unbounded\nNo detents", //无限制  不制动
    },
    [MOTOR_SUPER_DIAL] = {
        0,
        0,
        5 * PI / 180,
        2,
        1,
        1.1,
        "Super Dial", //无限制  不制动
    },
    [MOTOR_UNBOUND_COARSE_DETENTS] = {
        .num_positions = 0,
        .position = 0,
        .position_width_radians = 8.225806452 * _PI / 180,
        .detent_strength_unit = 2.3,
        .endstop_strength_unit = 1,
        .snap_point = 1.1,
        "Fine values\nWith detents\nUnbound"
    },
    [MOTOR_BOUND_0_12_NO_DETENTS]= {
        13,
        0,
        10 * PI / 180,
        0,
        1,
        1.1,
        "Bounded 0-13\nNo detents",
    },
    [MOTOR_BOUND_LCD_BK_BRIGHTNESS]= {
        101,
        10,
        2 * PI / 180,
        2,
        1,
        1.1,
        "Bounded 0-101\nNo detents",
    },
    [MOTOR_BOUND_LCD_BK_TIMEOUT]= {
        31,
        0,
        5 * PI / 180,
        2,
        1,
        1.1,
        "Bounded 0-3601\nNo detents",
    },
    [MOTOR_COARSE_DETENTS] = {
        32,
        0,
        8.225806452 * PI / 180,
        2,
        1,
        1.1,
        "Coarse values\nStrong detents", //粗糙的棘轮 强阻尼
    },

    [MOTOR_FINE_NO_DETENTS] = {
        256,
        127,
        1 * PI / 180,
        0,
        1,
        1.1,
        "Fine values\nNo detents", //任意运动的控制  无阻尼
    },
    [MOTOR_ON_OFF_STRONG_DETENTS] = {
        2, 
        0,
        60 * PI / 180, 
        1,             
        1,
        0.55,                    // Note the snap point is slightly past the midpoint (0.5); compare to normal detents which use a snap point *past* the next value (i.e. > 1)
        "On/off\nStrong detent", //模拟开关  强制动
    },

};

typedef enum
{
    MOTOR_UNBOUND_FINE_DETENTS,        // Fine values\nWith detents
    MOTOR_UNBOUND_NO_DETENTS,
    MOTOR_SUPER_DIAL, 
    MOTOR_UNBOUND_COARSE_DETENTS, // Coarse values\nStrong detents\n unbound
    MOTOR_BOUND_0_12_NO_DETENTS,
    MOTOR_BOUND_LCD_BK_BRIGHTNESS,
    MOTOR_BOUND_LCD_BK_TIMEOUT,
    MOTOR_COARSE_DETENTS,       // Coarse values\nStrong detents
    MOTOR_FINE_NO_DETENTS,     // Fine values\nNo detents
    MOTOR_ON_OFF_STRONG_DETENTS,             // "On/off\nStrong detent"
    MOTOR_MAX_MODES, //

} MOTOR_RUNNING_MODE_ENUM;

#endif