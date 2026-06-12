/*
 * @Author: ChenCalm cklnuaa@163.com
 * @Date: 2026-05-23 18:20:32
 * @LastEditors: 陈开龙 cklnuaa@163.com
 * @LastEditTime: 2026-06-12 23:31:03
 * @FilePath: \Veronia\src\motor\motor.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "stdio.h"
#include "esp_simplefoc.h"
#include "esp_timer.h"
#include "angleencoder.h"
#include "pins.h"
#include "math.h"


static constexpr uint8_t MOTOR_POLE_PAIRS = 7;
static constexpr int AS5047P_SPI_MODE = 1;
static constexpr int AS5047P_SPI_CLOCK_HZ = 100 * 1000;
static constexpr int MOTOR_POWER_SUPPLY = 5; // 5V供电

// 怠速
static constexpr float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
static constexpr float IDLE_VELOCITY_RAT_PER_SEC = 0.05;
static constexpr uint32_t IDLE_CORRECTION_MILLIS_THRESHOLD = 500U; //只有怠速超过500ms，才认为当前是静止不动的
static constexpr float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180; // 怠速校正最大角度，5度
static constexpr float IDLE_CORRECTION_RATE_ALPHA = 0.0005; // 怠速修正率


// 死区
static constexpr float DEAD_ZONE_DETENT_PERCENT = 0.2; // 死区制动百分率
static constexpr float DEAD_ZONE_RAD = 1 * PI / 180;  // 死区调整角度


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
} MOTOR_WORK_MODE_ENUM;

class MotorCtrl {
    private:
        MOTOR_WORK_MODE_ENUM workMode;
        GenericSensor encoder;
        BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);;
        BLDCDriver6PWM driver = BLDCDriver6PWM(TMC_UH, TMC_UL, TMC_VH, TMC_VL, TMC_WH, TMC_WL);
        MotorConfig workConfig;
        float current_detent_center = 0; // 虚拟的档位中心位置，在怠速情况下，需要拖拽到这个位置
        float angle_to_current_detent_center = 0; // 电机当前角度到虚拟档位中心的角度差
        float idle_velocity = 0;
        unsigned long last_idle_start = 0;
    public:
        MotorCtrl(){};
        ~MotorCtrl(){};
        void init();
        // 震动反馈
        void shake_motor(int strength, int delay_time);
        // 指向某个角度
        void move_motor_to_angle(float angle, float velocity = 5);
        // 更新foc的运转模式
        void update_motor_runmode(int mode, int init_position);
        // 发布当前foc的状态，供其他组件取用；
        void publish_motor_status(bool is_outbound);
        void motorUpdate(void *pvParameters);
};


#endif