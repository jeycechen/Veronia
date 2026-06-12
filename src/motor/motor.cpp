/*
 * @Author: 陈开龙 cklnuaa@163.com
 * @Date: 2026-04-21 10:00:50
 * @LastEditors: 陈开龙 cklnuaa@163.com
 * @LastEditTime: 2026-06-13 00:19:09
 * @FilePath: /Veronia/src/motor/motor.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "motor.h"

static const char *TAG = "motor_control";

AngleEncoder encoder(MOTOR_POLE_PAIRS);
static const int spiClk = 1000000; // 400KHz
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
        0,
        0,
        8.225806452 * _PI / 180,
        2.3,
        1,
        1.1,
        "Fine values\nWith detents\nUnbound",
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
// 限制value的值在合法的范围内
static inline float CLAMP(const float value, const float low, const float high) {
    return value < low ? low : (value > high ? high : value);
}

static void initMyAngleEncoderCallback(void) {
    if(!encoder.init(SPI2_HOST, MOSI_SDA_ENCODER, MISO_ENCODER, SCL_ENCODER, EN_CS_ENCODER,
                      AS5047P_SPI_MODE, AS5047P_SPI_CLOCK_HZ)) {
        ESP_LOGE(TAG, "AS5047P init failed");
    }
    return;
}

static float readMyAngleEncoderCallback() {
    float deg = encoder.get_mechanical_degrees(); // [0, 360)
    return (deg / 360.0) * 2 * PI; // 返回弧度
}
MotorCtrl motorCtrl;

void MotorCtrl::init(){
    encoder = GenericSensor(readMyAngleEncoderCallback, initMyAngleEncoderCallback);
    encoder.init();
    motor.linkSensor(&(this->encoder));
    driver.voltage_power_supply = MOTOR_POWER_SUPPLY;
    driver.init();
    motor.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    motor.controller = MotionControlType::torque;
    
    motor.PID_velocity.P = 1;
    motor.PID_velocity.I = 0;
    motor.PID_velocity.D = 0.01;

    motor.voltage_limit = 5;
    motor.LPF_velocity.Tf = 0.01;
    motor.velocity_limit = 10;

    motor.init();
    motor.initFOC();
}

void MotorCtrl::motorUpdate(void *pvParameters) {
    encoder.update();
    motor.loopFOC();
    
    // 计算当前的速度，使用一阶平滑滤波；
    idle_velocity = motor.shaft_velocity * IDLE_VELOCITY_EWMA_ALPHA + idle_velocity * ( 1 - IDLE_VELOCITY_EWMA_ALPHA);
    if (fabsf(idle_velocity) > IDLE_VELOCITY_RAT_PER_SEC) {
        // 说明这个时候不是怠速，可能已经发生移动了
        last_idle_start = 0;
    } else {
        // 说明这个时候是怠速；
        if(last_idle_start == 0) {
            last_idle_start = millis();
        }
    }

    // 如果上一次进入循环也是怠速，并且已经持续超过了怠速的时间阈值，并且当前角度距离虚拟档位中心不远，那么就认为这个时候是静止不动的，可以进行怠速校正；
    if (last_idle_start > 0 && millis() - last_idle_start > IDLE_CORRECTION_MILLIS_THRESHOLD
        && fabsf(motor.shaft_angle - current_detent_center < IDLE_CORRECTION_MAX_ANGLE_RAD))
    {
        current_detent_center = motor.shaft_angle * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
    }

    // 计算当前的位置到 修正后的档位中心的角度差值，需要根据电机安装位置调整       
    angle_to_current_detent_center = motor.shaft_angle - current_detent_center; 

    // 判断档位位置，是否执行跳档
    if (angle_to_current_detent_center > workConfig.position_width_radians * workConfig.snap_point
        && (workConfig.num_positions <= 0 || workConfig.position > 0)) { // 
        current_detent_center += workConfig.position_width_radians;
        angle_to_current_detent_center -= workConfig.position_width_radians;

        workConfig.position++;
    } else if (angle_to_current_detent_center < -workConfig.position_width_radians * workConfig.snap_point 
                && (workConfig.num_positions <=0 || workConfig.position < workConfig.num_positions - 1)) 
    {
        // 进入这个分支，说明是往反方向旋转
        current_detent_center -= workConfig.position_width_radians;
        angle_to_current_detent_center += workConfig.position_width_radians;

        workConfig.position--;
    }
    
    // 死区调整, 死区：在档位中心允许一定的误差，防止高频修正
    float dead_zone_adjustment = CLAMP(
        angle_to_current_detent_center,
        fmaxf(-workConfig.position_width_radians * DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
        fminf(workConfig.position_width_radians * DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD)
    );
    // 出界
    bool is_out_bound = workConfig.num_positions > 0 && 
                ((angle_to_current_detent_center > 0 && workConfig.position == 0) 
              || (angle_to_current_detent_center < 0 && workConfig.position == workConfig.num_positions - 1));
    
    motor.PID_velocity.limit = is_out_bound ? 10 : 3;
    motor.PID_velocity.P = is_out_bound ? workConfig.endstop_strength_unit * 4 : workConfig.detent_strength_unit * 4;
    
    // 处理float类型的绝对值
    if (fabsf(motor.shaft_velocity) > 60)
    {
        motor.move(0);
    } else {
        float torque = motor.PID_velocity(-angle_to_current_detent_center + dead_zone_adjustment);
        motor.move(torque);
    }
}

// 震动反馈
void MotorCtrl::shake_motor(int strength, int delay_time){
    motor.move(strength);
    for(int i = 0;i < delay_time; i++) {
        motor.loopFOC();
        vTaskDelay(1);
    }
    motor.move(-strength);
    for(int i = 0;i < delay_time; i++) {
        motor.loopFOC();
        vTaskDelay(1);
    }
}

// 指向某个角度
void MotorCtrl::move_motor_to_angle(float angle, float velocity){

}

// 发布当前foc的状态，供其他组件取用；
void MotorCtrl::publish_motor_status(bool is_outbound){
    
}

// 更新foc的运转模式
void MotorCtrl::update_motor_runmode(int mode, int init_position){
    workConfig = motor_configs[mode];
    workConfig.position = init_position;
    current_detent_center = -motor.shaft_angle;

    shake_motor(2, 2);
}

extern "C" void veronia_motor_task(void *pvParameters) {
    motorCtrl.init();

    while (1) {
        motorCtrl.motorUpdate(pvParameters);
        vTaskDelay(1);
        printf(">>> Motor control task running on core %d <<<\n", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(100)); // 每100ms更新一次
    }
}
