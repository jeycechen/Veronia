#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "motor/pins.h"
#include "motor/angleencoder.h"

static const char *TAG = "smart_knob";

// ====================================================================
// 📊 对齐 X-Knob 核心 PID 控制环与磁性跳转参数
// ====================================================================
static constexpr uint8_t MOTOR_POLE_PAIRS = 7; 
static constexpr float PI_F = 3.14159265359f;
static constexpr float TWO_PI_F = 6.28318530718f;

// 1. 划分 3 个档位（每档 120 度）
static constexpr float DETENT_SECTOR = TWO_PI_F / 3.0f; // 120° = 2.0944 rad

// 2. 磁性开关触发硬阈值：15 度
static constexpr float TRIGGER_THRESHOLD_RAD = 15.0f * (PI_F / 180.0f); // 15° = 0.2618 rad

// 3. 🎯【完全对齐 X-Knob 闭环位置环 PID 标准核心值】
static constexpr float X_PID_KP = 20.0f;    // 强力比例：产生极其陡峭、坚硬的阻力物理隔离墙
static constexpr float X_PID_KI = 0.5f;     // 强力积分：零死区、瞬间吃掉摩擦力，提供强磁吸合感
static constexpr float X_PID_KD = 0.45f;    // 强力微分：提供完美的吸入反向刹车阻尼，防止啪嗒后震荡

static constexpr float X_INTEGRAL_LIMIT = 1.0f; // 积分饱和限幅
static constexpr float X_VOLTAGE_LIMIT  = 2.0f; // Q轴等效发力电压绝对限制（拉满力矩）

// ====================================================================
// 🔋 底层物理 H 桥驱动引擎
// ====================================================================
static inline void set_bridge_states(int uh, int ul, int vh, int vl, int wh, int wl) {
    gpio_set_level((gpio_num_t)TMC_UH, uh); gpio_set_level((gpio_num_t)TMC_UL, ul);
    gpio_set_level((gpio_num_t)TMC_VH, vh); gpio_set_level((gpio_num_t)TMC_VL, vl);
    gpio_set_level((gpio_num_t)TMC_WH, wh); gpio_set_level((gpio_num_t)TMC_WL, wl);
}

// 完美承接位置 PID 计算结果的等效空间换相逻辑
void apply_motor_voltage_q(float v_q, float electrical_angle) {
    // 1. 电压幅值严格限幅
    if (v_q > X_VOLTAGE_LIMIT)  v_q = X_VOLTAGE_LIMIT;
    if (v_q < -X_VOLTAGE_LIMIT) v_q = -X_VOLTAGE_LIMIT;

    // 2. 微幅无发力死区清空
    if (fabsf(v_q) < 0.05f) {
        set_bridge_states(0,0, 0,0, 0,0);
        return;
    }

    // 3. Q轴磁场垂直正交变换 (+- 90度电角度换相)
    float drive_angle = electrical_angle + (v_q >= 0.0f ? (-PI_F / 2.0f) : (PI_F / 2.0f));
    
    while (drive_angle < 0.0f) drive_angle += TWO_PI_F;
    while (drive_angle >= TWO_PI_F) drive_angle -= TWO_PI_F;

    // 4. 高频空间 12-Sector 电压矢量控制（逼真正弦控制效果）
    int sector_12 = (int)(drive_angle / (PI_F / 6.0f)) % 12;
    if (sector_12 < 0) sector_12 += 12;

    switch (sector_12) {
        case 0:  set_bridge_states(1,0, 0,1, 0,0); break; // U+ V-
        case 1:  set_bridge_states(1,0, 0,1, 0,1); break; // U+ V- W-
        case 2:  set_bridge_states(1,0, 0,0, 0,1); break; // U+ W-
        case 3:  set_bridge_states(1,0, 1,0, 0,1); break; // U+ V+ W-
        case 4:  set_bridge_states(0,0, 1,0, 0,1); break; // V+ W-
        case 5:  set_bridge_states(0,1, 1,0, 0,1); break; // V+ U- W-
        case 6:  set_bridge_states(0,1, 1,0, 0,0); break; // V+ U-
        case 7:  set_bridge_states(0,1, 1,0, 1,0); break; // V+ W+ U-
        case 8:  set_bridge_states(0,1, 0,0, 1,0); break; // W+ U-
        case 9:  set_bridge_states(0,1, 0,1, 1,0); break; // W+ U- V-
        case 10: set_bridge_states(0,0, 0,1, 1,0); break; // W+ V-
        case 11: set_bridge_states(1,0, 0,1, 1,0); break; // W+ U+ V-
    }
}

// ====================================================================
// 🚀 智能旋钮主任务（完全闭环 PID 位置环状态机）
// ====================================================================
extern "C" void veronia_smart_knob_task(void *pvParameters) {
    (void)pvParameters;
    ESP_LOGI(TAG, "Smart Knob Booting... X-Knob PID [Active Low Enable] Loop Engaged.");

    // 初始化硬件 IO
    gpio_config_t motor_io_conf = {};
    motor_io_conf.intr_type = GPIO_INTR_DISABLE;
    motor_io_conf.mode = GPIO_MODE_OUTPUT;
    motor_io_conf.pin_bit_mask = (1ULL << EN_MOTOR) | 
                                 (1ULL << TMC_UH) | (1ULL << TMC_VH) | (1ULL << TMC_WH) |
                                 (1ULL << TMC_UL) | (1ULL << TMC_VL) | (1ULL << TMC_WL);
    gpio_config(&motor_io_conf);
    
    // 💡【核心修正】：使能信号直接取反！改为向驱动芯片输出低电平（0）以强制开启硬件驱动
    gpio_set_level((gpio_num_t)EN_MOTOR, 0); 

    // 初始化编码器
    AngleEncoder encoder(MOTOR_POLE_PAIRS);
    if (!encoder.init(SPI2_HOST, MOSI_SDA_ENCODER, MISO_ENCODER, SCL_ENCODER, EN_CS_ENCODER)) {
        ESP_LOGE(TAG, "AS5047P Encoder Init Failed!");
        // 初始化失败时拉高使能，切断电机
        gpio_set_level((gpio_num_t)EN_MOTOR, 1); 
        vTaskDelete(nullptr);
        return;
    }

    // 强物理零位定向（此时应该有很大吸力把你往 0 度扯）
    set_bridge_states(1,0, 0,1, 0,0);
    vTaskDelay(pdMS_TO_TICKS(400));
    encoder.calibrate_zero_offset(); 
    set_bridge_states(0,0, 0,0, 0,0); 
    ESP_LOGI(TAG, "Calibration Completed. Tracking True PID Target...");

    int64_t last_time_us = esp_timer_get_time();
    uint32_t print_divider = 0;

    // X-Knob 位置环核心记忆器
    float virtual_target_idx = 0.0f; // 锁定在 0, 1, 2 档位索引
    float integral_accumulator = 0.0f;

    while (true) {
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_time_us) * 1e-6f;
        last_time_us = now_us;

        if (dt <= 0.0f) dt = 0.001f;

        if (!encoder.update(dt)) {
            set_bridge_states(0,0, 0,0, 0,0);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        float mech_angle = encoder.get_mechanical_angle(); 
        float elec_angle = encoder.get_electrical_angle(); 
        float velocity   = encoder.get_velocity();          
        float mech_deg   = mech_angle * 57.2957795131f;

        // 1. 获取当前锁定的目标中心绝对弧度
        float target_center_angle = virtual_target_idx * DETENT_SECTOR;
        float angle_error = mech_angle - target_center_angle;

        // 2. 圆环误差 -PI 到 +PI 映射
        while (angle_error > PI_F)  angle_error -= TWO_PI_F;
        while (angle_error < -PI_F) angle_error += TWO_PI_F;

        // 3. 💡【状态机跳转层】：当手拧角度越过 15 度阈值
        if (angle_error > TRIGGER_THRESHOLD_RAD) {
            virtual_target_idx += 1.0f; 
            target_center_angle = virtual_target_idx * DETENT_SECTOR;
            angle_error = mech_angle - target_center_angle;
            integral_accumulator = 0.0f; // 发生过线弹射时清空积分累加，防止过冲
        } 
        else if (angle_error < -TRIGGER_THRESHOLD_RAD) {
            virtual_target_idx -= 1.0f; 
            target_center_angle = virtual_target_idx * DETENT_SECTOR;
            angle_error = mech_angle - target_center_angle;
            integral_accumulator = 0.0f;
        }

        // 保证索引锁定在 3 档内循环（0, 1, 2）
        if (virtual_target_idx >= 3.0f) virtual_target_idx -= 3.0f;
        if (virtual_target_idx < 0.0f)  virtual_target_idx += 3.0f;

        // 再次归一化误差
        while (angle_error > PI_F)  angle_error -= TWO_PI_F;
        while (angle_error < -PI_F) angle_error += TWO_PI_F;

        // ====================================================================
        // 🛠️ 标准全闭环位置 PID 控制律计算（X-Knob 经典参数注入）
        // ====================================================================
        
        // P：位置比例项
        float p_term = -X_PID_KP * angle_error;

        // I：位置积分项
        integral_accumulator += angle_error * dt;
        if (integral_accumulator > X_INTEGRAL_LIMIT)  integral_accumulator = X_INTEGRAL_LIMIT;
        if (integral_accumulator < -X_INTEGRAL_LIMIT) integral_accumulator = -X_INTEGRAL_LIMIT;
        float i_term = -X_PID_KI * integral_accumulator;

        // D：位置微分项
        float d_term = -X_PID_KD * velocity;

        // 计算最终的闭环等效控制电压 V_Q
        float target_voltage_q = p_term + i_term + d_term;

        // ====================================================================

        if (print_divider++ % 40 == 0) {
            ESP_LOGI(TAG, "DEG: %5.1f° | Target: %.0f | ErrorDeg: %5.1f° | V_Q: %5.2f", 
                     mech_deg, virtual_target_idx, angle_error * 57.2957795131f, target_voltage_q);
        }

        // 传递给底层的 12 微步换相发力模块
        apply_motor_voltage_q(target_voltage_q, elec_angle);
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}