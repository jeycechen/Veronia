#include "motor/angleencoder.h"
#include <math.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "AS5047P";
static constexpr float PI_F = 3.14159265359f;

AngleEncoder::AngleEncoder(uint8_t pole_pairs)
    : pole_pairs_(pole_pairs)
{
}

AngleEncoder::~AngleEncoder()
{
    if (spi_initialized_ && spi_handle_ != nullptr) {
        spi_bus_remove_device(spi_handle_);
    }
}

bool AngleEncoder::init(spi_host_device_t spi_host, int mosi_io, int miso_io, int sclk_io, int cs_io,
                        int spi_mode, int clock_speed_hz)
{
    // 如果 SPI 总线未初始化，进行初次分配
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = mosi_io;
    bus_cfg.miso_io_num = miso_io;
    bus_cfg.sclk_io_num = sclk_io;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 32;

    esp_err_t ret = spi_bus_initialize(spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = clock_speed_hz;
    dev_cfg.mode = spi_mode;
    dev_cfg.spics_io_num = cs_io;
    dev_cfg.queue_size = 1;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY;
    // 【关键修复】：赋予 CS 引脚预传输建立时间，确保时序对齐，彻底告别局部 INVCOMM 错误
    dev_cfg.cs_ena_pretrans = 2; 

    ret = spi_bus_add_device(spi_host, &dev_cfg, &spi_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI add AS5047P failed: %s", esp_err_to_name(ret));
        return false;
    }

    spi_initialized_ = true;

    // 清除上电可能存在的历史残留脏数据
    bool ok = false;
    (void)read_register(REG_ERRFL, &ok);
    if (ok) {
        ESP_LOGI(TAG, "Initial ERRFL=0x%04X", last_error_flags_);
    }
    
    // 预读一次角度填满二级缓冲管道
    (void)read_register(REG_ANGLECOM, &ok);

    ESP_LOGI(TAG, "AS5047P initialized on SPI host %d, mode=%d, clock=%d Hz",
             spi_host, spi_mode, clock_speed_hz);
    return true;
}

bool AngleEncoder::update(float dt)
{
    if (!spi_initialized_) {
        return false;
    }

    bool ok = false;
    uint16_t raw = read_register(REG_ANGLECOM, &ok);
    
    // 如果通信带有错位或 EF 报错，直接就地拦截，保护 FOC 控制
    if (!ok || last_error_bit_) {
        if ((error_log_count_++ % 20) == 0) {
            // 通过独立的、不影响主流水线的轻量流程清除并获取错误标志
            bool err_ok = false;
            (void)read_register(REG_ERRFL, &err_ok);
            ESP_LOGW(TAG, "Communication link blocked! Clear status. ERRFL=0x%04X", last_error_flags_);
        }
        return false; 
    }

    raw_data_ = raw & 0x3FFF;

    const float current_angle = ((float)raw_data_ / 16384.0f) * TWO_PI;
    mechanical_angle_ = normalize_angle(current_angle - zero_offset_);
    electrical_angle_ = normalize_angle(mechanical_angle_ * (float)pole_pairs_);

    if (dt > 0.0f) {
        float delta = mechanical_angle_ - last_mechanical_angle_;
        if (delta > PI_F) {
            delta -= TWO_PI;
        } else if (delta < -PI_F) {
            delta += TWO_PI;
        }

        const float raw_velocity = delta / dt;
        velocity_ = velocity_filter_alpha_ * raw_velocity +
                    (1.0f - velocity_filter_alpha_) * velocity_;
    }

    last_mechanical_angle_ = mechanical_angle_;
    return true;
}

bool AngleEncoder::calibrate_zero_offset()
{
    if (!spi_initialized_) {
        return false;
    }

    bool ok = false;
    uint16_t raw = read_register(REG_ANGLECOM, &ok);
    if (!ok) {
        return false;
    }

    zero_offset_ = ((float)(raw & 0x3FFF) / 16384.0f) * TWO_PI;
    last_mechanical_angle_ = 0.0f;
    velocity_ = 0.0f;
    ESP_LOGI(TAG, "Zero offset calibrated: %.4f rad", zero_offset_);
    return true;
}

bool AngleEncoder::read_error_flags()
{
    if (!spi_initialized_) {
        return false;
    }

    bool ok = false;
    (void)read_register(REG_ERRFL, &ok);
    return ok;
}

void AngleEncoder::set_zero_offset(float offset)
{
    zero_offset_ = normalize_angle(offset);
}

uint16_t AngleEncoder::read_register(uint16_t address, bool *ok)
{
    if (ok != nullptr) {
        *ok = false;
    }

    const uint16_t command = make_read_command(address);
    last_command_ = command;

    // 发送目标读取命令
    transfer16(command);
    // 发送空操作(NOP)，移出并在当前帧接收真正的数据
    uint16_t response = transfer16(CMD_NOP);
    last_response_ = response;

    const bool error = (response & 0x4000) != 0;
    last_error_bit_ = error;
    
    const bool parity_ok = has_even_parity(response);
    if (!parity_ok) {
        return 0;
    }

    if (address == REG_ERRFL) {
        last_error_flags_ = response & 0x3FFF;
    }

    if (ok != nullptr) {
        *ok = !error; 
    }

    return response & 0x3FFF;
}

uint16_t AngleEncoder::transfer16(uint16_t frame)
{
    uint8_t tx_buffer[2];
    uint8_t rx_buffer[2] = {0};

    // 【标准修复】：显式转换大端序，确保数据在硬件总线上顺序百分之百符合逻辑
    tx_buffer[0] = (frame >> 8) & 0xFF;
    tx_buffer[1] = frame & 0xFF;

    spi_transaction_t transaction = {};
    transaction.length = 16; 
    transaction.tx_buffer = tx_buffer;
    transaction.rx_buffer = rx_buffer;

    esp_err_t ret = spi_device_transmit(spi_handle_, &transaction);
    if (ret != ESP_OK) {
        return 0;
    }

    return ((uint16_t)rx_buffer[0] << 8) | rx_buffer[1];
}

uint16_t AngleEncoder::make_read_command(uint16_t address)
{
    uint16_t frame = 0x4000 | (address & 0x3FFF);
    if (!has_even_parity(frame)) {
        frame |= 0x8000;
    }
    return frame;
}

bool AngleEncoder::has_even_parity(uint16_t value)
{
    value ^= value >> 8;
    value ^= value >> 4;
    value ^= value >> 2;
    value ^= value >> 1;
    return (value & 1) == 0;
}

float AngleEncoder::normalize_angle(float angle)
{
    while (angle >= TWO_PI) {
        angle -= TWO_PI;
    }
    while (angle < 0.0f) {
        angle += TWO_PI;
    }
    return angle;
}