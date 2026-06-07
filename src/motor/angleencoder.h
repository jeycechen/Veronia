/*
 * @Author: ChenCalm cklnuaa@163.com
 * @Date: 2026-05-30 21:40:39
 * @LastEditors: ChenCalm cklnuaa@163.com
 * @LastEditTime: 2026-05-31 00:05:00
 * @FilePath: \Veronia\src\motor\angleencoder.h
 */
#ifndef ANGLE_ENCODER_H
#define ANGLE_ENCODER_H

#include <stdint.h>
#include "driver/spi_master.h"

class AngleEncoder {
public:
    explicit AngleEncoder(uint8_t pole_pairs);
    ~AngleEncoder();

    // 默认时钟降低到更稳健的 1MHz，默认模式为 1
    bool init(spi_host_device_t spi_host, int mosi_io, int miso_io, int sclk_io, int cs_io,
              int spi_mode = 1, int clock_speed_hz = 1 * 1000 * 1000);
    bool update(float dt);

    float get_mechanical_angle() const { return mechanical_angle_; }
    float get_mechanical_degrees() const { return mechanical_angle_ * 57.2957795131f; }
    float get_electrical_angle() const { return electrical_angle_; }
    float get_velocity() const { return velocity_; }
    uint16_t get_raw_data() const { return raw_data_; }
    uint16_t get_last_error_flags() const { return last_error_flags_; }
    uint16_t get_last_command() const { return last_command_; }
    uint16_t get_last_response() const { return last_response_; }
    bool get_last_error_bit() const { return last_error_bit_; }
    bool is_ready() const { return spi_initialized_; }

    bool calibrate_zero_offset();
    bool read_error_flags();
    void set_zero_offset(float offset);
    float get_zero_offset() const { return zero_offset_; }

private:
    static constexpr uint16_t REG_ERRFL = 0x0001;
    static constexpr uint16_t REG_ANGLECOM = 0x3FFF;
    static constexpr uint16_t CMD_NOP = 0x0000;
    static constexpr float TWO_PI = 6.28318530718f;

    spi_device_handle_t spi_handle_ = nullptr;
    bool spi_initialized_ = false;

    uint8_t pole_pairs_ = 0;
    uint16_t raw_data_ = 0;
    uint16_t last_error_flags_ = 0;
    uint16_t last_command_ = 0;
    uint16_t last_response_ = 0;
    bool last_error_bit_ = false;
    uint32_t error_log_count_ = 0;

    float zero_offset_ = 0.0f;
    float mechanical_angle_ = 0.0f;
    float electrical_angle_ = 0.0f;
    float last_mechanical_angle_ = 0.0f;
    float velocity_ = 0.0f;
    float velocity_filter_alpha_ = 0.15f;

    uint16_t read_register(uint16_t address, bool *ok = nullptr);
    uint16_t transfer16(uint16_t frame);
    static uint16_t make_read_command(uint16_t address);
    static bool has_even_parity(uint16_t value);
    static float normalize_angle(float angle);
};

#endif