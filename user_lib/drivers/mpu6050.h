#ifndef MPU6050_H
#define MPU6050_H

#include "bus/i2c_bus.h"
#include <stdint.h>

struct mpu6050_sample
{
    uint32_t sequence = 0U;
    uint32_t timestamp_us = 0U;
    float temperature_c = 0.0f;
    float acceleration_g[3]{};
    float angular_velocity_rad_s[3]{};
    float angle_rad[3]{};
};

class mpu6050
{
    public:
        mpu6050(uint8_t i2c_bus_id,
            uint8_t device_address,
            float accelerometer_weight);

    public:
        i2c_result init(bool calibrate_gyroscope = false);

    public:
        i2c_result update();
        const mpu6050_sample &sample() const;

    private:
        i2c_result read_registers(uint8_t register_address,
            uint8_t *data,
            uint16_t size);
        i2c_result write_register(uint8_t register_address, uint8_t value);
        i2c_result calibrate_gyroscope_offset();
        void process_raw_sample(uint32_t timestamp_us);

    private:
        i2c_bus i2c;
        uint8_t device_address;
        float accelerometer_weight;
        uint8_t raw_sample[14]{};
        float gyroscope_offset_rad_s[3]{};
        mpu6050_sample current_sample{};
        uint32_t previous_timestamp_us = 0U;
        bool initialized = false;
        bool first_sample = true;
};

#endif
