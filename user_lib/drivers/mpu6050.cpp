#include "mpu6050.h"

#include "system/sys_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

static constexpr uint8_t MPU6050_WHO_AM_I_REGISTER = 0x75;
static constexpr uint8_t MPU6050_EXPECTED_ID = 0x68;
static constexpr uint8_t MPU6050_SAMPLE_RATE_DIVIDER_REGISTER = 0x19;
static constexpr uint8_t MPU6050_CONFIG_REGISTER = 0x1A;
static constexpr uint8_t MPU6050_GYROSCOPE_CONFIG_REGISTER = 0x1B;
static constexpr uint8_t MPU6050_ACCELEROMETER_CONFIG_REGISTER = 0x1C;
static constexpr uint8_t MPU6050_ACCELEROMETER_DATA_REGISTER = 0x3B;
static constexpr uint8_t MPU6050_GYROSCOPE_DATA_REGISTER = 0x43;
static constexpr uint8_t MPU6050_POWER_MANAGEMENT_REGISTER = 0x6B;

static constexpr uint8_t MPU6050_RESET_COMMAND = 0x80;
static constexpr uint8_t MPU6050_CLOCK_SOURCE_PLL_X_GYRO = 0x01;
static constexpr uint8_t MPU6050_GYROSCOPE_RANGE_500_DPS = 0x08;
static constexpr uint8_t MPU6050_ACCELEROMETER_RANGE_2_G = 0x00;
static constexpr uint8_t MPU6050_DLPF_CONFIG = 0x00;
static constexpr uint8_t MPU6050_SAMPLE_RATE_DIVIDER = 0x00;

static constexpr float PI = 3.14159265358979323846f;
static constexpr float DEGREE_TO_RADIAN = PI / 180.0f;
static constexpr float ACCELEROMETER_SCALE_LSB_PER_G = 16384.0f;
static constexpr float GYROSCOPE_SCALE_LSB_PER_DPS = 65.5f;
static constexpr float TEMPERATURE_SCALE_LSB_PER_C = 340.0f;
static constexpr float TEMPERATURE_OFFSET_C = 36.53f;
static constexpr uint32_t MAX_UPDATE_INTERVAL_US = 100000U;

static constexpr uint32_t RESET_DELAY_MS = 100U;
static constexpr uint32_t WAKE_DELAY_MS = 10U;
static constexpr uint16_t CALIBRATION_WARMUP_SAMPLE_COUNT = 100U;
static constexpr uint16_t CALIBRATION_SAMPLE_COUNT = 1000U;

/**
 * @brief 从 MPU6050 大端寄存器数据中读取一个有符号 16 位值
 *
 * @param data 两字节寄存器数据
 *
 * @return 解码后的有符号原始值
 */
static int16_t decode_int16_be(const uint8_t *data)
{
    uint16_t raw_value = (uint16_t)((uint16_t)data[0] << 8) | data[1];
    return (int16_t)raw_value;
}

/**
 * @brief 把角度限制到负 PI 至正 PI
 *
 * @param angle_rad 待限制角度，单位弧度
 *
 * @return 限制后的角度，单位弧度
 */
static float wrap_angle_rad(float angle_rad)
{
    if(angle_rad > PI)
    {
        angle_rad -= 2.0f * PI;
    }
    else if(angle_rad < -PI)
    {
        angle_rad += 2.0f * PI;
    }

    return angle_rad;
}

/**
 * @brief 创建 MPU6050 驱动对象
 *
 * @param i2c_bus_id I2C 总线编号
 * @param device_address MPU6050 的 7 位 I2C 地址
 * @param accelerometer_weight 互补滤波中加速度角度的权重
 */
mpu6050::mpu6050(uint8_t i2c_bus_id,
    uint8_t device_address,
    float accelerometer_weight)
    : i2c(i2c_bus_id),
      device_address(device_address),
      accelerometer_weight(accelerometer_weight)
{
}

/**
 * @brief 初始化 MPU6050 并按需校准陀螺仪零偏
 *
 * @param calibrate_gyroscope 是否执行静止零偏校准
 *
 * @return I2C 操作结果
 */
i2c_result mpu6050::init(bool calibrate_gyroscope)
{
    initialized = false;
    first_sample = true;
    previous_timestamp_us = 0U;
    current_sample = {};

    i2c_result result = i2c.init();
    if(result != i2c_result::OK)
    {
        return result;
    }

    result = write_register(MPU6050_POWER_MANAGEMENT_REGISTER,
        MPU6050_RESET_COMMAND);
    if(result != i2c_result::OK)
    {
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(RESET_DELAY_MS));

    result = write_register(MPU6050_POWER_MANAGEMENT_REGISTER,
        MPU6050_CLOCK_SOURCE_PLL_X_GYRO);
    if(result != i2c_result::OK)
    {
        return result;
    }

    vTaskDelay(pdMS_TO_TICKS(WAKE_DELAY_MS));

    uint8_t device_id = 0U;
    result = read_registers(MPU6050_WHO_AM_I_REGISTER,
        &device_id,
        1U);
    if(result != i2c_result::OK)
    {
        return result;
    }

    if(device_id != MPU6050_EXPECTED_ID)
    {
        return i2c_result::BUS_ERROR;
    }

    result = write_register(MPU6050_SAMPLE_RATE_DIVIDER_REGISTER,
        MPU6050_SAMPLE_RATE_DIVIDER);
    if(result != i2c_result::OK)
    {
        return result;
    }

    result = write_register(MPU6050_CONFIG_REGISTER,
        MPU6050_DLPF_CONFIG);
    if(result != i2c_result::OK)
    {
        return result;
    }

    result = write_register(MPU6050_GYROSCOPE_CONFIG_REGISTER,
        MPU6050_GYROSCOPE_RANGE_500_DPS);
    if(result != i2c_result::OK)
    {
        return result;
    }

    result = write_register(MPU6050_ACCELEROMETER_CONFIG_REGISTER,
        MPU6050_ACCELEROMETER_RANGE_2_G);
    if(result != i2c_result::OK)
    {
        return result;
    }

    for(uint8_t axis = 0U; axis < 3U; axis++)
    {
        gyroscope_offset_rad_s[axis] = 0.0f;
    }

    if(calibrate_gyroscope)
    {
        result = calibrate_gyroscope_offset();
        if(result != i2c_result::OK)
        {
            return result;
        }
    }

    initialized = true;
    return i2c_result::OK;
}

/**
 * @brief 读取并融合一帧 MPU6050 数据
 *
 * @return I2C 操作结果
 */
i2c_result mpu6050::update()
{
    if(!initialized)
    {
        return i2c_result::NOT_INITIALIZED;
    }

    i2c_result result = read_registers(MPU6050_ACCELEROMETER_DATA_REGISTER,
        raw_sample,
        sizeof(raw_sample));
    if(result != i2c_result::OK)
    {
        return result;
    }

    uint32_t timestamp_us = sys_time::get_us_tick();
    process_raw_sample(timestamp_us);
    current_sample.timestamp_us = timestamp_us;
    current_sample.sequence++;
    return i2c_result::OK;
}

/**
 * @brief 获取最近一次完整的 MPU6050 采样
 *
 * @return 最近一次采样的只读引用
 */
const mpu6050_sample &mpu6050::sample() const
{
    return current_sample;
}

/**
 * @brief 读取 MPU6050 连续寄存器
 *
 * @param register_address 起始寄存器地址
 * @param data 接收缓冲区
 * @param size 接收长度
 *
 * @return I2C 操作结果
 */
i2c_result mpu6050::read_registers(uint8_t register_address,
    uint8_t *data,
    uint16_t size)
{
    return i2c.read_bytes(device_address,
        register_address,
        data,
        size);
}

/**
 * @brief 写入一个 MPU6050 配置寄存器
 *
 * @param register_address 寄存器地址
 * @param value 寄存器值
 *
 * @return I2C 操作结果
 */
i2c_result mpu6050::write_register(uint8_t register_address, uint8_t value)
{
    return i2c.write_bytes(device_address,
        register_address,
        &value,
        1U);
}

/**
 * @brief 在设备静止时计算三轴陀螺仪零偏
 *
 * @return I2C 操作结果
 */
i2c_result mpu6050::calibrate_gyroscope_offset()
{
    uint8_t gyroscope_raw[6]{};

    for(uint16_t sample_index = 0U;
        sample_index < CALIBRATION_WARMUP_SAMPLE_COUNT;
        sample_index++)
    {
        i2c_result result = read_registers(MPU6050_GYROSCOPE_DATA_REGISTER,
            gyroscope_raw,
            sizeof(gyroscope_raw));
        if(result != i2c_result::OK)
        {
            return result;
        }
    }

    int64_t gyroscope_sum[3]{};

    for(uint16_t sample_index = 0U;
        sample_index < CALIBRATION_SAMPLE_COUNT;
        sample_index++)
    {
        i2c_result result = read_registers(MPU6050_GYROSCOPE_DATA_REGISTER,
            gyroscope_raw,
            sizeof(gyroscope_raw));
        if(result != i2c_result::OK)
        {
            return result;
        }

        for(uint8_t axis = 0U; axis < 3U; axis++)
        {
            gyroscope_sum[axis] += decode_int16_be(
                &gyroscope_raw[axis * 2U]);
        }
    }

    for(uint8_t axis = 0U; axis < 3U; axis++)
    {
        float average_raw = (float)gyroscope_sum[axis] /
            (float)CALIBRATION_SAMPLE_COUNT;
        gyroscope_offset_rad_s[axis] = average_raw /
            GYROSCOPE_SCALE_LSB_PER_DPS * DEGREE_TO_RADIAN;
    }

    return i2c_result::OK;
}

/**
 * @brief 解码原始数据并更新三轴互补滤波角度
 *
 * @param timestamp_us 当前微秒时间戳
 */
void mpu6050::process_raw_sample(uint32_t timestamp_us)
{
    int16_t raw_temperature = decode_int16_be(&raw_sample[6]);
    current_sample.temperature_c = (float)raw_temperature /
        TEMPERATURE_SCALE_LSB_PER_C + TEMPERATURE_OFFSET_C;

    float native_acceleration_g[3]{};
    float native_angular_velocity_rad_s[3]{};

    for(uint8_t axis = 0U; axis < 3U; axis++)
    {
        int16_t raw_acceleration = decode_int16_be(
            &raw_sample[axis * 2U]);
        int16_t raw_gyroscope = decode_int16_be(
            &raw_sample[8U + axis * 2U]);

        native_acceleration_g[axis] = (float)raw_acceleration /
            ACCELEROMETER_SCALE_LSB_PER_G;
        native_angular_velocity_rad_s[axis] =
            (float)raw_gyroscope /
            GYROSCOPE_SCALE_LSB_PER_DPS * DEGREE_TO_RADIAN -
            gyroscope_offset_rad_s[axis];
    }

    // 轴向变换
    current_sample.acceleration_g[0] = -native_acceleration_g[1];
    current_sample.acceleration_g[1] = native_acceleration_g[0];
    current_sample.acceleration_g[2] = native_acceleration_g[2];
    current_sample.angular_velocity_rad_s[0] =
        -native_angular_velocity_rad_s[1];
    current_sample.angular_velocity_rad_s[1] =
        native_angular_velocity_rad_s[0];
    current_sample.angular_velocity_rad_s[2] =
        native_angular_velocity_rad_s[2];

    float accelerometer_roll_rad = atan2f(
        current_sample.acceleration_g[1],
        current_sample.acceleration_g[2]);
    float accelerometer_pitch_rad = atan2f(
        -current_sample.acceleration_g[0],
        sqrtf(current_sample.acceleration_g[1] *
            current_sample.acceleration_g[1] +
            current_sample.acceleration_g[2] *
            current_sample.acceleration_g[2]));

    if(first_sample)
    {
        current_sample.angle_rad[0] = accelerometer_roll_rad;
        current_sample.angle_rad[1] = accelerometer_pitch_rad;
        current_sample.angle_rad[2] = 0.0f;
        first_sample = false;
        previous_timestamp_us = timestamp_us;
        return;
    }

    uint32_t elapsed_us = timestamp_us - previous_timestamp_us;
    previous_timestamp_us = timestamp_us;
    if(elapsed_us == 0U || elapsed_us > MAX_UPDATE_INTERVAL_US)
    {
        return;
    }

    float elapsed_seconds = (float)elapsed_us * 1.0e-6f;
    float gyroscope_weight = 1.0f - accelerometer_weight;
    current_sample.angle_rad[0] = gyroscope_weight *
        (current_sample.angle_rad[0] +
            current_sample.angular_velocity_rad_s[0] * elapsed_seconds) +
        accelerometer_weight * accelerometer_roll_rad;
    current_sample.angle_rad[1] = gyroscope_weight *
        (current_sample.angle_rad[1] +
            current_sample.angular_velocity_rad_s[1] * elapsed_seconds) +
        accelerometer_weight * accelerometer_pitch_rad;
    current_sample.angle_rad[2] = wrap_angle_rad(
        current_sample.angle_rad[2] +
        current_sample.angular_velocity_rad_s[2] * elapsed_seconds);
}
