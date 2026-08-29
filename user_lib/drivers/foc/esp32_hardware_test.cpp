#include "esp32_hardware_test.h"

#include "foc_core.h"
#include "drivers/bus/i2c_bus.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>

/* ---- 硬件引脚和测试参数 ---- */

static const char *TAG = "esp32_foc_test";

static constexpr gpio_num_t ENABLE_PIN = GPIO_NUM_12;
static constexpr gpio_num_t CURRENT_A_PIN = GPIO_NUM_39;
static constexpr gpio_num_t CURRENT_B_PIN = GPIO_NUM_36;
static constexpr gpio_num_t BUS_SENSE_PIN = GPIO_NUM_4;
static constexpr gpio_num_t PWM_A_PIN = GPIO_NUM_32;
static constexpr gpio_num_t PWM_B_PIN = GPIO_NUM_33;
static constexpr gpio_num_t PWM_C_PIN = GPIO_NUM_25;
static constexpr gpio_num_t ENCODER_SDA_PIN = GPIO_NUM_19;
static constexpr gpio_num_t ENCODER_SCL_PIN = GPIO_NUM_18;

static constexpr uint8_t AS5600_ADDRESS = 0x36;
static constexpr uint8_t AS5600_RAW_ANGLE_REGISTER = 0x0C;
static constexpr uint32_t I2C_SPEED_HZ = 400000;

static constexpr ledc_mode_t PWM_SPEED_MODE = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t PWM_TIMER = LEDC_TIMER_0;
static constexpr ledc_channel_t PWM_A_CHANNEL = LEDC_CHANNEL_0;
static constexpr ledc_channel_t PWM_B_CHANNEL = LEDC_CHANNEL_1;
static constexpr ledc_channel_t PWM_C_CHANNEL = LEDC_CHANNEL_2;
static constexpr uint32_t PWM_FREQUENCY_HZ = 30000;
static constexpr uint32_t PWM_RESOLUTION_BITS = 8;
static constexpr ledc_timer_bit_t PWM_RESOLUTION = LEDC_TIMER_8_BIT;
static constexpr uint32_t PWM_DUTY_MAX =
    (1 << PWM_RESOLUTION_BITS) - 1;

static constexpr adc_channel_t CURRENT_A_CHANNEL = ADC_CHANNEL_3;
static constexpr adc_channel_t CURRENT_B_CHANNEL = ADC_CHANNEL_0;
static constexpr adc_channel_t BUS_SENSE_CHANNEL = ADC_CHANNEL_0;
static constexpr adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_12;
static constexpr adc_bitwidth_t ADC_BITWIDTH = ADC_BITWIDTH_DEFAULT;
static constexpr float ADC_RAW_TO_MV = 3300.0f / 4095.0f;

static constexpr float PI = 3.14159265358979323846f;
static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float SQRT_THREE_OVER_TWO = 0.86602540378443864676f;

static constexpr uint8_t MOTOR_POLE_PAIRS = 7;
static constexpr int8_t MOTOR_ROTOR_DIRECTION = -1;
static constexpr float MOTOR_BUS_VOLTAGE_V = 12.0f;
static constexpr float MOTOR_VOLTAGE_LIMIT_V = 2.0f;
static constexpr float MOTOR_MAX_PHASE_CURRENT_A = 0.2f;
static constexpr float OPEN_LOOP_TARGET_Q_CURRENT_A = 0.08f;
static constexpr float MOTOR_CONTROL_PERIOD_S = 0.001f;
static constexpr float CURRENT_PI_KP = 0.54f;
static constexpr float CURRENT_PI_KI = 400.0f;
static constexpr float CURRENT_PI_INTEGRAL_LIMIT_V = 1.0f;

static constexpr float OPEN_LOOP_TARGET_MECHANICAL_SPEED_RAD_S = 1.0f;
static constexpr float OPEN_LOOP_ELECTRICAL_SPEED_RAD_S =
    static_cast<float>(MOTOR_ROTOR_DIRECTION) *
    static_cast<float>(MOTOR_POLE_PAIRS) *
    OPEN_LOOP_TARGET_MECHANICAL_SPEED_RAD_S;
static constexpr float OPEN_LOOP_ELECTRICAL_ACCEL_RAD_S2 = 10.0f;
static constexpr float MOTOR_MAX_MECHANICAL_SPEED_RAD_S = 30.0f;
static constexpr float SPEED_FILTER_ALPHA = 0.2f;

static constexpr float CURRENT_SHUNT_RESISTOR_OHM = 0.01f;
static constexpr float CURRENT_AMPLIFIER_GAIN = 50.0f;
static constexpr float CURRENT_SENSE_SIGN = -1.0f;
static constexpr float CURRENT_SCALE_A_PER_MV =
    1.0f / CURRENT_SHUNT_RESISTOR_OHM /
        CURRENT_AMPLIFIER_GAIN / 1000.0f;

static constexpr uint32_t CURRENT_OFFSET_SAMPLE_COUNT = 1000;
static constexpr uint32_t CURRENT_SAMPLE_COUNT = 5;
static constexpr uint32_t CURRENT_SAMPLE_SPACING_US = 6;
static constexpr float ALIGNMENT_VOLTAGE_V = 0.4f;
static constexpr float ALIGNMENT_MAX_PHASE_CURRENT_A = 0.35f;
static constexpr uint32_t ALIGNMENT_DURATION_MS = 500;
static constexpr uint32_t AUTO_START_DELAY_MS = 1000;
static constexpr uint32_t TEST_RUN_DURATION_MS = 60000;
static constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;
static constexpr uint32_t LOG_PERIOD_MS = 500;

/* ---- 硬件运行状态 ---- */

static foc_core motor;
static i2c_bus encoder_bus(I2C_NUM_0,
    ENCODER_SDA_PIN,
    ENCODER_SCL_PIN,
    false);
static i2c_device encoder_device(encoder_bus,
    AS5600_ADDRESS,
    I2C_SPEED_HZ);

static adc_oneshot_unit_handle_t adc1_unit = nullptr;
static adc_oneshot_unit_handle_t adc2_unit = nullptr;
static adc_cali_handle_t adc1_calibration = nullptr;
static adc_cali_handle_t adc2_calibration = nullptr;

static bool pwm_initialized = false;
static bool bus_sense_available = false;
static bool module_initialized = false;
static float current_offset_a_mv = 0.0f;
static float current_offset_b_mv = 0.0f;
static bool encoder_tracking_valid = false;
static float encoder_previous_angle_rad = 0.0f;
static float encoder_unwrapped_angle_rad = 0.0f;
static int64_t encoder_previous_timestamp_us = 0;
static uint32_t sample_sequence = 0;
static int32_t bus_sense_voltage_mv = 0;
static bool bus_sense_read_valid = false;
static float motor_electrical_zero_offset_rad = 0.0f;

/* ---- 通用硬件辅助函数 ---- */

/**
 * @brief 计算浮点数绝对值
 *
 * @param value 输入数值
 *
 * @return 绝对值
 */
static float absolute_value(float value)
{
    return value < 0.0f ? -value : value;
}

/**
 * @brief 将角度限制到零至二倍圆周率范围
 *
 * @param angle_rad 输入角度，单位弧度
 *
 * @return 归一化角度，单位弧度
 */
static float normalize_angle(float angle_rad)
{
    while(angle_rad >= TWO_PI)
    {
        angle_rad -= TWO_PI;
    }
    while(angle_rad < 0.0f)
    {
        angle_rad += TWO_PI;
    }
    return angle_rad;
}

/**
 * @brief 配置一个 LEDC PWM 通道
 *
 * @param pin PWM 输出 GPIO
 * @param channel LEDC 通道
 *
 * @return 配置成功时返回 true
 */
static bool configure_pwm_channel(gpio_num_t pin,
    ledc_channel_t channel)
{
    ledc_channel_config_t config = {};
    config.gpio_num = static_cast<int>(pin);
    config.speed_mode = PWM_SPEED_MODE;
    config.channel = channel;
    config.timer_sel = PWM_TIMER;
    config.duty = PWM_DUTY_MAX / 2;
    config.hpoint = 0;
    config.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
    config.flags.output_invert = 0;
    config.deconfigure = false;

    return ledc_channel_config(&config) == ESP_OK;
}

/**
 * @brief 初始化三相 LEDC PWM 和使能 GPIO
 *
 * @return 初始化成功时返回 true
 */
static bool initialize_power_output()
{
    if(pwm_initialized)
    {
        return true;
    }

    esp_err_t error = gpio_set_direction(ENABLE_PIN, GPIO_MODE_OUTPUT);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "配置使能 GPIO 失败: %d", static_cast<int>(error));
        return false;
    }

    error = gpio_set_level(ENABLE_PIN, 0);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "关闭驱动器失败: %d", static_cast<int>(error));
        return false;
    }

    ledc_timer_config_t timer_config = {};
    timer_config.speed_mode = PWM_SPEED_MODE;
    timer_config.duty_resolution = PWM_RESOLUTION;
    timer_config.timer_num = PWM_TIMER;
    timer_config.freq_hz = PWM_FREQUENCY_HZ;
    timer_config.clk_cfg = LEDC_AUTO_CLK;
    timer_config.deconfigure = false;

    error = ledc_timer_config(&timer_config);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "配置 PWM 定时器失败: %d", static_cast<int>(error));
        return false;
    }

    if(!configure_pwm_channel(PWM_A_PIN, PWM_A_CHANNEL) ||
        !configure_pwm_channel(PWM_B_PIN, PWM_B_CHANNEL) ||
        !configure_pwm_channel(PWM_C_PIN, PWM_C_CHANNEL))
    {
        ESP_LOGE(TAG, "配置三相 PWM 通道失败");
        return false;
    }

    pwm_initialized = true;
    return true;
}

/**
 * @brief 将占空比转换为 LEDC 计数值
 *
 * @param duty 占空比，范围零至一
 *
 * @return LEDC 占空比计数值
 */
static uint32_t duty_to_counts(float duty)
{
    return static_cast<uint32_t>(duty * PWM_DUTY_MAX + 0.5f);
}

/**
 * @brief 向三相 LEDC 通道写入占空比
 *
 * @param duty 三相占空比
 *
 * @return 写入结果
 */
static foc_result write_pwm_duty(const phase_duty &duty)
{
    if(!pwm_initialized)
    {
        return foc_result::DRIVER_FAULT;
    }
    if(!(duty.phase_a >= 0.0f && duty.phase_a <= 1.0f) ||
        !(duty.phase_b >= 0.0f && duty.phase_b <= 1.0f) ||
        !(duty.phase_c >= 0.0f && duty.phase_c <= 1.0f))
    {
        return foc_result::OUTPUT_RANGE;
    }

    esp_err_t error = ledc_set_duty(PWM_SPEED_MODE,
        PWM_A_CHANNEL,
        duty_to_counts(duty.phase_a));
    if(error != ESP_OK)
    {
        return foc_result::DRIVER_FAULT;
    }

    error = ledc_set_duty(PWM_SPEED_MODE,
        PWM_B_CHANNEL,
        duty_to_counts(duty.phase_b));
    if(error != ESP_OK)
    {
        return foc_result::DRIVER_FAULT;
    }

    error = ledc_set_duty(PWM_SPEED_MODE,
        PWM_C_CHANNEL,
        duty_to_counts(duty.phase_c));
    if(error != ESP_OK)
    {
        return foc_result::DRIVER_FAULT;
    }

    error = ledc_update_duty(PWM_SPEED_MODE, PWM_A_CHANNEL);
    if(error != ESP_OK)
    {
        return foc_result::DRIVER_FAULT;
    }

    error = ledc_update_duty(PWM_SPEED_MODE, PWM_B_CHANNEL);
    if(error != ESP_OK)
    {
        return foc_result::DRIVER_FAULT;
    }

    error = ledc_update_duty(PWM_SPEED_MODE, PWM_C_CHANNEL);
    return error == ESP_OK ? foc_result::OK : foc_result::DRIVER_FAULT;
}

/**
 * @brief 关闭功率级并恢复三相中性占空比
 */
static void disable_power_stage()
{
    phase_duty neutral_duty{};
    write_pwm_duty(neutral_duty);
    gpio_set_level(ENABLE_PIN, 0);
}

/**
 * @brief 打开功率级使能
 *
 * @return 打开成功时返回 true
 */
static bool enable_power_stage()
{
    if(!pwm_initialized)
    {
        return false;
    }

    return gpio_set_level(ENABLE_PIN, 1) == ESP_OK;
}

/**
 * @brief 计算固定 Alpha-Beta 电压对应的 SVPWM 占空比
 *
 * @param alpha_voltage_v Alpha 轴电压，单位伏特
 * @param beta_voltage_v Beta 轴电压，单位伏特
 * @param duty 用于接收三相占空比
 *
 * @return 计算成功且占空比未越界时返回 true
 */
static bool calculate_svpwm_duty(float alpha_voltage_v,
    float beta_voltage_v,
    phase_duty &duty)
{
    float phase_a_voltage_v = alpha_voltage_v;
    float phase_b_voltage_v = -0.5f * alpha_voltage_v +
        SQRT_THREE_OVER_TWO * beta_voltage_v;
    float phase_c_voltage_v = -0.5f * alpha_voltage_v -
        SQRT_THREE_OVER_TWO * beta_voltage_v;
    float maximum_voltage_v = phase_a_voltage_v;
    if(phase_b_voltage_v > maximum_voltage_v)
    {
        maximum_voltage_v = phase_b_voltage_v;
    }
    if(phase_c_voltage_v > maximum_voltage_v)
    {
        maximum_voltage_v = phase_c_voltage_v;
    }

    float minimum_voltage_v = phase_a_voltage_v;
    if(phase_b_voltage_v < minimum_voltage_v)
    {
        minimum_voltage_v = phase_b_voltage_v;
    }
    if(phase_c_voltage_v < minimum_voltage_v)
    {
        minimum_voltage_v = phase_c_voltage_v;
    }

    float common_mode_voltage_v = -0.5f *
        (maximum_voltage_v + minimum_voltage_v);
    float phase_a_duty = 0.5f +
        (phase_a_voltage_v + common_mode_voltage_v) /
            MOTOR_BUS_VOLTAGE_V;
    float phase_b_duty = 0.5f +
        (phase_b_voltage_v + common_mode_voltage_v) /
            MOTOR_BUS_VOLTAGE_V;
    float phase_c_duty = 0.5f +
        (phase_c_voltage_v + common_mode_voltage_v) /
            MOTOR_BUS_VOLTAGE_V;

    if(!(phase_a_duty >= 0.0f && phase_a_duty <= 1.0f) ||
        !(phase_b_duty >= 0.0f && phase_b_duty <= 1.0f) ||
        !(phase_c_duty >= 0.0f && phase_c_duty <= 1.0f))
    {
        duty = {};
        return false;
    }

    duty.phase_a = phase_a_duty;
    duty.phase_b = phase_b_duty;
    duty.phase_c = phase_c_duty;
    return true;
}

/* ---- ADC 初始化和读取 ---- */

/**
 * @brief 创建一个 ADC 线性标定句柄
 *
 * @param unit ADC 单元
 * @param calibration 用于接收标定句柄
 */
static void initialize_adc_calibration(adc_unit_t unit,
    adc_cali_handle_t &calibration)
{
    calibration = nullptr;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t config = {};
    config.unit_id = unit;
    config.atten = ADC_ATTENUATION;
    config.bitwidth = ADC_BITWIDTH;
    config.default_vref = 1100;

    esp_err_t error = adc_cali_create_scheme_line_fitting(&config,
        &calibration);
    if(error == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC%d 使用线性标定",
            static_cast<int>(unit) + 1);
    }
    else
    {
        ESP_LOGW(TAG, "ADC%d 无法使用线性标定，回退到近似换算: %d",
            static_cast<int>(unit) + 1,
            static_cast<int>(error));
    }
#else
    ESP_LOGW(TAG, "ADC%d 不支持线性标定，使用近似换算",
        static_cast<int>(unit) + 1);
#endif
}

/**
 * @brief 初始化 ADC1 电流通道和 ADC2 母线采样通道
 *
 * @return 电流 ADC 初始化成功时返回 true
 */
static bool initialize_adc()
{
    adc_oneshot_unit_init_cfg_t adc1_config = {};
    adc1_config.unit_id = ADC_UNIT_1;
    adc1_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    adc1_config.ulp_mode = ADC_ULP_MODE_DISABLE;

    esp_err_t error = adc_oneshot_new_unit(&adc1_config, &adc1_unit);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "初始化 ADC1 失败: %d", static_cast<int>(error));
        return false;
    }

    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten = ADC_ATTENUATION;
    channel_config.bitwidth = ADC_BITWIDTH;
    error = adc_oneshot_config_channel(adc1_unit,
        CURRENT_A_CHANNEL,
        &channel_config);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "配置 GPIO%d ADC 通道失败: %d",
            static_cast<int>(CURRENT_A_PIN),
            static_cast<int>(error));
        return false;
    }

    error = adc_oneshot_config_channel(adc1_unit,
        CURRENT_B_CHANNEL,
        &channel_config);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "配置 GPIO%d ADC 通道失败: %d",
            static_cast<int>(CURRENT_B_PIN),
            static_cast<int>(error));
        return false;
    }
    initialize_adc_calibration(ADC_UNIT_1, adc1_calibration);

    adc_oneshot_unit_init_cfg_t adc2_config = {};
    adc2_config.unit_id = ADC_UNIT_2;
    adc2_config.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
    adc2_config.ulp_mode = ADC_ULP_MODE_DISABLE;
    error = adc_oneshot_new_unit(&adc2_config, &adc2_unit);
    if(error != ESP_OK)
    {
        ESP_LOGW(TAG, "初始化 ADC2 失败，暂不记录 GPIO%d 母线采样: %d",
            static_cast<int>(BUS_SENSE_PIN),
            static_cast<int>(error));
        adc2_unit = nullptr;
        return true;
    }

    error = adc_oneshot_config_channel(adc2_unit,
        BUS_SENSE_CHANNEL,
        &channel_config);
    if(error != ESP_OK)
    {
        ESP_LOGW(TAG, "配置 GPIO%d ADC 通道失败，暂不记录母线采样: %d",
            static_cast<int>(BUS_SENSE_PIN),
            static_cast<int>(error));
        adc_oneshot_del_unit(adc2_unit);
        adc2_unit = nullptr;
        return true;
    }

    initialize_adc_calibration(ADC_UNIT_2, adc2_calibration);
    bus_sense_available = true;
    return true;
}

/**
 * @brief 读取一个 ADC 通道的毫伏值
 *
 * @param unit ADC 单元句柄
 * @param calibration ADC 标定句柄，可为空
 * @param channel ADC 通道
 * @param voltage_mv 用于接收电压，单位毫伏
 *
 * @return 读取成功时返回 true
 */
static bool read_adc_millivolts(adc_oneshot_unit_handle_t unit,
    adc_cali_handle_t calibration,
    adc_channel_t channel,
    int32_t &voltage_mv)
{
    if(!unit)
    {
        return false;
    }

    int raw_value = 0;
    esp_err_t error = adc_oneshot_read(unit, channel, &raw_value);
    if(error != ESP_OK)
    {
        return false;
    }

    if(calibration)
    {
        int calibrated_voltage_mv = 0;
        error = adc_cali_raw_to_voltage(calibration,
            raw_value,
            &calibrated_voltage_mv);
        if(error == ESP_OK)
        {
            voltage_mv = static_cast<int32_t>(calibrated_voltage_mv);
            return true;
        }
    }

    voltage_mv = static_cast<int32_t>(
        static_cast<float>(raw_value) * ADC_RAW_TO_MV);
    return true;
}

/**
 * @brief 读取两路电流采样的毫伏值并进行中值滤波
 *
 * @param voltage_a_mv A 路电压，单位毫伏
 * @param voltage_b_mv B 路电压，单位毫伏
 *
 * @return 读取成功时返回 true
 */
static bool read_current_millivolts(int32_t &voltage_a_mv,
    int32_t &voltage_b_mv)
{
    int32_t samples_a_mv[CURRENT_SAMPLE_COUNT] = {};
    int32_t samples_b_mv[CURRENT_SAMPLE_COUNT] = {};
    for(uint32_t index = 0; index < CURRENT_SAMPLE_COUNT; index++)
    {
        if(!read_adc_millivolts(adc1_unit,
            adc1_calibration,
            CURRENT_A_CHANNEL,
            samples_a_mv[index]) ||
            !read_adc_millivolts(adc1_unit,
                adc1_calibration,
                CURRENT_B_CHANNEL,
                samples_b_mv[index]))
        {
            return false;
        }
        if(index + 1 < CURRENT_SAMPLE_COUNT)
        {
            esp_rom_delay_us(CURRENT_SAMPLE_SPACING_US);
        }
    }

    for(uint32_t index = 1; index < CURRENT_SAMPLE_COUNT; index++)
    {
        int32_t value_a_mv = samples_a_mv[index];
        int32_t value_b_mv = samples_b_mv[index];
        uint32_t sorted_index = index;
        while(sorted_index > 0 &&
            samples_a_mv[sorted_index - 1] > value_a_mv)
        {
            samples_a_mv[sorted_index] =
                samples_a_mv[sorted_index - 1];
            sorted_index--;
        }
        samples_a_mv[sorted_index] = value_a_mv;

        sorted_index = index;
        while(sorted_index > 0 &&
            samples_b_mv[sorted_index - 1] > value_b_mv)
        {
            samples_b_mv[sorted_index] =
                samples_b_mv[sorted_index - 1];
            sorted_index--;
        }
        samples_b_mv[sorted_index] = value_b_mv;
    }

    voltage_a_mv = samples_a_mv[CURRENT_SAMPLE_COUNT / 2];
    voltage_b_mv = samples_b_mv[CURRENT_SAMPLE_COUNT / 2];
    return true;
}

/**
 * @brief 在功率级关闭时校准两路电流零点
 *
 * @return 校准成功时返回 true
 */
static bool calibrate_current_offsets()
{
    int64_t sum_a_mv = 0;
    int64_t sum_b_mv = 0;
    for(uint32_t index = 0;
        index < CURRENT_OFFSET_SAMPLE_COUNT;
        index++)
    {
        int32_t voltage_a_mv = 0;
        int32_t voltage_b_mv = 0;
        if(!read_current_millivolts(voltage_a_mv, voltage_b_mv))
        {
            ESP_LOGE(TAG, "电流零点校准读取失败，样本=%d",
                static_cast<int>(index));
            return false;
        }

        sum_a_mv += voltage_a_mv;
        sum_b_mv += voltage_b_mv;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    current_offset_a_mv = static_cast<float>(sum_a_mv) /
        CURRENT_OFFSET_SAMPLE_COUNT;
    current_offset_b_mv = static_cast<float>(sum_b_mv) /
        CURRENT_OFFSET_SAMPLE_COUNT;
    ESP_LOGI(TAG, "电流零点: O1=%.2f mV, O2=%.2f mV",
        current_offset_a_mv,
        current_offset_b_mv);
    return true;
}

/**
 * @brief 将电流采样毫伏值换算为两路相电流
 *
 * @param voltage_a_mv A 路电压，单位毫伏
 * @param voltage_b_mv B 路电压，单位毫伏
 * @param phase_a_a 用于接收 A 相电流，单位安培
 * @param phase_b_a 用于接收 B 相电流，单位安培
 * @param phase_c_a 用于接收重构的 C 相电流，单位安培
 */
static void convert_current(int32_t voltage_a_mv,
    int32_t voltage_b_mv,
    float &phase_a_a,
    float &phase_b_a,
    float &phase_c_a)
{
    phase_a_a = (static_cast<float>(voltage_a_mv) -
        current_offset_a_mv) * CURRENT_SCALE_A_PER_MV *
        CURRENT_SENSE_SIGN;
    phase_b_a = (static_cast<float>(voltage_b_mv) -
        current_offset_b_mv) * CURRENT_SCALE_A_PER_MV *
        CURRENT_SENSE_SIGN;
    phase_c_a = -phase_a_a - phase_b_a;
}

/**
 * @brief 读取当前两路采样对应的三相电流
 *
 * @param phase_a_a 用于接收 A 相电流，单位安培
 * @param phase_b_a 用于接收 B 相电流，单位安培
 * @param phase_c_a 用于接收 C 相电流，单位安培
 *
 * @return 读取成功时返回 true
 */
static bool read_phase_currents(float &phase_a_a,
    float &phase_b_a,
    float &phase_c_a)
{
    int32_t voltage_a_mv = 0;
    int32_t voltage_b_mv = 0;
    if(!read_current_millivolts(voltage_a_mv, voltage_b_mv))
    {
        return false;
    }

    convert_current(voltage_a_mv,
        voltage_b_mv,
        phase_a_a,
        phase_b_a,
        phase_c_a);
    return true;
}

/* ---- AS5600 读取和传感器对齐 ---- */

/**
 * @brief 读取 AS5600 原始机械角度
 *
 * @param angle_rad 用于接收机械角度，单位弧度
 *
 * @return 读取成功时返回 true
 */
static bool read_encoder_angle(float &angle_rad)
{
    uint8_t data[2] = {};
    i2c_result result = encoder_device.read_bytes(
        AS5600_RAW_ANGLE_REGISTER,
        data,
        2,
        5);
    if(result != i2c_result::OK)
    {
        return false;
    }

    uint16_t raw_angle = (static_cast<uint16_t>(data[0]) << 8) |
        static_cast<uint16_t>(data[1]);
    raw_angle &= 0x0FFF;
    angle_rad = static_cast<float>(raw_angle) * TWO_PI / 4096.0f;
    return true;
}

/**
 * @brief 清除编码器连续角度和速度历史
 */
static void reset_encoder_tracking()
{
    encoder_tracking_valid = false;
    encoder_previous_angle_rad = 0.0f;
    encoder_unwrapped_angle_rad = 0.0f;
    encoder_previous_timestamp_us = 0;
}

/**
 * @brief 根据固定 Alpha 轴电压矢量执行转子对齐
 *
 * @param electrical_zero_offset_rad 用于接收电角度零点偏移
 *
 * @return 对齐成功时返回 true
 */
static bool align_sensor(float &electrical_zero_offset_rad)
{
    phase_duty alignment_duty{};
    if(!calculate_svpwm_duty(ALIGNMENT_VOLTAGE_V,
        0.0f,
        alignment_duty))
    {
        ESP_LOGE(TAG, "计算传感器对齐占空比失败");
        return false;
    }

    if(!enable_power_stage())
    {
        ESP_LOGE(TAG, "传感器对齐时无法打开驱动器");
        disable_power_stage();
        return false;
    }

    if(write_pwm_duty(alignment_duty) != foc_result::OK)
    {
        ESP_LOGE(TAG, "输出传感器对齐电压失败");
        disable_power_stage();
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    int64_t alignment_end_us = esp_timer_get_time() +
        static_cast<int64_t>(ALIGNMENT_DURATION_MS) * 1000;
    while(esp_timer_get_time() < alignment_end_us)
    {
        float phase_a_a = 0.0f;
        float phase_b_a = 0.0f;
        float phase_c_a = 0.0f;
        if(!read_phase_currents(phase_a_a, phase_b_a, phase_c_a))
        {
            ESP_LOGE(TAG, "传感器对齐时无法读取电流");
            disable_power_stage();
            return false;
        }

        if(absolute_value(phase_a_a) > ALIGNMENT_MAX_PHASE_CURRENT_A ||
            absolute_value(phase_b_a) > ALIGNMENT_MAX_PHASE_CURRENT_A ||
            absolute_value(phase_c_a) > ALIGNMENT_MAX_PHASE_CURRENT_A)
        {
            ESP_LOGE(TAG, "传感器对齐过流: A=%.3f B=%.3f C=%.3f",
                phase_a_a,
                phase_b_a,
                phase_c_a);
            disable_power_stage();
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    float aligned_mechanical_angle_rad = 0.0f;
    if(!read_encoder_angle(aligned_mechanical_angle_rad))
    {
        disable_power_stage();
        ESP_LOGE(TAG, "传感器对齐后无法读取机械角度");
        return false;
    }

    electrical_zero_offset_rad = normalize_angle(
        static_cast<float>(MOTOR_ROTOR_DIRECTION) *
        static_cast<float>(MOTOR_POLE_PAIRS) *
        aligned_mechanical_angle_rad);
    disable_power_stage();
    vTaskDelay(pdMS_TO_TICKS(20));
    reset_encoder_tracking();
    ESP_LOGI(TAG, "传感器对齐完成: mechanical=%.5f, zero=%.5f",
        aligned_mechanical_angle_rad,
        electrical_zero_offset_rad);
    return true;
}

/**
 * @brief 读取并展开一个 AS5600 转子样本
 *
 * @param timestamp_us 本次采样时间戳，单位微秒
 * @param sequence 样本序号
 * @param sample 用于接收转子样本
 *
 * @return 读取成功时返回 true
 */
static bool read_rotor_sample(int64_t timestamp_us,
    uint32_t sequence,
    rotor_sample &sample)
{
    sample = {};
    sample.sequence = sequence;
    sample.timestamp_us = static_cast<uint32_t>(timestamp_us);

    float angle_rad = 0.0f;
    if(!read_encoder_angle(angle_rad))
    {
        return false;
    }

    float velocity_rad_s = 0.0f;
    if(!encoder_tracking_valid)
    {
        encoder_previous_angle_rad = angle_rad;
        encoder_unwrapped_angle_rad = angle_rad;
        encoder_previous_timestamp_us = timestamp_us;
        encoder_tracking_valid = true;
    }
    else
    {
        float delta_angle_rad = angle_rad - encoder_previous_angle_rad;
        if(delta_angle_rad > PI)
        {
            delta_angle_rad -= TWO_PI;
        }
        else if(delta_angle_rad < -PI)
        {
            delta_angle_rad += TWO_PI;
        }

        int64_t delta_time_us = timestamp_us -
            encoder_previous_timestamp_us;
        if(delta_time_us <= 0)
        {
            return false;
        }

        encoder_unwrapped_angle_rad += delta_angle_rad;
        velocity_rad_s = delta_angle_rad /
            (static_cast<float>(delta_time_us) * 1.0e-6f);
        encoder_previous_angle_rad = angle_rad;
        encoder_previous_timestamp_us = timestamp_us;
    }

    sample.mechanical_angle_rad = encoder_unwrapped_angle_rad;
    sample.mechanical_velocity_rad_s = velocity_rad_s;
    sample.valid = true;
    return true;
}

/* ---- foc_output 回调和控制任务 ---- */

/**
 * @brief 初始化 FOC 功率输出回调
 *
 * @param context 输出上下文
 *
 * @return 初始化结果
 */
static foc_result foc_output_init(void *context)
{
    if(!initialize_power_output())
    {
        return foc_result::DRIVER_FAULT;
    }

    phase_duty neutral_duty{};
    return write_pwm_duty(neutral_duty);
}

/**
 * @brief 打开 FOC 功率输出回调
 *
 * @param context 输出上下文
 *
 * @return 打开结果
 */
static foc_result foc_output_enable(void *context)
{
    return enable_power_stage() ? foc_result::OK :
        foc_result::DRIVER_FAULT;
}

/**
 * @brief 关闭 FOC 功率输出回调
 *
 * @param context 输出上下文
 */
static void foc_output_disable(void *context)
{
    disable_power_stage();
}

/**
 * @brief 应用任务上下文中的三相占空比
 *
 * @param context 输出上下文
 * @param duty 三相占空比
 *
 * @return 应用结果
 */
static foc_result foc_output_apply_duty(void *context,
    const phase_duty &duty)
{
    return write_pwm_duty(duty);
}

/**
 * @brief 拒绝当前未配置为 ISR 安全的 PWM 输出调用
 *
 * @param context 输出上下文
 * @param duty 三相占空比
 *
 * @return 输出结果
 *
 * @note 当前测试使用任务上下文 core_loop()。LEDC 控制函数尚未配置到
 *       IRAM，因此不能把此回调当作 ISR 输出路径使用。
 */
static foc_result foc_output_apply_duty_from_isr(void *context,
    const phase_duty &duty)
{
    return foc_result::DRIVER_FAULT;
}

/**
 * @brief 查询功率级硬件故障输入
 *
 * @param context 输出上下文
 *
 * @return 当前硬件无故障输入，固定返回 false
 */
static bool foc_output_fault_active(void *context)
{
    return false;
}

/**
 * @brief 创建当前测试使用的 FOC 输出回调表
 *
 * @return FOC 输出回调表
 */
static foc_output create_foc_output()
{
    foc_output output{};
    output.init = foc_output_init;
    output.enable = foc_output_enable;
    output.disable = foc_output_disable;
    output.apply_duty = foc_output_apply_duty;
    output.apply_duty_from_isr = foc_output_apply_duty_from_isr;
    output.fault_active = foc_output_fault_active;
    return output;
}

/**
 * @brief 读取驱动板分压后的 GPIO4 母线采样管脚电压
 *
 * @note GPIO4 接收的是驱动板已经分压后的电压。由于尚未确认分压电阻比，
 *       当前只记录管脚电压，FOC 母线配置仍使用已知的 12 V。GPIO4 属于
 *       经典 ESP32 的 ADC2；以后启用 Wi-Fi 后该采样可能不可用。
 */
static void read_bus_sense()
{
    if(!bus_sense_available)
    {
        bus_sense_read_valid = false;
        return;
    }

    bus_sense_read_valid = read_adc_millivolts(adc2_unit,
        adc2_calibration,
        BUS_SENSE_CHANNEL,
        bus_sense_voltage_mv);
}

/**
 * @brief FOC 硬件测试控制任务
 *
 * @param argument FreeRTOS 任务参数
 *
 * @note 任务每 1 ms 读取 ADC 和 AS5600，向 Topic 发布样本，生成低速强制角
 *       样本，再运行一次任务上下文的 FOC 电流环。真实 AS5600 角度只用于
 *       速度观察和超速保护；当前没有使用 ISR 控制路径。
 */
static void foc_control_task_entry(void *argument)
{
    foc_topic_access topics = motor.topics();
    TickType_t last_wake_tick = xTaskGetTickCount();
    int64_t task_start_us = esp_timer_get_time();
    bool motor_started = false;
    bool test_finished = false;
    bool start_error_reported = false;
    bool speed_filter_valid = false;
    float filtered_mechanical_speed_rad_s = 0.0f;
    bool open_loop_angle_valid = false;
    float open_loop_electrical_angle_rad = 0.0f;
    float open_loop_electrical_speed_rad_s = 0.0f;
    int64_t last_log_us = task_start_us;

    while(true)
    {
        uint32_t sequence = ++sample_sequence;
        int64_t current_timestamp_us = esp_timer_get_time();
        phase_current_sample current{};
        bool current_valid = read_phase_currents(
            current.phase_a_a,
            current.phase_b_a,
            current.phase_c_a);
        current.sequence = sequence;
        current.timestamp_us = static_cast<uint32_t>(current_timestamp_us);
        current.valid = current_valid;

        int64_t rotor_timestamp_us = esp_timer_get_time();
        rotor_sample rotor{};
        bool rotor_valid = read_rotor_sample(rotor_timestamp_us,
            sequence,
            rotor);

        if(rotor_valid)
        {
            if(!speed_filter_valid)
            {
                filtered_mechanical_speed_rad_s =
                    rotor.mechanical_velocity_rad_s;
                speed_filter_valid = true;
            }
            else
            {
                filtered_mechanical_speed_rad_s +=
                    SPEED_FILTER_ALPHA *
                    (rotor.mechanical_velocity_rad_s -
                        filtered_mechanical_speed_rad_s);
            }
        }

        read_bus_sense();
        topics.current.publish(current);
        topics.rotor.publish(rotor);

        int64_t now_us = esp_timer_get_time();
        if(!motor_started && !test_finished &&
            now_us - task_start_us >=
            static_cast<int64_t>(AUTO_START_DELAY_MS) * 1000)
        {
            if(current_valid && rotor_valid)
            {
                foc_result result = motor.enable();
                if(result == foc_result::OK)
                {
                    motor_started = true;
                    ESP_LOGI(TAG,
                        "进入低速强制角: target=%.2f rad/s, q=%.3f A",
                        OPEN_LOOP_TARGET_MECHANICAL_SPEED_RAD_S,
                        OPEN_LOOP_TARGET_Q_CURRENT_A);
                }
                else if(!start_error_reported)
                {
                    ESP_LOGE(TAG, "开启 FOC 失败: %d",
                        static_cast<int>(result));
                    start_error_reported = true;
                }
            }
        }

        foc_result loop_result = foc_result::NOT_READY;
        if(motor_started)
        {
            if(speed_filter_valid &&
                absolute_value(filtered_mechanical_speed_rad_s) >
                    MOTOR_MAX_MECHANICAL_SPEED_RAD_S)
            {
                motor.disable();
                motor_started = false;
                test_finished = true;
                ESP_LOGE(TAG,
                    "机械速度超限，已停机: speed=%.3f rad/s",
                    filtered_mechanical_speed_rad_s);
            }
            else
            {
                if(!open_loop_angle_valid)
                {
                    open_loop_electrical_angle_rad = normalize_angle(
                        static_cast<float>(MOTOR_ROTOR_DIRECTION) *
                        static_cast<float>(MOTOR_POLE_PAIRS) *
                        rotor.mechanical_angle_rad -
                        motor_electrical_zero_offset_rad);
                    open_loop_angle_valid = true;
                }

                float electrical_speed_step_rad_s =
                    OPEN_LOOP_ELECTRICAL_ACCEL_RAD_S2 *
                    MOTOR_CONTROL_PERIOD_S;
                if(open_loop_electrical_speed_rad_s <
                    OPEN_LOOP_ELECTRICAL_SPEED_RAD_S)
                {
                    open_loop_electrical_speed_rad_s +=
                        electrical_speed_step_rad_s;
                    if(open_loop_electrical_speed_rad_s >
                        OPEN_LOOP_ELECTRICAL_SPEED_RAD_S)
                    {
                        open_loop_electrical_speed_rad_s =
                            OPEN_LOOP_ELECTRICAL_SPEED_RAD_S;
                    }
                }
                else if(open_loop_electrical_speed_rad_s >
                    OPEN_LOOP_ELECTRICAL_SPEED_RAD_S)
                {
                    open_loop_electrical_speed_rad_s -=
                        electrical_speed_step_rad_s;
                    if(open_loop_electrical_speed_rad_s <
                        OPEN_LOOP_ELECTRICAL_SPEED_RAD_S)
                    {
                        open_loop_electrical_speed_rad_s =
                            OPEN_LOOP_ELECTRICAL_SPEED_RAD_S;
                    }
                }
                open_loop_electrical_angle_rad +=
                    open_loop_electrical_speed_rad_s *
                    MOTOR_CONTROL_PERIOD_S;

                foc_target open_loop_target{};
                open_loop_target.timestamp_us =
                    static_cast<uint32_t>(now_us);
                open_loop_target.mode = foc_control_mode::CURRENT;
                open_loop_target.d_axis_current_a = 0.0f;
                open_loop_target.q_axis_current_a =
                    OPEN_LOOP_TARGET_Q_CURRENT_A;
                foc_result target_result = motor.set_target(open_loop_target);
                if(target_result != foc_result::OK)
                {
                    motor.disable();
                    motor_started = false;
                    test_finished = true;
                    ESP_LOGE(TAG, "更新强制角测试目标失败，已停机: %d",
                        static_cast<int>(target_result));
                }
                else
                {
                    float direction_and_pole_pairs =
                        static_cast<float>(MOTOR_ROTOR_DIRECTION) *
                        static_cast<float>(MOTOR_POLE_PAIRS);
                    rotor_sample control_rotor = rotor;
                    control_rotor.mechanical_angle_rad =
                        (open_loop_electrical_angle_rad +
                            motor_electrical_zero_offset_rad) /
                        direction_and_pole_pairs;
                    control_rotor.mechanical_velocity_rad_s =
                        open_loop_electrical_speed_rad_s /
                        direction_and_pole_pairs;
                    if(!topics.rotor.publish(control_rotor))
                    {
                        motor.disable();
                        motor_started = false;
                        test_finished = true;
                        ESP_LOGE(TAG, "发布强制角样本失败，已停机");
                    }
                    else
                    {
                        uint32_t control_timestamp_us =
                            static_cast<uint32_t>(now_us);
                        loop_result = motor.core_loop(control_timestamp_us);
                        if(loop_result != foc_result::OK &&
                            loop_result != foc_result::DISABLED)
                        {
                            ESP_LOGE(TAG, "FOC 控制周期失败，已停机: %d",
                                static_cast<int>(loop_result));
                            motor_started = false;
                        }
                    }
                }
            }
        }

        if(motor_started && now_us - task_start_us >=
            static_cast<int64_t>(TEST_RUN_DURATION_MS) * 1000)
        {
            motor.disable();
            motor_started = false;
            test_finished = true;
            ESP_LOGI(TAG, "硬件测试时长到达，已自动停机");
        }

        if(now_us - last_log_us >=
            static_cast<int64_t>(LOG_PERIOD_MS) * 1000)
        {
            foc_snapshot snapshot{};
            bool snapshot_valid = topics.snapshot.peek(snapshot, 0);
            int32_t logged_bus_sense_mv = bus_sense_read_valid ?
                bus_sense_voltage_mv : -1;
            if(snapshot_valid)
            {
                ESP_LOGI(TAG,
                    "state=%d loop=%d Ia=%.3f Ib=%.3f Ic=%.3f "
                    "Id=%.3f Iq=%.3f Iq_target=%.3f speed=%.3f "
                    "field_speed=%.3f encoder_angle=%.3f field_angle=%.3f "
                    "control_angle=%.3f bus_pin=%d mV",
                    static_cast<int>(snapshot.state),
                    static_cast<int>(loop_result),
                    snapshot.phase_a_current_a,
                    snapshot.phase_b_current_a,
                    snapshot.phase_c_current_a,
                    snapshot.i_d_a,
                    snapshot.i_q_a,
                    snapshot.target_i_q_a,
                    filtered_mechanical_speed_rad_s,
                    open_loop_electrical_speed_rad_s,
                    rotor.mechanical_angle_rad,
                    open_loop_electrical_angle_rad,
                    snapshot.mechanical_angle_rad,
                    static_cast<int>(logged_bus_sense_mv));
            }
            else
            {
                ESP_LOGI(TAG,
                    "等待 FOC 快照: current=%d rotor=%d bus_pin=%d mV",
                    current_valid ? 1 : 0,
                    rotor_valid ? 1 : 0,
                    static_cast<int>(logged_bus_sense_mv));
            }
            last_log_us = now_us;
        }

        vTaskDelayUntil(&last_wake_tick,
            pdMS_TO_TICKS(CONTROL_TASK_PERIOD_MS));
    }
}

/* ---- esp32_hardware_test 公共 API ---- */

/**
 * @brief 初始化 ESP32 V3P 单电机 FOC 硬件测试
 *
 * @note 该测试会先保持 EN 低电平，校准电流零点，使用 0.4 V 固定电压矢量
 *       对齐 AS5600，然后延时 1 s 自动进入低速强制角测试。强制角目标机械
 *       速度为 1 rad/s，q 轴电流目标为 0.08 A，运行 60 s 后自动停机。没有
 *       硬件故障输入，保护依靠电流采样、机械速度检查和 foc_core 软件限幅。
 */
void esp32_hardware_test::init()
{
    if(module_initialized)
    {
        return;
    }

    if(!initialize_power_output() || !initialize_adc())
    {
        disable_power_stage();
        ESP_LOGE(TAG, "硬件测试初始化失败，保持驱动器关闭");
        return;
    }

    i2c_result encoder_result = encoder_device.init();
    if(encoder_result != i2c_result::OK)
    {
        disable_power_stage();
        ESP_LOGE(TAG, "AS5600 I2C 初始化失败: %d",
            static_cast<int>(encoder_result));
        return;
    }

    if(!calibrate_current_offsets())
    {
        disable_power_stage();
        ESP_LOGE(TAG, "电流零点校准失败，保持驱动器关闭");
        return;
    }

    float initial_encoder_angle_rad = 0.0f;
    if(!read_encoder_angle(initial_encoder_angle_rad))
    {
        disable_power_stage();
        ESP_LOGE(TAG, "AS5600 初始角度读取失败，保持驱动器关闭");
        return;
    }
    ESP_LOGI(TAG, "AS5600 初始机械角度: %.5f rad",
        initial_encoder_angle_rad);

    float electrical_zero_offset_rad = 0.0f;
    if(!align_sensor(electrical_zero_offset_rad))
    {
        disable_power_stage();
        ESP_LOGE(TAG, "传感器对齐失败，保持驱动器关闭");
        return;
    }

    foc_config config{};
    motor_electrical_zero_offset_rad = electrical_zero_offset_rad;
    config.pole_pairs = MOTOR_POLE_PAIRS;
    config.rotor_direction = MOTOR_ROTOR_DIRECTION;
    config.electrical_zero_offset_rad = electrical_zero_offset_rad;
    config.control_period_s = MOTOR_CONTROL_PERIOD_S;
    config.bus_voltage_v = MOTOR_BUS_VOLTAGE_V;
    config.voltage_limit_v = MOTOR_VOLTAGE_LIMIT_V;
    config.max_phase_current_a = MOTOR_MAX_PHASE_CURRENT_A;
    config.d_axis_pi.kp = CURRENT_PI_KP;
    config.d_axis_pi.ki = CURRENT_PI_KI;
    config.d_axis_pi.integral_limit = CURRENT_PI_INTEGRAL_LIMIT_V;
    config.q_axis_pi.kp = CURRENT_PI_KP;
    config.q_axis_pi.ki = CURRENT_PI_KI;
    config.q_axis_pi.integral_limit = CURRENT_PI_INTEGRAL_LIMIT_V;

    foc_result result = motor.init(config, create_foc_output());
    if(result != foc_result::OK)
    {
        disable_power_stage();
        ESP_LOGE(TAG, "FOC 核心初始化失败: %d", static_cast<int>(result));
        return;
    }

    foc_topic_access topics = motor.topics();
    uint32_t initial_sequence = ++sample_sequence;
    int64_t current_timestamp_us = esp_timer_get_time();
    phase_current_sample current{};
    current.sequence = initial_sequence;
    current.timestamp_us = static_cast<uint32_t>(current_timestamp_us);
    current.valid = read_phase_currents(current.phase_a_a,
        current.phase_b_a,
        current.phase_c_a);
    int64_t rotor_timestamp_us = esp_timer_get_time();
    rotor_sample rotor{};
    bool rotor_valid = read_rotor_sample(rotor_timestamp_us,
        initial_sequence,
        rotor);
    if(!current.valid || !rotor_valid ||
        !topics.current.publish(current) ||
        !topics.rotor.publish(rotor))
    {
        motor.disable();
        ESP_LOGE(TAG, "初始传感器样本无效，保持驱动器关闭");
        return;
    }

    foc_target target{};
    target.timestamp_us = static_cast<uint32_t>(esp_timer_get_time());
    target.mode = foc_control_mode::CURRENT;
    target.d_axis_current_a = 0.0f;
    target.q_axis_current_a = OPEN_LOOP_TARGET_Q_CURRENT_A;
    result = motor.set_target(target);
    if(result != foc_result::OK)
    {
        motor.disable();
        ESP_LOGE(TAG, "设置初始电流目标失败: %d", static_cast<int>(result));
        return;
    }

    ESP_LOGI(TAG,
        "硬件测试就绪: PWM=%d Hz, bus=%.1f V, shunt=%.3f ohm, gain=%.1f, "
        "current_scale=%.4f A/mV, force_speed=%.2f rad/s, "
        "q_target=%.3f A, auto_start=%d",
        static_cast<int>(PWM_FREQUENCY_HZ),
        MOTOR_BUS_VOLTAGE_V,
        CURRENT_SHUNT_RESISTOR_OHM,
        CURRENT_AMPLIFIER_GAIN,
        CURRENT_SCALE_A_PER_MV,
        OPEN_LOOP_TARGET_MECHANICAL_SPEED_RAD_S,
        OPEN_LOOP_TARGET_Q_CURRENT_A,
        1);

    BaseType_t task_result = xTaskCreate(foc_control_task_entry,
        "foc_control",
        4096,
        nullptr,
        tskIDLE_PRIORITY + 3,
        nullptr);
    if(task_result != pdPASS)
    {
        motor.disable();
        ESP_LOGE(TAG, "创建 FOC 控制任务失败");
        return;
    }

    module_initialized = true;
}
