#include "esp32_hardware_test.h"

#include "foc_core.h"
#include "drivers/bus/i2c_bus.h"
#include "driver/gpio.h"
#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "esp_ipc.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_struct.h"
#include "soc/soc_caps.h"
#include <atomic>
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
static constexpr bool I2C_ENABLE_INTERNAL_PULLUP = true;
static constexpr uint32_t ENCODER_READ_RETRY_COUNT = 2;
static constexpr uint32_t ENCODER_RETRY_DELAY_MS = 1;

static constexpr int PWM_GROUP_ID = 0;
static constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
static constexpr uint32_t PWM_RESOLUTION_HZ = 10000000;
static constexpr uint32_t PWM_PERIOD_TICKS =
    PWM_RESOLUTION_HZ / PWM_FREQUENCY_HZ;
static constexpr gpio_num_t PWM_PINS[] = {
    PWM_A_PIN,
    PWM_B_PIN,
    PWM_C_PIN
};

static constexpr adc_channel_t CURRENT_A_CHANNEL = ADC_CHANNEL_3;
static constexpr adc_channel_t CURRENT_B_CHANNEL = ADC_CHANNEL_0;
static constexpr adc_channel_t BUS_SENSE_CHANNEL = ADC_CHANNEL_0;
static constexpr adc_atten_t ADC_ATTENUATION = ADC_ATTEN_DB_12;
static constexpr adc_bitwidth_t ADC_BITWIDTH = ADC_BITWIDTH_DEFAULT;
static constexpr float ADC_RAW_TO_MV = 3300.0f / 4095.0f;
static constexpr uint32_t CURRENT_ADC_SAMPLE_FREQ_HZ = 40000;
static constexpr uint32_t CURRENT_ADC_FRAME_SIZE = 64;
static constexpr BaseType_t CURRENT_ADC_TASK_CORE = 0;
static constexpr UBaseType_t CURRENT_ADC_TASK_PRIORITY =
    tskIDLE_PRIORITY + 3;
static constexpr UBaseType_t CONTROL_TASK_PRIORITY =
    tskIDLE_PRIORITY + 4;

static constexpr float PI = 3.14159265358979323846f;
static constexpr float TWO_PI = 6.28318530717958647692f;
static constexpr float RAD_PER_SECOND_TO_RPM = 60.0f / TWO_PI;
static constexpr float SQRT_THREE_OVER_TWO = 0.86602540378443864676f;

static constexpr uint8_t MOTOR_POLE_PAIRS = 7;
static constexpr int8_t MOTOR_ROTOR_DIRECTION = 1;
static constexpr float MOTOR_BUS_VOLTAGE_V = 12.0f;
static constexpr float MOTOR_VOLTAGE_LIMIT_V = 4.0f;
static constexpr float MOTOR_MAX_PHASE_CURRENT_A = 0.2f;
static constexpr uint32_t FOC_CONTROL_FREQUENCY_HZ = PWM_FREQUENCY_HZ;
static constexpr uint32_t FOC_CONTROL_PERIOD_US =
    1000000 / FOC_CONTROL_FREQUENCY_HZ;
static constexpr uint32_t CONTROL_TIMER_RESOLUTION_HZ = PWM_RESOLUTION_HZ;
static constexpr float FOC_CONTROL_PERIOD_S = 0.00005f;
static constexpr float SPEED_CONTROL_PERIOD_S = 0.001f;
static constexpr float CURRENT_PI_KP = 0.54f;
static constexpr float CURRENT_PI_KI = 400.0f;
static constexpr float CURRENT_PI_INTEGRAL_LIMIT_V = 1.0f;

static constexpr float CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S = 5.0f;
static constexpr float CLOSED_LOOP_SPEED_RAMP_RAD_S2 = 20.0f;
static constexpr float CLOSED_LOOP_MAX_Q_CURRENT_A = 0.05f;
static constexpr float SPEED_PI_KP_A_PER_RAD_S = 0.01f;
static constexpr float SPEED_PI_KI_A_PER_RAD_S2 = 0.005f;
static constexpr float SPEED_PI_INTEGRAL_LIMIT_A = 0.003f;
static constexpr float MOTOR_MAX_MECHANICAL_SPEED_RAD_S = 30.0f;
// 允许采样瞬时速度高于保护阈值，用于过滤明显错误的角度跳变。
static constexpr float ENCODER_MAX_SAMPLE_SPEED_RAD_S = 120.0f;
static constexpr uint32_t SPEED_OVERSPEED_CONFIRMATION_COUNT = 3;
static constexpr uint32_t SPEED_LOG_WINDOW_MAX_US = 2000000;
// 当前已验证 0.5 系数能够可靠起转，后续再单独优化速度反馈平滑度。
static constexpr float SPEED_FILTER_ALPHA = 0.5f;
static constexpr uint32_t VELOCITY_ESTIMATION_PERIOD_US = 20000;
static constexpr float STARTUP_KICK_Q_CURRENT_A = 0.05f;
static constexpr uint32_t STARTUP_KICK_DURATION_MS = 100;
static constexpr uint32_t CONTROL_TARGET_HANDOFF_DELAY_MS = 10;

static constexpr float CURRENT_SHUNT_RESISTOR_OHM = 0.01f;
static constexpr float CURRENT_AMPLIFIER_GAIN = 50.0f;
static constexpr float CURRENT_SENSE_SIGN = -1.0f;
static constexpr float CURRENT_SCALE_A_PER_MV =
    1.0f / CURRENT_SHUNT_RESISTOR_OHM /
        CURRENT_AMPLIFIER_GAIN / 1000.0f;

static constexpr uint32_t CURRENT_OFFSET_SAMPLE_COUNT = 1000;
static constexpr uint32_t CURRENT_SAMPLE_COUNT = 5;
static constexpr uint32_t CURRENT_SAMPLE_SPACING_US = 6;
static constexpr uint32_t CURRENT_SAMPLE_TIMEOUT_US = 5000;
static constexpr float ALIGNMENT_VOLTAGE_V = 0.4f;
static constexpr float ALIGNMENT_MAX_PHASE_CURRENT_A = 0.35f;
static constexpr uint32_t ALIGNMENT_DURATION_MS = 500;
static constexpr uint32_t AUTO_START_DELAY_MS = 1000;
static constexpr uint32_t TEST_RUN_DURATION_MS = 60000;
static constexpr uint32_t CONTROL_TASK_PERIOD_MS = 1;
static constexpr BaseType_t CONTROL_ISR_CORE = 1;
static constexpr BaseType_t CONTROL_TASK_CORE = 0;
static constexpr uint32_t LOG_PERIOD_MS = 500;

/* ---- 硬件运行状态 ---- */

static foc_core motor;
static i2c_bus encoder_bus(I2C_NUM_0,
    ENCODER_SDA_PIN,
    ENCODER_SCL_PIN,
    I2C_ENABLE_INTERNAL_PULLUP);
static i2c_device encoder_device(encoder_bus,
    AS5600_ADDRESS,
    I2C_SPEED_HZ);

static adc_continuous_handle_t adc1_continuous = nullptr;
static adc_oneshot_unit_handle_t adc2_unit = nullptr;
static adc_cali_handle_t adc2_calibration = nullptr;

static bool pwm_initialized = false;
static bool bus_sense_available = false;
static bool module_initialized = false;
static volatile DRAM_ATTR uint32_t current_raw_packed = 0;
static volatile DRAM_ATTR uint32_t current_raw_version = 0;
static volatile DRAM_ATTR uint32_t current_raw_sequence = 0;
static volatile DRAM_ATTR uint32_t current_adc_frame_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_read_error_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_frame_size = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_sample_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_a_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_b_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_invalid_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_commit_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_commit_sequence = 0;
static volatile DRAM_ATTR uint32_t current_adc_last_commit_timestamp_us = 0;
static volatile DRAM_ATTR uint32_t current_adc_update_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_no_pair_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_update_stage = 0;
static volatile DRAM_ATTR uint32_t current_adc_dma_callback_count = 0;
static volatile DRAM_ATTR uint8_t current_adc_dma_core_id = 0xff;
static volatile DRAM_ATTR uint32_t current_adc_dma_last_timestamp_us = 0;
static volatile DRAM_ATTR uint32_t current_adc_start_frame_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_start_update_count = 0;
static volatile DRAM_ATTR uint32_t current_adc_start_dma_callback_count = 0;
static float current_offset_a_mv = 0.0f;
static float current_offset_b_mv = 0.0f;
static bool encoder_tracking_valid = false;
static float encoder_previous_angle_rad = 0.0f;
static float encoder_unwrapped_angle_rad = 0.0f;
static int64_t encoder_previous_timestamp_us = 0;
static bool encoder_velocity_reference_valid = false;
static float encoder_velocity_reference_angle_rad = 0.0f;
static int64_t encoder_velocity_reference_timestamp_us = 0;
static float encoder_velocity_rad_s = 0.0f;
static std::atomic<uint32_t> encoder_rejected_sample_count{0};
static volatile float speed_feedback_for_log_rad_s = 0.0f;
static uint32_t sample_sequence = 0;
static int32_t bus_sense_voltage_mv = 0;
static bool bus_sense_read_valid = false;
static uint32_t bus_sense_last_duration_us = 0;
static uint32_t bus_sense_max_duration_us = 0;
static mcpwm_timer_handle_t control_timer = nullptr;
static mcpwm_oper_handle_t pwm_operators[3] = {};
static mcpwm_cmpr_handle_t pwm_comparators[3] = {};
static mcpwm_gen_handle_t pwm_generators[3] = {};
static bool control_timer_started = false;
static volatile DRAM_ATTR bool control_timer_running = false;
static volatile DRAM_ATTR bool control_isr_in_progress = false;
static volatile DRAM_ATTR bool control_isr_fault = false;
static volatile DRAM_ATTR uint8_t control_isr_last_result =
    static_cast<uint8_t>(foc_result::NOT_READY);
static volatile DRAM_ATTR uint8_t control_isr_fault_result =
    static_cast<uint8_t>(foc_result::NOT_READY);
static volatile DRAM_ATTR uint32_t control_isr_tick_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_last_timestamp_us = 0;
static volatile DRAM_ATTR uint32_t control_isr_last_cycles = 0;
static volatile DRAM_ATTR uint32_t control_isr_max_cycles = 0;
static volatile DRAM_ATTR uint8_t control_isr_core_id = 0xff;
static volatile DRAM_ATTR esp_err_t control_timer_initialize_result = ESP_FAIL;
static volatile DRAM_ATTR uint32_t control_isr_fault_timestamp_us = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_current_age_us = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_raw_sequence = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_locked_sequence = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_unlocked_sequence = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_sample_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_a_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_b_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_invalid_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_commit_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_commit_sequence = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_commit_age_us = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_update_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_no_pair_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_update_stage = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_dma_callback_count = 0;
static volatile DRAM_ATTR uint8_t control_isr_fault_dma_core_id = 0xff;
static volatile DRAM_ATTR uint32_t control_isr_fault_dma_age_us = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_frame_delta = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_update_delta = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_dma_delta = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_snapshot_retry_count = 0;
static volatile DRAM_ATTR uint8_t control_isr_fault_rotor_peek_ok = 0;
static volatile DRAM_ATTR uint8_t control_isr_fault_rotor_valid = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_rotor_age_us = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_task_iteration_count = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_task_stage = 0;
static volatile DRAM_ATTR uint32_t control_isr_fault_task_stage_age_us = 0;
static volatile DRAM_ATTR uint32_t current_isr_last_timestamp_us = 0;
static volatile DRAM_ATTR uint32_t current_isr_last_raw_sequence = 0;
static volatile DRAM_ATTR uint32_t current_isr_unlocked_raw_sequence = 0;
static volatile DRAM_ATTR uint16_t current_isr_last_raw_a = 0;
static volatile DRAM_ATTR uint16_t current_isr_last_raw_b = 0;
static volatile DRAM_ATTR uint32_t current_isr_last_good_raw_sequence = 0;
static volatile DRAM_ATTR uint32_t current_isr_snapshot_retry_count = 0;
static foc_topic_access *control_topics = nullptr;
static uint32_t current_isr_sequence = 0;
static uint32_t rotor_read_success_count = 0;
static uint32_t rotor_read_failure_count = 0;
static uint32_t rotor_last_published_timestamp_us = 0;
static uint32_t rotor_publish_count = 0;
static uint32_t rotor_last_publish_interval_us = 0;
static uint32_t rotor_max_publish_interval_us = 0;
static uint32_t encoder_read_attempt_failure_count = 0;
static uint32_t encoder_bus_reset_count = 0;
static uint32_t encoder_last_read_duration_us = 0;
static uint32_t encoder_max_read_duration_us = 0;
static volatile DRAM_ATTR uint32_t control_task_iteration_count = 0;
static volatile DRAM_ATTR uint32_t control_task_stage = 0;
static volatile DRAM_ATTR uint32_t control_task_stage_timestamp_us = 0;

/**
 * @brief 记录 ADC Continuous DMA 转换完成中断的位置
 *
 * @param handle ADC Continuous 句柄
 * @param event_data 转换帧事件数据
 * @param user_data 用户上下文，未使用
 *
 * @return 不请求任务切换
 *
 * @note 该回调只记录诊断信息，不读取 DMA 帧、不解析数据、不打印日志。
 */
static bool IRAM_ATTR adc_conversion_done_callback(
    adc_continuous_handle_t handle,
    const adc_continuous_evt_data_t *event_data,
    void *user_data)
{
    (void)handle;
    (void)event_data;
    (void)user_data;
    uint32_t next_callback_count = current_adc_dma_callback_count;
    current_adc_dma_callback_count = next_callback_count + 1;
    current_adc_dma_core_id = static_cast<uint8_t>(xPortGetCoreID());
    current_adc_dma_last_timestamp_us = static_cast<uint32_t>(
        esp_timer_get_time());
    return false;
}

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
 * @brief 将数值限制到对称区间
 *
 * @param value 输入数值
 * @param limit 对称区间的绝对值上限
 *
 * @return 限制后的数值
 */
static float clamp_symmetric(float value, float limit)
{
    if(value > limit){return limit;}
    if(value < -limit){return -limit;}
    return value;
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
 * @brief 根据速度误差计算 q 轴电流目标
 *
 * @param target_speed_rad_s 目标机械速度，单位弧度每秒
 * @param measured_speed_rad_s 反馈机械速度，单位弧度每秒
 * @param integral_a 速度 PI 积分状态，单位安培
 *
 * @return q 轴电流目标，单位安培
 *
 * @note 该速度环位于 foc_core 的电流环之外，只在任务上下文运行。
 */
static float calculate_speed_q_current_target(float target_speed_rad_s,
    float measured_speed_rad_s,
    float &integral_a)
{
    float speed_error_rad_s = target_speed_rad_s - measured_speed_rad_s;
    float integral_candidate_a = integral_a +
        SPEED_PI_KI_A_PER_RAD_S2 * speed_error_rad_s *
            SPEED_CONTROL_PERIOD_S;
    integral_a = clamp_symmetric(integral_candidate_a,
        SPEED_PI_INTEGRAL_LIMIT_A);

    float current_target_a = SPEED_PI_KP_A_PER_RAD_S *
        speed_error_rad_s + integral_a;
    return clamp_symmetric(current_target_a,
        CLOSED_LOOP_MAX_Q_CURRENT_A);
}

/**
 * @brief 释放 MCPWM 三相 PWM 资源
 */
static void deinitialize_power_output()
{
    for(uint32_t index = 0; index < 3; index++)
    {
        if(pwm_generators[index])
        {
            mcpwm_del_generator(pwm_generators[index]);
            pwm_generators[index] = nullptr;
        }
    }
    for(uint32_t index = 0; index < 3; index++)
    {
        if(pwm_comparators[index])
        {
            mcpwm_del_comparator(pwm_comparators[index]);
            pwm_comparators[index] = nullptr;
        }
    }
    for(uint32_t index = 0; index < 3; index++)
    {
        if(pwm_operators[index])
        {
            mcpwm_del_operator(pwm_operators[index]);
            pwm_operators[index] = nullptr;
        }
    }
    if(control_timer)
    {
        mcpwm_del_timer(control_timer);
        control_timer = nullptr;
    }
    pwm_initialized = false;
}

/**
 * @brief 初始化 MCPWM 三相 PWM 和使能 GPIO
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

    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = PWM_GROUP_ID;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = CONTROL_TIMER_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = PWM_PERIOD_TICKS;
    // 经典 ESP32 的 FOC ISR 会执行浮点运算，固定使用 Level-1，配合
    // CONFIG_FREERTOS_FPU_IN_ISR，避免由驱动自动选择到不匹配的中断级别。
    timer_config.intr_priority = 1;
    timer_config.flags.allow_pd = false;

    error = mcpwm_new_timer(&timer_config, &control_timer);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "创建 MCPWM 定时器失败: %d", static_cast<int>(error));
        return false;
    }

    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = PWM_GROUP_ID;
    for(uint32_t index = 0; index < 3; index++)
    {
        error = mcpwm_new_operator(&operator_config,
            &pwm_operators[index]);
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "创建 MCPWM operator%lu 失败: %d",
                static_cast<unsigned long>(index),
                static_cast<int>(error));
            deinitialize_power_output();
            return false;
        }

        error = mcpwm_operator_connect_timer(pwm_operators[index],
            control_timer);
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "连接 MCPWM operator%lu 失败: %d",
                static_cast<unsigned long>(index),
                static_cast<int>(error));
            deinitialize_power_output();
            return false;
        }
    }

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;
    for(uint32_t index = 0; index < 3; index++)
    {
        error = mcpwm_new_comparator(pwm_operators[index],
            &comparator_config,
            &pwm_comparators[index]);
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "创建 MCPWM comparator%lu 失败: %d",
                static_cast<unsigned long>(index),
                static_cast<int>(error));
            deinitialize_power_output();
            return false;
        }
    }

    mcpwm_generator_config_t generator_config = {};
    for(uint32_t index = 0; index < 3; index++)
    {
        generator_config.gen_gpio_num = static_cast<int>(PWM_PINS[index]);
        error = mcpwm_new_generator(pwm_operators[index],
            &generator_config,
            &pwm_generators[index]);
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "创建 MCPWM generator%lu 失败: %d",
                static_cast<unsigned long>(index),
                static_cast<int>(error));
            deinitialize_power_output();
            return false;
        }

        error = mcpwm_generator_set_action_on_timer_event(
            pwm_generators[index],
            MCPWM_GEN_TIMER_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY,
                MCPWM_GEN_ACTION_HIGH));
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "配置 MCPWM generator%lu 周期动作失败: %d",
                static_cast<unsigned long>(index),
                static_cast<int>(error));
            deinitialize_power_output();
            return false;
        }

        error = mcpwm_generator_set_action_on_compare_event(
            pwm_generators[index],
            MCPWM_GEN_COMPARE_EVENT_ACTION(
                MCPWM_TIMER_DIRECTION_UP,
                pwm_comparators[index],
                MCPWM_GEN_ACTION_LOW));
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "配置 MCPWM generator%lu 比较动作失败: %d",
                static_cast<unsigned long>(index),
                static_cast<int>(error));
            deinitialize_power_output();
            return false;
        }
    }

    pwm_initialized = true;
    return true;
}

/**
 * @brief 将占空比转换为 MCPWM 比较值
 *
 * @param duty 占空比，范围零至一
 *
 * @return MCPWM 比较计数值
 */
static uint32_t IRAM_ATTR duty_to_counts(float duty)
{
    return static_cast<uint32_t>(duty * PWM_PERIOD_TICKS + 0.5f);
}

/**
 * @brief 向 MCPWM 比较器写入单路占空比
 *
 * @param comparator MCPWM 比较器
 * @param duty 占空比，范围零至一
 *
 * @return 写入结果
 *
 * @note MCPWM 比较值接口同时支持任务和 ISR 上下文；该函数保持在
 *       IRAM 中，供 20 kHz 控制 ISR 调用。
 */
static foc_result IRAM_ATTR write_pwm_compare(
    mcpwm_cmpr_handle_t comparator,
    float duty)
{
    if(!comparator)
    {
        return foc_result::DRIVER_FAULT;
    }

    esp_err_t error = mcpwm_comparator_set_compare_value(comparator,
        duty_to_counts(duty));
    return error == ESP_OK ? foc_result::OK : foc_result::DRIVER_FAULT;
}

/**
 * @brief 向三相 MCPWM 通道写入占空比
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

    if(write_pwm_compare(pwm_comparators[0], duty.phase_a) !=
        foc_result::OK)
    {
        return foc_result::DRIVER_FAULT;
    }
    if(write_pwm_compare(pwm_comparators[1], duty.phase_b) !=
        foc_result::OK)
    {
        return foc_result::DRIVER_FAULT;
    }
    if(write_pwm_compare(pwm_comparators[2], duty.phase_c) !=
        foc_result::OK)
    {
        return foc_result::DRIVER_FAULT;
    }
    return foc_result::OK;
}

/**
 * @brief 在 ISR 中更新三相 MCPWM 占空比
 *
 * @param duty 三相占空比
 *
 * @return 写入结果
 *
 * @note MCPWM 比较器值在 TEZ 处更新，三相写入均由 MCPWM 硬件同步输出。
 */
static foc_result IRAM_ATTR write_pwm_duty_from_isr(const phase_duty &duty)
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

    if(write_pwm_compare(pwm_comparators[0], duty.phase_a) !=
        foc_result::OK)
    {
        return foc_result::DRIVER_FAULT;
    }
    if(write_pwm_compare(pwm_comparators[1], duty.phase_b) !=
        foc_result::OK)
    {
        return foc_result::DRIVER_FAULT;
    }
    if(write_pwm_compare(pwm_comparators[2], duty.phase_c) !=
        foc_result::OK)
    {
        return foc_result::DRIVER_FAULT;
    }
    return foc_result::OK;
}

/**
 * @brief 在 ISR 中直接控制驱动器使能脚
 *
 * @param enabled 是否打开驱动器
 */
static void IRAM_ATTR set_enable_pin_from_isr(bool enabled)
{
    constexpr uint32_t enable_pin_mask =
        1U << static_cast<uint32_t>(ENABLE_PIN);
    if(enabled)
    {
        GPIO.out_w1ts = enable_pin_mask;
    }
    else
    {
        GPIO.out_w1tc = enable_pin_mask;
    }
}

/**
 * @brief 关闭功率级并恢复三相中性占空比
 */
static void IRAM_ATTR disable_power_stage()
{
    phase_duty neutral_duty{};
    write_pwm_duty_from_isr(neutral_duty);
    set_enable_pin_from_isr(false);
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
 * @brief 将一帧 ADC1 连续采样结果更新到电流快照
 *
 * @param samples 已解析的 ADC 样本数组
 * @param sample_count 样本数量
 *
 * @note 只在同一帧同时收到 O1/O2 时提交新快照，避免两路电流来自
 *       不同采样时刻。该函数运行在 ADC 读取任务中，不在 ISR 中解析。
 */
static void update_current_raw_samples(
    const adc_continuous_data_t *samples,
    uint32_t sample_count)
{
    uint32_t next_update_count = current_adc_update_count;
    current_adc_update_count = next_update_count + 1;
    current_adc_update_stage = 1;
    uint16_t next_current_a_raw = 0;
    uint16_t next_current_b_raw = 0;
    bool has_current_a = false;
    bool has_current_b = false;
    uint32_t current_a_count = 0;
    uint32_t current_b_count = 0;
    uint32_t invalid_count = 0;
    for(uint32_t index = 0; index < sample_count; index++)
    {
        if(!samples[index].valid)
        {
            invalid_count++;
            continue;
        }
        if(samples[index].channel == CURRENT_A_CHANNEL)
        {
            next_current_a_raw = static_cast<uint16_t>(
                samples[index].raw_data);
            has_current_a = true;
            current_a_count++;
        }
        else if(samples[index].channel == CURRENT_B_CHANNEL)
        {
            next_current_b_raw = static_cast<uint16_t>(
                samples[index].raw_data);
            has_current_b = true;
            current_b_count++;
        }
    }

    current_adc_update_stage = 2;
    current_adc_last_sample_count = sample_count;
    current_adc_last_a_count = current_a_count;
    current_adc_last_b_count = current_b_count;
    current_adc_last_invalid_count = invalid_count;

    if(!has_current_a || !has_current_b)
    {
        uint32_t next_no_pair_count = current_adc_no_pair_count;
        current_adc_no_pair_count = next_no_pair_count + 1;
        return;
    }

    current_adc_update_stage = 3;
    uint32_t commit_timestamp_us = static_cast<uint32_t>(
        esp_timer_get_time());
    uint32_t packed_raw = static_cast<uint32_t>(next_current_a_raw) |
        (static_cast<uint32_t>(next_current_b_raw) << 16);
    uint32_t next_sequence = current_raw_sequence;
    next_sequence++;
    uint32_t writing_version = (next_sequence << 1) | 1U;
    __atomic_store_n(&current_raw_version,
        writing_version,
        __ATOMIC_RELEASE);
    current_adc_update_stage = 4;
    __atomic_store_n(&current_raw_packed,
        packed_raw,
        __ATOMIC_RELAXED);
    current_raw_sequence = next_sequence;
    uint32_t next_commit_count = current_adc_commit_count;
    current_adc_commit_count = next_commit_count + 1;
    current_adc_last_commit_sequence = current_raw_sequence;
    current_adc_last_commit_timestamp_us = commit_timestamp_us;
    __atomic_store_n(&current_raw_version,
        next_sequence << 1,
        __ATOMIC_RELEASE);
    current_adc_update_stage = 5;
}

struct current_raw_snapshot
{
    uint16_t raw_a = 0;
    uint16_t raw_b = 0;
    uint32_t sequence = 0;
    bool valid = false;
};

/**
 * @brief 无阻塞读取 ADC 双通道原始值快照
 *
 * @return 一致且已经提交的原始值快照
 *
 * @note 生产者先发布奇数版本号，再写入打包后的两个原始值，最后发布
 *       偶数版本号。读取方只接受两次读取到的版本号相同且为偶数的结果，
 *       因此 ISR 不需要等待任务释放自旋锁。
 */
static current_raw_snapshot IRAM_ATTR read_current_raw_snapshot()
{
    for(uint32_t attempt = 0; attempt < 2; attempt++)
    {
        uint32_t version_before = __atomic_load_n(&current_raw_version,
            __ATOMIC_ACQUIRE);
        if((version_before & 1U) != 0U)
        {
            continue;
        }

        uint32_t packed_raw = __atomic_load_n(&current_raw_packed,
            __ATOMIC_RELAXED);
        uint32_t version_after = __atomic_load_n(&current_raw_version,
            __ATOMIC_ACQUIRE);
        if(version_before == version_after)
        {
            current_raw_snapshot snapshot{};
            snapshot.raw_a = static_cast<uint16_t>(packed_raw & 0xffffU);
            snapshot.raw_b = static_cast<uint16_t>(packed_raw >> 16);
            snapshot.sequence = version_after >> 1;
            snapshot.valid = snapshot.sequence != 0;
            return snapshot;
        }
    }

    return {};
}

/**
 * @brief 从 ADC Continuous 驱动读取并解析电流采样帧
 *
 * @param argument 未使用
 *
 * @note 该任务固定运行在 CPU0。ADC 驱动 ISR 只负责把 DMA 帧放入
 *       驱动内部环形缓冲区，由此任务取出并更新供 MCPWM ISR 消费的
 *       最新双通道快照。
 */
static void adc_current_reader_task(void *argument)
{
    (void)argument;
    uint8_t raw_data[CURRENT_ADC_FRAME_SIZE] = {};
    adc_continuous_data_t parsed_data[
        CURRENT_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES] = {};

    while(true)
    {
        uint32_t bytes_read = 0;
        esp_err_t error = adc_continuous_read(adc1_continuous,
            raw_data,
            sizeof(raw_data),
            &bytes_read,
            ADC_MAX_DELAY);
        if(error != ESP_OK)
        {
            uint32_t next_error_count = current_adc_read_error_count;
            current_adc_read_error_count = next_error_count + 1;
            ESP_LOGE(TAG, "读取 ADC1 连续采样失败: %d",
                static_cast<int>(error));
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        uint32_t sample_count = 0;
        error = adc_continuous_parse_data(adc1_continuous,
            raw_data,
            bytes_read,
            parsed_data,
            &sample_count);
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "解析 ADC1 连续采样失败: %d",
                static_cast<int>(error));
            continue;
        }

        uint32_t next_frame_count = current_adc_frame_count;
        current_adc_frame_count = next_frame_count + 1;
        current_adc_last_frame_size = bytes_read;
        update_current_raw_samples(parsed_data, sample_count);
        taskYIELD();
    }
}

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
    current_raw_packed = 0;
    current_raw_version = 0;
    current_raw_sequence = 0;
    current_isr_last_raw_a = 0;
    current_isr_last_raw_b = 0;
    current_isr_last_good_raw_sequence = 0;
    current_isr_snapshot_retry_count = 0;
    current_adc_commit_count = 0;
    current_adc_last_commit_sequence = 0;
    current_adc_last_commit_timestamp_us = 0;
    current_adc_update_count = 0;
    current_adc_no_pair_count = 0;
    current_adc_update_stage = 0;
    current_adc_dma_callback_count = 0;
    current_adc_dma_core_id = 0xff;
    current_adc_dma_last_timestamp_us = 0;
    current_adc_start_frame_count = 0;
    current_adc_start_update_count = 0;
    current_adc_start_dma_callback_count = 0;

    adc_continuous_handle_cfg_t continuous_handle_config = {};
    continuous_handle_config.max_store_buf_size = CURRENT_ADC_FRAME_SIZE * 4;
    continuous_handle_config.conv_frame_size = CURRENT_ADC_FRAME_SIZE;
    continuous_handle_config.flags.flush_pool = true;

    esp_err_t error = adc_continuous_new_handle(&continuous_handle_config,
        &adc1_continuous);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "初始化 ADC1 连续采样失败: %d",
            static_cast<int>(error));
        return false;
    }

    adc_digi_pattern_config_t adc_pattern[2] = {};
    adc_pattern[0].atten = ADC_ATTENUATION;
    adc_pattern[0].channel = CURRENT_A_CHANNEL;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    adc_pattern[1].atten = ADC_ATTENUATION;
    adc_pattern[1].channel = CURRENT_B_CHANNEL;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    adc_continuous_config_t continuous_config = {};
    continuous_config.pattern_num = 2;
    continuous_config.adc_pattern = adc_pattern;
    continuous_config.sample_freq_hz = CURRENT_ADC_SAMPLE_FREQ_HZ;
    continuous_config.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    error = adc_continuous_config(adc1_continuous, &continuous_config);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "配置 ADC1 连续采样通道失败: %d",
            static_cast<int>(error));
        adc_continuous_deinit(adc1_continuous);
        adc1_continuous = nullptr;
        return false;
    }

    adc_continuous_evt_cbs_t callbacks = {};
    callbacks.on_conv_done = adc_conversion_done_callback;
    error = adc_continuous_register_event_callbacks(adc1_continuous,
        &callbacks,
        nullptr);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "注册 ADC1 连续采样回调失败: %d",
            static_cast<int>(error));
        adc_continuous_deinit(adc1_continuous);
        adc1_continuous = nullptr;
        return false;
    }

    error = adc_continuous_start(adc1_continuous);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "启动 ADC1 连续采样失败: %d", static_cast<int>(error));
        adc_continuous_deinit(adc1_continuous);
        adc1_continuous = nullptr;
        return false;
    }

    BaseType_t task_result = xTaskCreatePinnedToCore(
        adc_current_reader_task,
        "adc_current",
        3072,
        nullptr,
        CURRENT_ADC_TASK_PRIORITY,
        nullptr,
        CURRENT_ADC_TASK_CORE);
    if(task_result != pdPASS)
    {
        ESP_LOGE(TAG, "创建 ADC1 电流采样任务失败");
        adc_continuous_stop(adc1_continuous);
        adc_continuous_deinit(adc1_continuous);
        adc1_continuous = nullptr;
        return false;
    }

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

    adc_oneshot_chan_cfg_t channel_config = {};
    channel_config.atten = ADC_ATTENUATION;
    channel_config.bitwidth = ADC_BITWIDTH;
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
    current_raw_snapshot initial_snapshot = read_current_raw_snapshot();
    if(!initial_snapshot.valid)
    {
        return false;
    }

    int32_t samples_a_mv[CURRENT_SAMPLE_COUNT] = {};
    int32_t samples_b_mv[CURRENT_SAMPLE_COUNT] = {};
    for(uint32_t index = 0; index < CURRENT_SAMPLE_COUNT; index++)
    {
        current_raw_snapshot snapshot = read_current_raw_snapshot();
        if(!snapshot.valid)
        {
            return false;
        }
        uint16_t raw_a = snapshot.raw_a;
        uint16_t raw_b = snapshot.raw_b;
        samples_a_mv[index] = static_cast<int32_t>(
            static_cast<float>(raw_a) * ADC_RAW_TO_MV);
        samples_b_mv[index] = static_cast<int32_t>(
            static_cast<float>(raw_b) * ADC_RAW_TO_MV);
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
static void IRAM_ATTR convert_current(int32_t voltage_a_mv,
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
    int64_t read_start_us = esp_timer_get_time();
    uint8_t data[2] = {};
    i2c_result result = i2c_result::BUS_ERROR;
    for(uint32_t attempt = 0;
        attempt <= ENCODER_READ_RETRY_COUNT;
        ++attempt)
    {
        result = encoder_device.read_bytes(
            AS5600_RAW_ANGLE_REGISTER,
            data,
            2,
            5);
        if(result == i2c_result::OK)
        {
            break;
        }
        if(attempt == ENCODER_READ_RETRY_COUNT)
        {
            break;
        }
        encoder_read_attempt_failure_count++;
        if(result == i2c_result::TRANSFER_TIMEOUT ||
            result == i2c_result::BUS_ERROR ||
            result == i2c_result::NACK)
        {
            encoder_bus_reset_count++;
            encoder_bus.reset();
        }
        vTaskDelay(pdMS_TO_TICKS(ENCODER_RETRY_DELAY_MS));
    }
    if(result != i2c_result::OK)
    {
        uint32_t read_duration_us = static_cast<uint32_t>(
            esp_timer_get_time() - read_start_us);
        encoder_last_read_duration_us = read_duration_us;
        if(read_duration_us > encoder_max_read_duration_us)
        {
            encoder_max_read_duration_us = read_duration_us;
        }
        ESP_LOGW(TAG,
            "AS5600 读取失败: result=%d，已尝试复位 I2C；请重新插拔驱动板供电后重试",
            static_cast<int>(result));
        return false;
    }

    uint32_t read_duration_us = static_cast<uint32_t>(
        esp_timer_get_time() - read_start_us);
    encoder_last_read_duration_us = read_duration_us;
    if(read_duration_us > encoder_max_read_duration_us)
    {
        encoder_max_read_duration_us = read_duration_us;
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
    encoder_velocity_reference_valid = false;
    encoder_velocity_reference_angle_rad = 0.0f;
    encoder_velocity_reference_timestamp_us = 0;
    encoder_velocity_rad_s = 0.0f;
}

static bool start_pwm_carrier_for_alignment();
static bool stop_pwm_carrier_for_alignment();

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

    if(!start_pwm_carrier_for_alignment())
    {
        ESP_LOGE(TAG, "传感器对齐时无法启动 PWM 载波");
        return false;
    }

    if(write_pwm_duty(alignment_duty) != foc_result::OK)
    {
        ESP_LOGE(TAG, "输出传感器对齐电压失败");
        stop_pwm_carrier_for_alignment();
        return false;
    }

    if(!enable_power_stage())
    {
        ESP_LOGE(TAG, "传感器对齐时无法打开驱动器");
        disable_power_stage();
        stop_pwm_carrier_for_alignment();
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
            stop_pwm_carrier_for_alignment();
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
            stop_pwm_carrier_for_alignment();
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    float aligned_mechanical_angle_rad = 0.0f;
    if(!read_encoder_angle(aligned_mechanical_angle_rad))
    {
        disable_power_stage();
        stop_pwm_carrier_for_alignment();
        ESP_LOGE(TAG, "传感器对齐后无法读取机械角度");
        return false;
    }

    electrical_zero_offset_rad = normalize_angle(
        static_cast<float>(MOTOR_ROTOR_DIRECTION) *
        static_cast<float>(MOTOR_POLE_PAIRS) *
        aligned_mechanical_angle_rad);
    disable_power_stage();
    if(!stop_pwm_carrier_for_alignment())
    {
        ESP_LOGE(TAG, "停止传感器对齐 PWM 载波失败");
        return false;
    }
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

    if(!encoder_tracking_valid)
    {
        encoder_previous_angle_rad = angle_rad;
        encoder_unwrapped_angle_rad = angle_rad;
        encoder_previous_timestamp_us = timestamp_us;
        encoder_velocity_reference_angle_rad =
            encoder_unwrapped_angle_rad;
        encoder_velocity_reference_timestamp_us = timestamp_us;
        encoder_velocity_reference_valid = true;
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

        float sample_speed_rad_s = delta_angle_rad /
            (static_cast<float>(delta_time_us) * 1.0e-6f);
        if(absolute_value(sample_speed_rad_s) >
            ENCODER_MAX_SAMPLE_SPEED_RAD_S)
        {
            encoder_rejected_sample_count.fetch_add(1,
                std::memory_order_relaxed);
            return false;
        }

        encoder_unwrapped_angle_rad += delta_angle_rad;
        encoder_previous_angle_rad = angle_rad;
        encoder_previous_timestamp_us = timestamp_us;

        if(encoder_velocity_reference_valid)
        {
            int64_t velocity_delta_time_us = timestamp_us -
                encoder_velocity_reference_timestamp_us;
            if(velocity_delta_time_us >=
                static_cast<int64_t>(VELOCITY_ESTIMATION_PERIOD_US))
            {
                encoder_velocity_rad_s =
                    (encoder_unwrapped_angle_rad -
                        encoder_velocity_reference_angle_rad) /
                    (static_cast<float>(velocity_delta_time_us) * 1.0e-6f);
                encoder_velocity_reference_angle_rad =
                    encoder_unwrapped_angle_rad;
                encoder_velocity_reference_timestamp_us = timestamp_us;
            }
        }
    }

    sample.mechanical_angle_rad = encoder_unwrapped_angle_rad;
    sample.mechanical_velocity_rad_s = encoder_velocity_rad_s;
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
static void IRAM_ATTR foc_output_disable(void *context)
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
 * @brief 应用 ISR 上下文中的三相占空比
 *
 * @param context 输出上下文
 * @param duty 三相占空比
 *
 * @return 输出结果
 */
static foc_result IRAM_ATTR foc_output_apply_duty_from_isr(void *context,
    const phase_duty &duty)
{
    return write_pwm_duty_from_isr(duty);
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
    int64_t read_start_us = esp_timer_get_time();
    if(!bus_sense_available)
    {
        bus_sense_read_valid = false;
        bus_sense_last_duration_us = static_cast<uint32_t>(
            esp_timer_get_time() - read_start_us);
        return;
    }

    bus_sense_read_valid = read_adc_millivolts(adc2_unit,
        adc2_calibration,
        BUS_SENSE_CHANNEL,
        bus_sense_voltage_mv);
    uint32_t read_duration_us = static_cast<uint32_t>(
        esp_timer_get_time() - read_start_us);
    bus_sense_last_duration_us = read_duration_us;
    if(read_duration_us > bus_sense_max_duration_us)
    {
        bus_sense_max_duration_us = read_duration_us;
    }
}

/**
 * @brief 在控制 ISR 中将最新 ADC 原始值发布为电流 Topic
 *
 * @param timestamp_us 当前控制周期时间戳，单位微秒
 * @param higher_priority_task_woken ISR 任务唤醒标记
 *
 * @return 发布成功时返回 true
 */
static bool IRAM_ATTR publish_current_sample_from_isr(
    uint32_t timestamp_us,
    BaseType_t &higher_priority_task_woken)
{
    current_raw_snapshot raw_snapshot = read_current_raw_snapshot();
    if(raw_snapshot.valid)
    {
        current_isr_last_raw_a = raw_snapshot.raw_a;
        current_isr_last_raw_b = raw_snapshot.raw_b;
        current_isr_last_good_raw_sequence = raw_snapshot.sequence;
    }
    else
    {
        uint32_t next_retry_count = current_isr_snapshot_retry_count;
        current_isr_snapshot_retry_count = next_retry_count + 1;
        raw_snapshot.raw_a = current_isr_last_raw_a;
        raw_snapshot.raw_b = current_isr_last_raw_b;
        raw_snapshot.sequence = current_isr_last_good_raw_sequence;
        raw_snapshot.valid = raw_snapshot.sequence != 0;
    }
    uint32_t raw_sequence = raw_snapshot.sequence;
    uint16_t raw_a = raw_snapshot.raw_a;
    uint16_t raw_b = raw_snapshot.raw_b;
    current_isr_unlocked_raw_sequence = raw_sequence;
    if(raw_sequence != current_isr_last_raw_sequence)
    {
        current_isr_last_raw_sequence = raw_sequence;
        current_isr_last_timestamp_us = timestamp_us;
    }

    phase_current_sample sample{};
    sample.sequence = ++current_isr_sequence;
    sample.timestamp_us = current_isr_last_timestamp_us;
    sample.valid = raw_snapshot.valid &&
        static_cast<uint32_t>(timestamp_us - current_isr_last_timestamp_us) <=
            CURRENT_SAMPLE_TIMEOUT_US;
    if(sample.valid)
    {
        int32_t voltage_a_mv = static_cast<int32_t>(
            static_cast<float>(raw_a) * ADC_RAW_TO_MV);
        int32_t voltage_b_mv = static_cast<int32_t>(
            static_cast<float>(raw_b) * ADC_RAW_TO_MV);
        convert_current(voltage_a_mv,
            voltage_b_mv,
            sample.phase_a_a,
            sample.phase_b_a,
            sample.phase_c_a);
    }

    if(!control_topics)
    {
        return false;
    }
    return control_topics->current.publish_from_isr(sample,
        higher_priority_task_woken);
}

/**
 * @brief 20 kHz MCPWM 定时器 ISR 回调
 *
 * @param timer MCPWM 定时器句柄
 * @param event_data MCPWM 定时器事件数据
 * @param user_data 用户上下文
 *
 * @return 是否请求调度更高优先级任务
 */
static bool IRAM_ATTR control_timer_alarm_callback(
    mcpwm_timer_handle_t timer,
    const mcpwm_timer_event_data_t *event_data,
    void *user_data)
{
    (void)timer;
    (void)event_data;
    (void)user_data;
    control_isr_in_progress = true;
    if(!control_timer_running || control_isr_fault)
    {
        control_isr_in_progress = false;
        return false;
    }

    control_isr_core_id = static_cast<uint8_t>(xPortGetCoreID());
    uint32_t start_cycles = esp_cpu_get_cycle_count();
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint32_t timestamp_us = static_cast<uint32_t>(esp_timer_get_time());
    publish_current_sample_from_isr(timestamp_us,
        higher_priority_task_woken);
    foc_result result = motor.core_loop_from_isr(timestamp_us,
        higher_priority_task_woken);
    uint32_t next_tick_count = control_isr_tick_count;
    control_isr_tick_count = next_tick_count + 1;
    control_isr_last_timestamp_us = timestamp_us;
    control_isr_last_result = static_cast<uint8_t>(result);
    if(result != foc_result::OK && result != foc_result::DISABLED)
    {
        control_isr_fault = true;
        control_isr_fault_result = static_cast<uint8_t>(result);
        control_isr_fault_timestamp_us = timestamp_us;
        control_isr_fault_current_age_us = static_cast<uint32_t>(
            timestamp_us - current_isr_last_timestamp_us);
        control_isr_fault_raw_sequence = current_raw_sequence;
        control_isr_fault_locked_sequence = current_isr_last_raw_sequence;
        control_isr_fault_unlocked_sequence =
            current_isr_unlocked_raw_sequence;
        control_isr_fault_sample_count = current_adc_last_sample_count;
        control_isr_fault_a_count = current_adc_last_a_count;
        control_isr_fault_b_count = current_adc_last_b_count;
        control_isr_fault_invalid_count = current_adc_last_invalid_count;
        control_isr_fault_commit_count = current_adc_commit_count;
        control_isr_fault_commit_sequence =
            current_adc_last_commit_sequence;
        control_isr_fault_commit_age_us = static_cast<uint32_t>(
            timestamp_us - current_adc_last_commit_timestamp_us);
        control_isr_fault_update_count = current_adc_update_count;
        control_isr_fault_no_pair_count = current_adc_no_pair_count;
        control_isr_fault_update_stage = current_adc_update_stage;
        control_isr_fault_dma_callback_count =
            current_adc_dma_callback_count;
        control_isr_fault_dma_core_id = current_adc_dma_core_id;
        control_isr_fault_dma_age_us = static_cast<uint32_t>(
            timestamp_us - current_adc_dma_last_timestamp_us);
        control_isr_fault_frame_delta = current_adc_frame_count -
            current_adc_start_frame_count;
        control_isr_fault_update_delta = current_adc_update_count -
            current_adc_start_update_count;
        control_isr_fault_dma_delta = current_adc_dma_callback_count -
            current_adc_start_dma_callback_count;
        control_isr_fault_snapshot_retry_count =
            current_isr_snapshot_retry_count;
        rotor_sample fault_rotor{};
        bool rotor_peek_ok = control_topics &&
            control_topics->rotor.peek_from_isr(fault_rotor);
        control_isr_fault_rotor_peek_ok = rotor_peek_ok ? 1 : 0;
        control_isr_fault_rotor_valid =
            rotor_peek_ok && fault_rotor.valid ? 1 : 0;
        control_isr_fault_rotor_age_us = rotor_peek_ok ?
            static_cast<uint32_t>(timestamp_us - fault_rotor.timestamp_us) :
            0;
        control_isr_fault_task_iteration_count = control_task_iteration_count;
        control_isr_fault_task_stage = control_task_stage;
        control_isr_fault_task_stage_age_us = static_cast<uint32_t>(
            timestamp_us - control_task_stage_timestamp_us);
    }
    uint32_t elapsed_cycles = esp_cpu_get_cycle_count() - start_cycles;
    control_isr_last_cycles = elapsed_cycles;
    if(elapsed_cycles > control_isr_max_cycles)
    {
        control_isr_max_cycles = elapsed_cycles;
    }
    control_isr_in_progress = false;
    return higher_priority_task_woken == pdTRUE;
}

/**
 * @brief 初始化 20 kHz MCPWM FOC 控制定时器
 *
 * @return 初始化成功时返回 true
 */
static bool register_and_enable_control_timer()
{
    if(!control_timer)
    {
        return false;
    }

    mcpwm_timer_event_callbacks_t callbacks = {};
    callbacks.on_empty = control_timer_alarm_callback;
    esp_err_t error = mcpwm_timer_register_event_callbacks(control_timer,
        &callbacks,
        nullptr);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "注册 MCPWM FOC 控制定时器回调失败: %d",
            static_cast<int>(error));
        return false;
    }

    error = mcpwm_timer_enable(control_timer);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "使能 MCPWM FOC 控制定时器失败: %d",
            static_cast<int>(error));
        return false;
    }
    return true;
}

/**
 * @brief 在指定 CPU 上注册并使能 MCPWM 控制定时器中断
 *
 * @param argument 未使用
 *
 * @note MCPWM 驱动会把中断分配到调用注册函数的 CPU。让 CPU1 执行该
 *       操作，可以把 CPU0 留给 ADC1 连续采样和 I2C 控制任务。
 */
static void initialize_control_timer_on_core(void *argument)
{
    (void)argument;
    control_timer_initialize_result = register_and_enable_control_timer() ?
        ESP_OK : ESP_FAIL;
}

/**
 * @brief 初始化 20 kHz MCPWM FOC 控制定时器
 *
 * @return 初始化成功时返回 true
 */
static bool initialize_control_timer()
{
    if(xPortGetCoreID() == CONTROL_ISR_CORE)
    {
        return register_and_enable_control_timer();
    }

    control_timer_initialize_result = ESP_FAIL;
    esp_err_t error = esp_ipc_call_blocking(
        static_cast<uint32_t>(CONTROL_ISR_CORE),
        initialize_control_timer_on_core,
        nullptr);
    if(error != ESP_OK || control_timer_initialize_result != ESP_OK)
    {
        ESP_LOGE(TAG, "在 CPU%d 上初始化 MCPWM 控制定时器失败: ipc=%d, init=%d",
            static_cast<int>(CONTROL_ISR_CORE),
            static_cast<int>(error),
            static_cast<int>(control_timer_initialize_result));
        return false;
    }
    return true;
}

/**
 * @brief 启动仅用于传感器对齐的 MCPWM 载波
 *
 * @return 启动成功时返回 true
 *
 * @note 此时 control_timer_running 保持为 false，MCPWM 回调只会快速返回，
 *       不会访问尚未初始化完成的 FOC 核心。
 */
static bool start_pwm_carrier_for_alignment()
{
    if(!control_timer)
    {
        return false;
    }
    if(control_timer_started)
    {
        return true;
    }

    control_timer_running = false;
    esp_err_t error = mcpwm_timer_start_stop(control_timer,
        MCPWM_TIMER_START_NO_STOP);
    if(error != ESP_OK)
    {
        ESP_LOGE(TAG, "启动传感器对齐 PWM 载波失败: %d",
            static_cast<int>(error));
        return false;
    }
    control_timer_started = true;
    return true;
}

/**
 * @brief 停止传感器对齐期间使用的 MCPWM 载波
 *
 * @return 停止成功时返回 true
 */
static bool stop_pwm_carrier_for_alignment()
{
    control_timer_running = false;
    bool stop_successful = true;
    if(control_timer && control_timer_started)
    {
        esp_err_t error = mcpwm_timer_start_stop(control_timer,
            MCPWM_TIMER_STOP_EMPTY);
        if(error != ESP_OK)
        {
            ESP_LOGE(TAG, "停止传感器对齐 PWM 载波失败: %d",
                static_cast<int>(error));
            stop_successful = false;
        }
        control_timer_started = false;
    }

    while(control_isr_in_progress)
    {
        taskYIELD();
    }
    return stop_successful;
}

/**
 * @brief 启动 20 kHz FOC 控制定时器
 *
 * @return 启动成功时返回 true
 */
static bool start_control_timer()
{
    if(!control_timer || control_timer_started)
    {
        return control_timer_started;
    }

    control_isr_fault = false;
    control_isr_fault_result = static_cast<uint8_t>(foc_result::NOT_READY);
    control_isr_last_result = static_cast<uint8_t>(foc_result::NOT_READY);
    control_isr_tick_count = 0;
    control_isr_last_timestamp_us = 0;
    control_isr_last_cycles = 0;
    control_isr_max_cycles = 0;
    current_isr_last_timestamp_us =
        static_cast<uint32_t>(esp_timer_get_time());
    current_isr_last_raw_sequence = current_raw_sequence;
    current_adc_start_frame_count = current_adc_frame_count;
    current_adc_start_update_count = current_adc_update_count;
    current_adc_start_dma_callback_count = current_adc_dma_callback_count;
    rotor_publish_count = 0;
    rotor_last_published_timestamp_us = 0;
    rotor_last_publish_interval_us = 0;
    rotor_max_publish_interval_us = 0;
    encoder_read_attempt_failure_count = 0;
    encoder_bus_reset_count = 0;
    encoder_last_read_duration_us = 0;
    encoder_max_read_duration_us = 0;
    bus_sense_last_duration_us = 0;
    bus_sense_max_duration_us = 0;
    control_timer_running = true;
    esp_err_t error = mcpwm_timer_start_stop(control_timer,
        MCPWM_TIMER_START_NO_STOP);
    if(error != ESP_OK)
    {
        control_timer_running = false;
        ESP_LOGE(TAG, "启动 MCPWM FOC 控制定时器失败: %d",
            static_cast<int>(error));
        return false;
    }
    control_timer_started = true;
    return true;
}

/**
 * @brief 停止 20 kHz MCPWM FOC 控制定时器并等待当前 ISR 退出
 *
 * @return 停止成功时返回 true
 */
static bool stop_control_timer()
{
    control_timer_running = false;
    bool stop_successful = true;
    if(control_timer && control_timer_started)
    {
        esp_err_t error = mcpwm_timer_start_stop(control_timer,
            MCPWM_TIMER_STOP_EMPTY);
        if(error != ESP_OK)
        {
            stop_successful = false;
            ESP_LOGE(TAG, "停止 MCPWM FOC 控制定时器失败: %d",
                static_cast<int>(error));
        }
        control_timer_started = false;
    }

    while(control_isr_in_progress)
    {
        taskYIELD();
    }
    return stop_successful;
}

/**
 * @brief FOC 硬件测试控制任务
 *
 * @param argument FreeRTOS 任务参数
 *
 * @note 任务每 1 ms 读取 AS5600 并向 Topic 发布转子样本，运行低速机械速度
 *       环得到 q 轴电流目标。ADC1 连续采样任务提供电流原始值，20 kHz
 *       MCPWM ISR 发布电流样本并运行 foc_core::core_loop_from_isr()；
 *       MCPWM ISR 在 CPU1，当前任务在 CPU0。
 */
static void foc_control_task_entry(void *argument)
{
    foc_topic_access &topics = *control_topics;
    TickType_t last_wake_tick = xTaskGetTickCount();
    int64_t task_start_us = esp_timer_get_time();
    bool motor_started = false;
    bool test_finished = false;
    bool start_error_reported = false;
    bool speed_filter_valid = false;
    float filtered_speed_feedback_rad_s = 0.0f;
    uint32_t speed_over_limit_count = 0;
    float commanded_speed_target_rad_s = 0.0f;
    float speed_integral_a = 0.0f;
    float closed_loop_start_angle_rad = 0.0f;
    float closed_loop_min_angle_rad = 0.0f;
    float closed_loop_max_angle_rad = 0.0f;
    float closed_loop_max_speed_rad_s = 0.0f;
    bool closed_loop_motion_stats_valid = false;
    int64_t motor_start_timestamp_us = 0;
    int64_t last_log_us = task_start_us;

    while(true)
    {
        uint32_t next_iteration_count = control_task_iteration_count;
        control_task_iteration_count = next_iteration_count + 1;
        control_task_stage = 1;
        control_task_stage_timestamp_us = static_cast<uint32_t>(
            esp_timer_get_time());
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
        control_task_stage = 2;
        control_task_stage_timestamp_us = static_cast<uint32_t>(
            esp_timer_get_time());

        control_task_stage = 3;
        control_task_stage_timestamp_us = static_cast<uint32_t>(
            esp_timer_get_time());
        int64_t rotor_timestamp_us = esp_timer_get_time();
        rotor_sample rotor{};
        bool rotor_valid = read_rotor_sample(rotor_timestamp_us,
            sequence,
            rotor);

        if(rotor_valid)
        {
            float speed_feedback_rad_s =
                static_cast<float>(MOTOR_ROTOR_DIRECTION) *
                rotor.mechanical_velocity_rad_s;
            if(!speed_filter_valid)
            {
                filtered_speed_feedback_rad_s = speed_feedback_rad_s;
                speed_filter_valid = true;
            }
            else
            {
                filtered_speed_feedback_rad_s +=
                    SPEED_FILTER_ALPHA *
                    (speed_feedback_rad_s -
                        filtered_speed_feedback_rad_s);
            }
            speed_feedback_for_log_rad_s = filtered_speed_feedback_rad_s;
            rotor_read_success_count++;
        }
        else
        {
            rotor_read_failure_count++;
        }
        control_task_stage = 4;
        control_task_stage_timestamp_us = static_cast<uint32_t>(
            esp_timer_get_time());

        if(!control_timer_running)
        {
            read_bus_sense();
        }
        current.timestamp_us = static_cast<uint32_t>(
            esp_timer_get_time());
        if(rotor_valid)
        {
            rotor.timestamp_us = static_cast<uint32_t>(
                esp_timer_get_time());
        }
        if(!control_timer_running)
        {
            topics.current.publish(current);
        }
        bool rotor_published = false;
        if(rotor_valid)
        {
            control_task_stage = 5;
            control_task_stage_timestamp_us = static_cast<uint32_t>(
                esp_timer_get_time());
            rotor_published = topics.rotor.publish(rotor);
            control_task_stage = 6;
            control_task_stage_timestamp_us = static_cast<uint32_t>(
                esp_timer_get_time());
            if(rotor_published)
            {
                uint32_t publish_timestamp_us = static_cast<uint32_t>(
                    esp_timer_get_time());
                if(rotor_publish_count > 0)
                {
                    rotor_last_publish_interval_us =
                        publish_timestamp_us -
                        rotor_last_published_timestamp_us;
                    if(rotor_last_publish_interval_us >
                        rotor_max_publish_interval_us)
                    {
                        rotor_max_publish_interval_us =
                            rotor_last_publish_interval_us;
                    }
                }
                rotor_last_published_timestamp_us = publish_timestamp_us;
                rotor_publish_count++;
            }
        }

        int64_t now_us = esp_timer_get_time();
        if(!motor_started && !test_finished &&
            now_us - task_start_us >=
            static_cast<int64_t>(AUTO_START_DELAY_MS) * 1000)
        {
            if(current_valid && rotor_valid)
            {
                control_task_stage = 7;
                control_task_stage_timestamp_us = static_cast<uint32_t>(
                    esp_timer_get_time());
                foc_result result = motor.enable();
                control_task_stage = 8;
                control_task_stage_timestamp_us = static_cast<uint32_t>(
                    esp_timer_get_time());
                if(result == foc_result::OK)
                {
                    ESP_LOGI(TAG,
                        "准备进入 AS5600 速度闭环: FOC=%d Hz, target=%.2f rad/s, "
                        "q_limit=%.3f A, kick=%.3f A/%d ms",
                        static_cast<int>(FOC_CONTROL_FREQUENCY_HZ),
                        CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S,
                        CLOSED_LOOP_MAX_Q_CURRENT_A,
                        STARTUP_KICK_Q_CURRENT_A,
                        static_cast<int>(STARTUP_KICK_DURATION_MS));
                    if(start_control_timer())
                    {
                        control_task_stage = 9;
                        control_task_stage_timestamp_us = static_cast<uint32_t>(
                            esp_timer_get_time());
                        motor_started = true;
                        commanded_speed_target_rad_s = 0.0f;
                        speed_integral_a = 0.0f;
                        closed_loop_start_angle_rad =
                            rotor.mechanical_angle_rad;
                        closed_loop_min_angle_rad =
                            rotor.mechanical_angle_rad;
                        closed_loop_max_angle_rad =
                            rotor.mechanical_angle_rad;
                        closed_loop_max_speed_rad_s = 0.0f;
                        closed_loop_motion_stats_valid = true;
                        motor_start_timestamp_us = now_us;
                    }
                    else
                    {
                        motor.disable();
                        test_finished = true;
                        ESP_LOGE(TAG, "FOC 控制定时器启动失败，已停机");
                    }
                }
                else if(!start_error_reported)
                {
                    ESP_LOGE(TAG, "开启 FOC 失败: %d",
                        static_cast<int>(result));
                    start_error_reported = true;
                }
            }
        }

        foc_result loop_result = static_cast<foc_result>(
            control_isr_last_result);
        if(motor_started)
        {
            if(rotor_valid && closed_loop_motion_stats_valid)
            {
                if(rotor.mechanical_angle_rad < closed_loop_min_angle_rad)
                {
                    closed_loop_min_angle_rad = rotor.mechanical_angle_rad;
                }
                if(rotor.mechanical_angle_rad > closed_loop_max_angle_rad)
                {
                    closed_loop_max_angle_rad = rotor.mechanical_angle_rad;
                }
                float absolute_speed_rad_s = absolute_value(
                    rotor.mechanical_velocity_rad_s);
                if(absolute_speed_rad_s > closed_loop_max_speed_rad_s)
                {
                    closed_loop_max_speed_rad_s = absolute_speed_rad_s;
                }
            }
            if(speed_filter_valid &&
                absolute_value(filtered_speed_feedback_rad_s) >
                    MOTOR_MAX_MECHANICAL_SPEED_RAD_S)
            {
                if(speed_over_limit_count <
                    SPEED_OVERSPEED_CONFIRMATION_COUNT)
                {
                    speed_over_limit_count++;
                }
            }
            else
            {
                speed_over_limit_count = 0;
            }
            if(control_isr_fault)
            {
                uint8_t fault_result = control_isr_fault_result;
                uint32_t isr_ticks = control_isr_tick_count;
                uint32_t diagnostic_now_us = static_cast<uint32_t>(
                    esp_timer_get_time());
                stop_control_timer();
                motor.disable();
                motor_started = false;
                test_finished = true;
                ESP_LOGE(TAG,
                    "20 kHz FOC ISR 失败，已停机: result=%d, ticks=%lu, "
                    "rotor_ok=%lu, rotor_fail=%lu, rotor_age=%lu us, "
                    "rotor_pub=%lu interval=%lu max_interval=%lu us, "
                    "current_age=%lu us, adc_seq=%lu/%lu/%lu, adc_frames=%lu, "
                    "adc_errors=%lu, adc_channels=%lu/%lu/%lu/%lu, "
                    "adc_commit=%lu/%lu age=%lu us, "
                    "adc_update=%lu/%lu stage=%lu, "
                    "adc_dma=%lu +%lu age=%lu us core=%u, "
                    "adc_delta=%lu/%lu/%lu, isr_core=%u, "
                    "snapshot_retry=%lu, "
                    "rotor=%u/%u age=%lu us, "
                    "task=%lu stage=%lu age=%lu us, "
                    "encoder_fail=%lu reset=%lu read=%lu/%lu us, "
                    "bus_read=%lu/%lu us, "
                    "adc_frame=%lu, isr_cycles=%lu/%lu",
                    static_cast<int>(fault_result),
                    static_cast<unsigned long>(isr_ticks),
                    static_cast<unsigned long>(rotor_read_success_count),
                    static_cast<unsigned long>(rotor_read_failure_count),
                    static_cast<unsigned long>(
                        diagnostic_now_us -
                            rotor_last_published_timestamp_us),
                    static_cast<unsigned long>(rotor_publish_count),
                    static_cast<unsigned long>(rotor_last_publish_interval_us),
                    static_cast<unsigned long>(rotor_max_publish_interval_us),
                    static_cast<unsigned long>(
                        control_isr_fault_current_age_us),
                    static_cast<unsigned long>(
                        control_isr_fault_raw_sequence),
                    static_cast<unsigned long>(
                        control_isr_fault_locked_sequence),
                    static_cast<unsigned long>(
                        control_isr_fault_unlocked_sequence),
                    static_cast<unsigned long>(current_adc_frame_count),
                    static_cast<unsigned long>(current_adc_read_error_count),
                    static_cast<unsigned long>(
                        control_isr_fault_sample_count),
                    static_cast<unsigned long>(control_isr_fault_a_count),
                    static_cast<unsigned long>(control_isr_fault_b_count),
                    static_cast<unsigned long>(
                        control_isr_fault_invalid_count),
                    static_cast<unsigned long>(
                        control_isr_fault_commit_count),
                    static_cast<unsigned long>(
                        control_isr_fault_commit_sequence),
                    static_cast<unsigned long>(
                        control_isr_fault_commit_age_us),
                    static_cast<unsigned long>(
                        control_isr_fault_update_count),
                    static_cast<unsigned long>(
                        control_isr_fault_no_pair_count),
                    static_cast<unsigned long>(
                        control_isr_fault_update_stage),
                    static_cast<unsigned long>(
                        control_isr_fault_dma_callback_count),
                    static_cast<unsigned long>(
                        control_isr_fault_dma_delta),
                    static_cast<unsigned long>(
                        control_isr_fault_dma_age_us),
                    static_cast<unsigned int>(
                        control_isr_fault_dma_core_id),
                    static_cast<unsigned long>(
                        control_isr_fault_frame_delta),
                    static_cast<unsigned long>(
                        control_isr_fault_update_delta),
                    static_cast<unsigned long>(
                        control_isr_fault_dma_delta),
                    static_cast<unsigned int>(control_isr_core_id),
                    static_cast<unsigned long>(
                        control_isr_fault_snapshot_retry_count),
                    static_cast<unsigned int>(
                        control_isr_fault_rotor_peek_ok),
                    static_cast<unsigned int>(
                        control_isr_fault_rotor_valid),
                    static_cast<unsigned long>(
                        control_isr_fault_rotor_age_us),
                    static_cast<unsigned long>(
                        control_isr_fault_task_iteration_count),
                    static_cast<unsigned long>(control_isr_fault_task_stage),
                    static_cast<unsigned long>(
                        control_isr_fault_task_stage_age_us),
                    static_cast<unsigned long>(
                        encoder_read_attempt_failure_count),
                    static_cast<unsigned long>(encoder_bus_reset_count),
                    static_cast<unsigned long>(encoder_last_read_duration_us),
                    static_cast<unsigned long>(encoder_max_read_duration_us),
                    static_cast<unsigned long>(bus_sense_last_duration_us),
                    static_cast<unsigned long>(bus_sense_max_duration_us),
                    static_cast<unsigned long>(current_adc_last_frame_size),
                    static_cast<unsigned long>(control_isr_last_cycles),
                    static_cast<unsigned long>(control_isr_max_cycles));
                ESP_LOGE(TAG,
                    "控制任务诊断: iteration=%lu stage=%lu stage_age=%lu us",
                    static_cast<unsigned long>(control_task_iteration_count),
                    static_cast<unsigned long>(control_task_stage),
                    static_cast<unsigned long>(
                        diagnostic_now_us - control_task_stage_timestamp_us));
            }
            else if(rotor_valid && !rotor_published)
            {
                stop_control_timer();
                motor.disable();
                motor_started = false;
                test_finished = true;
                ESP_LOGE(TAG, "发布转子样本失败，已停机");
            }
            else if(speed_over_limit_count >=
                SPEED_OVERSPEED_CONFIRMATION_COUNT)
            {
                stop_control_timer();
                motor.disable();
                motor_started = false;
                test_finished = true;
                ESP_LOGE(TAG,
                    "机械速度超限，已停机: speed=%.3f rad/s",
                    filtered_speed_feedback_rad_s);
            }
            else
            {
                if(now_us - motor_start_timestamp_us >=
                    static_cast<int64_t>(CONTROL_TARGET_HANDOFF_DELAY_MS) *
                        1000)
                {
                    float speed_target_step_rad_s =
                        CLOSED_LOOP_SPEED_RAMP_RAD_S2 *
                        SPEED_CONTROL_PERIOD_S;
                    if(commanded_speed_target_rad_s <
                        CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S)
                    {
                        commanded_speed_target_rad_s +=
                            speed_target_step_rad_s;
                        if(commanded_speed_target_rad_s >
                            CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S)
                        {
                            commanded_speed_target_rad_s =
                                CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S;
                        }
                    }
                    else if(commanded_speed_target_rad_s >
                        CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S)
                    {
                        commanded_speed_target_rad_s -=
                            speed_target_step_rad_s;
                        if(commanded_speed_target_rad_s <
                            CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S)
                        {
                            commanded_speed_target_rad_s =
                                CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S;
                        }
                    }

                    bool startup_kick_active = now_us -
                        motor_start_timestamp_us <
                        static_cast<int64_t>(STARTUP_KICK_DURATION_MS) * 1000;
                    float target_q_current_a = startup_kick_active ?
                        STARTUP_KICK_Q_CURRENT_A :
                        calculate_speed_q_current_target(
                                commanded_speed_target_rad_s,
                                filtered_speed_feedback_rad_s,
                                speed_integral_a);
                    foc_target closed_loop_target{};
                    closed_loop_target.timestamp_us =
                        static_cast<uint32_t>(now_us);
                    closed_loop_target.mode = foc_control_mode::CURRENT;
                    closed_loop_target.d_axis_current_a = 0.0f;
                    closed_loop_target.q_axis_current_a = target_q_current_a;
                    control_task_stage = 10;
                    control_task_stage_timestamp_us = static_cast<uint32_t>(
                        esp_timer_get_time());
                    foc_result target_result = motor.set_target(
                        closed_loop_target);
                    control_task_stage = 11;
                    control_task_stage_timestamp_us = static_cast<uint32_t>(
                        esp_timer_get_time());
                    if(target_result != foc_result::OK)
                    {
                        stop_control_timer();
                        motor.disable();
                        motor_started = false;
                        test_finished = true;
                        ESP_LOGE(TAG, "更新速度闭环目标失败，已停机: %d",
                            static_cast<int>(target_result));
                    }
                }
            }
        }

        if(motor_started && now_us - task_start_us >=
            static_cast<int64_t>(TEST_RUN_DURATION_MS) * 1000)
        {
            stop_control_timer();
            motor.disable();
            motor_started = false;
            test_finished = true;
            float closed_loop_angle_span_rad =
                closed_loop_motion_stats_valid ?
                closed_loop_max_angle_rad - closed_loop_min_angle_rad : 0.0f;
            float closed_loop_angle_delta_rad =
                closed_loop_motion_stats_valid ?
                rotor.mechanical_angle_rad - closed_loop_start_angle_rad :
                0.0f;
            ESP_LOGI(TAG,
                "硬件测试时长到达，已自动停机: ticks=%lu, "
                "isr_cycles=%lu/%lu, angle_delta=%.3f rad, "
                "angle_span=%.3f rad, max_speed=%.3f rad/s",
                static_cast<unsigned long>(control_isr_tick_count),
                static_cast<unsigned long>(control_isr_last_cycles),
                static_cast<unsigned long>(control_isr_max_cycles),
                closed_loop_angle_delta_rad,
                closed_loop_angle_span_rad,
                closed_loop_max_speed_rad_s);
        }

        if(!control_timer_running && now_us - last_log_us >=
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
                    "Id=%.3f Iq=%.3f Iq_target=%.3f speed_target=%.3f "
                    "speed_feedback=%.3f encoder_speed=%.3f "
                    "encoder_angle=%.3f electrical_angle=%.3f "
                    "fault=0x%lx bus_pin=%d mV",
                    static_cast<int>(snapshot.state),
                    static_cast<int>(loop_result),
                    snapshot.phase_a_current_a,
                    snapshot.phase_b_current_a,
                    snapshot.phase_c_current_a,
                    snapshot.i_d_a,
                    snapshot.i_q_a,
                    snapshot.target_i_q_a,
                    commanded_speed_target_rad_s,
                    filtered_speed_feedback_rad_s,
                    rotor.mechanical_velocity_rad_s,
                    rotor.mechanical_angle_rad,
                    snapshot.electrical_angle_rad,
                    static_cast<unsigned long>(snapshot.fault_flags),
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
        control_task_stage = 12;
        control_task_stage_timestamp_us = static_cast<uint32_t>(
            esp_timer_get_time());
    }
}

/**
 * @brief 输出 FOC 运行状态的低优先级诊断任务
 *
 * @param argument FreeRTOS 任务参数
 *
 * @note 该任务只在任务上下文读取快照并打印，不参与控制计算。将串口
 *       输出与控制任务分离，避免日志阻塞 AS5600 采样和 ADC 读取。
 */
static void foc_status_logger_task_entry(void *argument)
{
    (void)argument;
    bool logger_reference_valid = false;
    float logger_reference_angle_rad = 0.0f;
    uint32_t logger_reference_timestamp_us = 0;
    while(true)
    {
        if(control_timer_running && control_topics && !control_isr_fault)
        {
            foc_snapshot snapshot{};
            rotor_sample rotor{};
            bool snapshot_valid = control_topics->snapshot.peek(snapshot, 0);
            bool rotor_valid = control_topics->rotor.peek(rotor, 0);
            if(snapshot_valid && rotor_valid)
            {
                uint32_t now_us = static_cast<uint32_t>(
                    esp_timer_get_time());
                float window_speed_rad_s =
                    rotor.mechanical_velocity_rad_s;
                if(logger_reference_valid)
                {
                    uint32_t window_time_us = rotor.timestamp_us -
                        logger_reference_timestamp_us;
                    if(window_time_us > 0 &&
                        window_time_us < SPEED_LOG_WINDOW_MAX_US)
                    {
                        window_speed_rad_s =
                            (rotor.mechanical_angle_rad -
                                logger_reference_angle_rad) /
                            (static_cast<float>(window_time_us) * 1.0e-6f);
                    }
                }
                logger_reference_angle_rad = rotor.mechanical_angle_rad;
                logger_reference_timestamp_us = rotor.timestamp_us;
                logger_reference_valid = true;
                float window_speed_rpm = window_speed_rad_s *
                    RAD_PER_SECOND_TO_RPM;
                uint32_t rotor_age_us = now_us - rotor.timestamp_us;
                ESP_LOGI(TAG,
                    "运行状态: Iq=%.3f/%.3f A, speed_raw=%.3f rad/s, "
                    "speed_window=%.3f rad/s(%.1f rpm), "
                    "speed_feedback=%.3f rad/s, angle=%.3f rad, "
                    "electrical=%.3f rad, rotor_seq=%lu age=%lu us, "
                    "enc_reject=%lu, isr=%lu/%lu",
                    snapshot.i_q_a,
                    snapshot.target_i_q_a,
                    rotor.mechanical_velocity_rad_s,
                    window_speed_rad_s,
                    window_speed_rpm,
                    speed_feedback_for_log_rad_s,
                    rotor.mechanical_angle_rad,
                    snapshot.electrical_angle_rad,
                    static_cast<unsigned long>(rotor.sequence),
                    static_cast<unsigned long>(rotor_age_us),
                    static_cast<unsigned long>(
                        encoder_rejected_sample_count.load(
                            std::memory_order_relaxed)),
                    static_cast<unsigned long>(control_isr_last_cycles),
                    static_cast<unsigned long>(control_isr_max_cycles));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
    }
}

/* ---- esp32_hardware_test 公共 API ---- */

/**
 * @brief 初始化 ESP32 V3P 单电机 FOC 硬件测试
 *
 * @note 该测试会先保持 EN 低电平，校准电流零点，使用 0.4 V 固定电压矢量
 *       对齐 AS5600，然后延时 1 s 自动进入低速 AS5600 速度闭环。目标机械
 *       速度为 5 rad/s，正常 q 轴电流限制为 0.05 A，启动阶段使用 0.05 A
 *       助推 100 ms，运行 60 s 后自动停机。电流环由 20 kHz MCPWM ISR
 *       驱动，AS5600 仍由任务读取。没有硬件故障输入，保护依靠电流采样、
 *       机械速度检查和 foc_core 软件限幅。
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

    encoder_result = encoder_bus.reset();
    if(encoder_result != i2c_result::OK)
    {
        ESP_LOGW(TAG, "AS5600 I2C 总线初始复位失败: %d",
            static_cast<int>(encoder_result));
    }

    if(!initialize_control_timer())
    {
        disable_power_stage();
        ESP_LOGE(TAG, "20 kHz FOC 控制定时器初始化失败，保持驱动器关闭");
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
    config.pole_pairs = MOTOR_POLE_PAIRS;
    config.rotor_direction = MOTOR_ROTOR_DIRECTION;
    config.electrical_zero_offset_rad = electrical_zero_offset_rad;
    config.control_period_s = FOC_CONTROL_PERIOD_S;
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

    static foc_topic_access topic_access = motor.topics();
    control_topics = &topic_access;
    foc_topic_access &topics = *control_topics;
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
    target.q_axis_current_a = 0.0f;
    result = motor.set_target(target);
    if(result != foc_result::OK)
    {
        motor.disable();
        ESP_LOGE(TAG, "设置初始电流目标失败: %d", static_cast<int>(result));
        return;
    }

    ESP_LOGI(TAG,
        "硬件测试就绪: PWM=%d Hz, FOC=%d Hz, I2C=%d Hz, bus=%.1f V, "
        "shunt=%.3f ohm, gain=%.1f, "
        "current_scale=%.4f A/mV, closed_loop_speed=%.2f rad/s, "
        "q_limit=%.3f A, auto_start=%d",
        static_cast<int>(PWM_FREQUENCY_HZ),
        static_cast<int>(FOC_CONTROL_FREQUENCY_HZ),
        static_cast<int>(I2C_SPEED_HZ),
        MOTOR_BUS_VOLTAGE_V,
        CURRENT_SHUNT_RESISTOR_OHM,
        CURRENT_AMPLIFIER_GAIN,
        CURRENT_SCALE_A_PER_MV,
        CLOSED_LOOP_TARGET_MECHANICAL_SPEED_RAD_S,
        CLOSED_LOOP_MAX_Q_CURRENT_A,
        1);

    BaseType_t task_result = xTaskCreatePinnedToCore(
        foc_control_task_entry,
        "foc_control",
        4096,
        nullptr,
        CONTROL_TASK_PRIORITY,
        nullptr,
        CONTROL_TASK_CORE);
    if(task_result != pdPASS)
    {
        motor.disable();
        ESP_LOGE(TAG, "创建 FOC 控制任务失败");
        return;
    }

    task_result = xTaskCreatePinnedToCore(
        foc_status_logger_task_entry,
        "foc_status",
        3072,
        nullptr,
        tskIDLE_PRIORITY + 1,
        nullptr,
        CONTROL_TASK_CORE);
    if(task_result != pdPASS)
    {
        ESP_LOGW(TAG, "创建 FOC 状态诊断任务失败");
    }

    module_initialized = true;
}
