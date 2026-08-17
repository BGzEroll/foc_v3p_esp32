#include "mpu6050_dev.h"

#include "main.h"
#include "system/topic.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint8_t MPU6050_I2C_BUS_ID = 1U;
static constexpr uint8_t MPU6050_I2C_ADDRESS = 0x68U;
static constexpr float MPU6050_ACCELEROMETER_WEIGHT = 0.02f;
static constexpr uint16_t MPU6050_TASK_STACK_DEPTH = 512U;
static constexpr UBaseType_t MPU6050_TASK_PRIORITY = tskIDLE_PRIORITY + 3U;
static constexpr uint32_t MPU6050_UPDATE_PERIOD_MS = 5U;
static constexpr uint32_t MPU6050_RETRY_DELAY_MS = 1000U;

static mpu6050 imu(MPU6050_I2C_BUS_ID,
    MPU6050_I2C_ADDRESS,
    MPU6050_ACCELEROMETER_WEIGHT);
static topic::latest_topic<mpu6050_sample> sample_topic;

/**
 * @brief 初始化 MPU6050 并周期发布最新样本
 *
 * @param argument FreeRTOS 任务参数
 */
static void mpu6050_task_entry(void *argument)
{
    i2c_result init_result;
    do
    {
        init_result = imu.init(true);

        if(init_result != i2c_result::OK)
        {
            vTaskDelay(pdMS_TO_TICKS(MPU6050_RETRY_DELAY_MS));
        }
    }
    while(init_result != i2c_result::OK);

    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        if(imu.update() == i2c_result::OK &&
            !sample_topic.publish(imu.sample()))
        {
            Error_Handler();
        }

        vTaskDelayUntil(&last_wake_time,
            pdMS_TO_TICKS(MPU6050_UPDATE_PERIOD_MS));
    }
}

/**
 * @brief 读取最新 MPU6050 样本但不移除话题数据
 *
 * @param sample 用于接收快照的对象
 *
 * @return 已存在样本时返回 true
 */
bool mpu6050_dev::peek_latest(mpu6050_sample &sample)
{
    return sample_topic.peek(sample);
}

/**
 * @brief 初始化 MPU6050 设备、话题并创建样本生产任务
 */
void mpu6050_dev::init()
{
    if(!sample_topic.init())
    {
        Error_Handler();
    }

    BaseType_t result = xTaskCreate(mpu6050_task_entry,
        "mpu6050",
        MPU6050_TASK_STACK_DEPTH,
        nullptr,
        MPU6050_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}
