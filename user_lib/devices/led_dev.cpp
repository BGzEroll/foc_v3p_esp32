#include "led_dev.h"

#include "drivers/leds.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

static constexpr uint32_t GREEN_LED_ON_TIME_MS = 100;
static constexpr uint32_t GREEN_LED_OFF_TIME_MS = 900;
static constexpr uint16_t LED_TASK_STACK_DEPTH = 128;
static constexpr UBaseType_t LED_TASK_PRIORITY = tskIDLE_PRIORITY + 1U;

static leds green_led(ONBORAD_GREEN_LED_GPIO_Port,
    ONBORAD_GREEN_LED_Pin,
    GPIO_PIN_RESET);

/**
 * @brief 绿色 LED 周期闪烁任务
 *
 * @param argument FreeRTOS 任务参数
 */
static void green_led_task_entry(void *argument)
{
    while(true)
    {
        green_led.on();
        vTaskDelay(pdMS_TO_TICKS(GREEN_LED_ON_TIME_MS));

        green_led.off();
        vTaskDelay(pdMS_TO_TICKS(GREEN_LED_OFF_TIME_MS));
    }
}

/**
 * @brief 创建绿色 LED 周期闪烁任务
 */
void led_dev::init()
{
    BaseType_t result = xTaskCreate(green_led_task_entry,
        "green_led",
        LED_TASK_STACK_DEPTH,
        nullptr,
        LED_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        Error_Handler();
    }
}
