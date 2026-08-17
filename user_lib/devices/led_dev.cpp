#include "led_dev.h"

#include "drivers/leds.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr uint32_t GREEN_LED_ON_TIME_MS = 100;
static constexpr uint32_t GREEN_LED_OFF_TIME_MS = 900;
static constexpr uint32_t LED_TASK_STACK_SIZE = 2048;
static constexpr UBaseType_t LED_TASK_PRIORITY = tskIDLE_PRIORITY + 1U;
static constexpr gpio_num_t GREEN_LED_GPIO = GPIO_NUM_2;
static constexpr uint32_t GREEN_LED_ON_LEVEL = 0U;

static leds green_led(GREEN_LED_GPIO, GREEN_LED_ON_LEVEL);

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
    ESP_ERROR_CHECK(green_led.init());

    BaseType_t result = xTaskCreate(green_led_task_entry,
        "green_led",
        LED_TASK_STACK_SIZE,
        nullptr,
        LED_TASK_PRIORITY,
        nullptr);

    if(result != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
