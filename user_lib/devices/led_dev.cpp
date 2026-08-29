#include "led_dev.h"

#include "drivers/leds.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static leds onboard_led(GPIO_NUM_22, 0);

/**
 * @brief 板载 LED 周期闪烁任务
 *
 * @param argument FreeRTOS 任务参数
 */
static void onboard_led_task_entry(void *argument)
{
    while(true)
    {
        onboard_led.on();
        vTaskDelay(pdMS_TO_TICKS(50));

        onboard_led.off();
        vTaskDelay(pdMS_TO_TICKS(950));
    }
}

/**
 * @brief 创建 LED 周期闪烁任务
 */
void led_dev::init()
{
    ESP_ERROR_CHECK(onboard_led.init());

    BaseType_t result = xTaskCreate(onboard_led_task_entry,
        "onboard_led",
        2048,
        nullptr,
        tskIDLE_PRIORITY + 1,
        nullptr);

    if(result != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
