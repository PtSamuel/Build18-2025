
#include "lcd.h"
#include "esp_log.h"
#include "stdlib.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "lvgl.h"

static const char* TAG = "main";

void app_main(void)
{
    lcd_init();
    
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
    }
}
