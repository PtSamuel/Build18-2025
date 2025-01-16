
#include "lcd.h"
#include "sd_card.h"
#include "gps.h"
#include "imu.h"
#include "esp_log.h"
#include "stdlib.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "lvgl.h"

static const char* TAG = "main";

void app_main(void)
{
    lcd_init();
    sd_card_init();
    gps_init();
    imu_init();
    
    while(1)
    {
        // imu_read();
        // vTaskDelay(pdMS_TO_TICKS(500));

        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
    }
}
