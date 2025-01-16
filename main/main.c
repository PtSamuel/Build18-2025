#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "lcd.h"
#include "sd_card.h"
#include "gps.h"
#include "esp_log.h"
#include "stdlib.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "lvgl.h"

#include <math.h>

static const char* TAG = "main";

// static void cs_high(spi_transaction_t* t)
// {
//     ESP_EARLY_LOGV(TAG, "cs high %d.", GPIO_NUM_25);
//     gpio_set_level(GPIO_NUM_25, 1);
// }

// static void cs_low(spi_transaction_t* t)
// {
//     gpio_set_level(GPIO_NUM_25, 0);
//     ESP_EARLY_LOGV(TAG, "cs low %d.", GPIO_NUM_25);
// }

// 主函数
void app_main(void)
{
    lcd_init();
    sd_card_init();
    gps_init();

    // spi_device_handle_t spi;
    // spi_bus_config_t buscfg = {
    //     .miso_io_num = GPIO_NUM_22,
    //     .mosi_io_num = GPIO_NUM_23,
    //     .sclk_io_num = GPIO_NUM_27,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    // };
    // spi_device_interface_config_t devcfg = {
    //     .command_bits = 0,
    //     .clock_speed_hz = 1 * 1000 * 1000,     // Clock out at 1 MHz
    //     .mode = 0,                              // SPI mode 0
    //     .spics_io_num = GPIO_NUM_25,             // CS pin
    //     .queue_size = 1,                
    // };
    // //Initialize the SPI bus
    // ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    // ESP_ERROR_CHECK(ret);
    // //Attach the LCD to the SPI bus
    // ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    // ESP_ERROR_CHECK(ret);


    // spi_transaction_t t = {
    //     .length = 8,
    //     .tx_data = {0x80 | 0x19},
    //     .rxlength = 8,
    //     .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
    // };

    // esp_err_t err;
    // err = spi_device_acquire_bus(spi, portMAX_DELAY);
    // if(err != ESP_OK) {
    //     ESP_LOGE(TAG, "Failure acquiring bus", esp_err_to_name(err));
    // }

    while(1)
    {
        // err = spi_device_polling_transmit(spi, &t);
        // err = spi_device_transmit(spi, &t);
        // if (err != ESP_OK) {
        //     ESP_LOGE(TAG, "Failure reading register: %s", esp_err_to_name(err));
        // } else {
        //     ESP_LOGI(TAG, "Register value: (%d, %d, %d, %d)", t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
        // }
        
        // vTaskDelay(pdMS_TO_TICKS(500));

        vTaskDelay(pdMS_TO_TICKS(10));
        lv_task_handler();
        // st7789_flush(50, 210, 0, 160, color_buf);
        // draw(0, 20, 0, 100, 0x8000);


        // const int pixel_num = 20 * 100;
        // uint8_t *color_buf = heap_caps_malloc(pixel_num * 2, MALLOC_CAP_DMA);
        // for(int i = 0; i < pixel_num; i++) {
        //     color_buf[2 * i] = 0xFF;
        //     color_buf[2 * i + 1] = 0xFF;
        // }
        // st7789_flush(100, 120, 0, 100, color_buf);
        // free(color_buf);
    }
}
