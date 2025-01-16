#include "lcd.h"

#include "lvgl.h"
#include "lv_port.h"
#include "lv_demos.h"
#include "st7789_driver.h"

#include "stdlib.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LCD_WIDTH   280
#define LCD_HEIGHT  240

static const char* TAG = "LCD";

extern lv_disp_drv_t disp_drv;

static void flush_complete(void* param)
{
    lv_disp_flush_ready(&disp_drv);
}

void draw(int x1, int x2, int y1, int y2, uint16_t color) {
    const int pixel_num = (x2 - x1) * (y2 - y1);
    uint16_t *color_buf = heap_caps_malloc(pixel_num * sizeof(uint16_t), MALLOC_CAP_DMA);
    for(int i = 0; i < pixel_num; i++) {
        color_buf[i] = color;
    }
    st7789_flush(x1, x2, y1, y2, color_buf);
    // free(color_buf);
}


void lcd_init() {
    lv_port_init();

    st7789_cfg_t st7789_config;
    st7789_config.mosi = GPIO_NUM_19;
    st7789_config.clk = GPIO_NUM_18;
    st7789_config.cs = GPIO_NUM_5;
    st7789_config.dc = GPIO_NUM_17;
    st7789_config.rst = GPIO_NUM_21;
    st7789_config.bl = GPIO_NUM_26;
    st7789_config.spi_fre = 40*1000*1000;       //SPI时钟频率
    st7789_config.width = LCD_WIDTH;            //屏宽
    st7789_config.height = LCD_HEIGHT;          //屏高
    st7789_config.spin = 1;                     //顺时针旋转90度
    st7789_config.done_cb = flush_complete;    //数据写入完成回调函数
    st7789_config.cb_param = NULL;         //回调函数参数
    st7789_driver_hw_init(&st7789_config);

    st7789_lcd_backlight(true);         //打开背光

    draw(0, 10, 0, 100, 0x8000);
    draw(10, 20, 0, 100, 0x4000);
    draw(20, 30, 0, 100, 0x2000);
    draw(30, 40, 0, 100, 0x1000);
    draw(40, 50, 0, 100, 0x0800);
    draw(50, 60, 0, 100, 0x0400);
    draw(60, 70, 0, 100, 0x0200);
    draw(70, 80, 0, 100, 0x0100);
    draw(80, 90, 0, 100, 0x0080);
    draw(90, 100, 0, 100, 0x0040);
    draw(100, 110, 0, 100, 0x0020);
    draw(110, 120, 0, 100, 0x0010);
    draw(120, 130, 0, 100, 0x0008);
    draw(130, 140, 0, 100, 0x0004);
    draw(140, 150, 0, 100, 0x0002);
    draw(150, 160, 0, 100, 0x0001);

    ESP_LOGI(TAG, "Flush complete.");

    lv_demo_widgets();
}
