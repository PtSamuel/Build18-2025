#include "lcd.h"

#include "lv_demos.h"
#include "lv_port.h"
#include "lvgl.h"
#include "st7789_driver.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "stdlib.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LCD_WIDTH 280
#define LCD_HEIGHT 240

static const char *TAG = "LCD";

extern lv_disp_drv_t disp_drv;

// Callback after flushing
static void flush_complete(void *param) {
  ESP_LOGD(TAG, "Flush complete callback called.");
  lv_disp_flush_ready(&disp_drv);
}

// Function to create and populate the interface
void create_interface() {
  ESP_LOGI(TAG, "Creating interface...");

  // Create Status label
  lv_obj_t *label_status = lv_label_create(lv_scr_act());
  lv_label_set_text(label_status, "Status: Normal");
  lv_obj_align(label_status, LV_ALIGN_TOP_LEFT, 10, 10);

  // Create Description label
  lv_obj_t *label_desc = lv_label_create(lv_scr_act());
  lv_label_set_text(label_desc, "Desc: some string");
  lv_obj_align_to(label_desc, label_status, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

  // Create Time label
  lv_obj_t *label_time = lv_label_create(lv_scr_act());
  lv_label_set_text(label_time, "Time: 00:00:00 GTC");
  lv_obj_align_to(label_time, label_desc, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);

  // Create Longitude label
  lv_obj_t *label_longitude = lv_label_create(lv_scr_act());
  lv_label_set_text(label_longitude, "Longitude: 000.00");
  lv_obj_align_to(label_longitude, label_time, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

  // Create Latitude label
  lv_obj_t *label_latitude = lv_label_create(lv_scr_act());
  lv_label_set_text(label_latitude, "Latitude: 00.00");
  lv_obj_align_to(label_latitude, label_longitude, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                  5);

  // Create Velocity label
  lv_obj_t *label_velocity = lv_label_create(lv_scr_act());
  lv_label_set_text(label_velocity, "Velocity: 00.00");
  lv_obj_align_to(label_velocity, label_latitude, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                  5);

  // Create Altitude label
  lv_obj_t *label_altitude = lv_label_create(lv_scr_act());
  lv_label_set_text(label_altitude, "Altitude: 00.00");
  lv_obj_align_to(label_altitude, label_velocity, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                  5);

  ESP_LOGI(TAG, "Interface created.");
}

// Initialize the LCD with debug statements and the new interface
void lcd_init() {
  ESP_LOGI(TAG, "Starting LCD initialization...");
  lv_port_init();

  st7789_cfg_t st7789_config;
  st7789_config.mosi = GPIO_NUM_11;
  st7789_config.clk = GPIO_NUM_12;
  st7789_config.cs = GPIO_NUM_15;
  st7789_config.dc = GPIO_NUM_2;
  st7789_config.rst = GPIO_NUM_4;
  st7789_config.bl = GPIO_NUM_20;
  st7789_config.spi_fre = 40 * 1000 * 1000; // SPI clock frequency
  st7789_config.width = LCD_WIDTH;          // Screen width
  st7789_config.height = LCD_HEIGHT;        // Screen height
  st7789_config.spin = 1;                   // Clockwise rotation
  st7789_config.done_cb = flush_complete;   // Flush callback
  st7789_config.cb_param = NULL;

  ESP_LOGI(TAG, "Configuring ST7789 driver...");
  st7789_driver_hw_init(&st7789_config);
  ESP_LOGI(TAG, "ST7789 driver initialized.");

  st7789_lcd_backlight(true);
  ESP_LOGI(TAG, "Backlight turned on.");

  // Create the interface
  create_interface();

  ESP_LOGI(TAG, "LCD initialization complete.");
}
