#include "lcd.h"

#include "lvgl.h"
#include "lv_port.h"
#include "lv_demos.h"
#include "st7789_driver.h"

#include "stdlib.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "LCD";
static const uint32_t lcd_update_period = 33; // 30 Hz.

void draw(int x1, int x2, int y1, int y2, uint16_t color) {
    const int pixel_num = (x2 - x1) * (y2 - y1);
    uint16_t *color_buf = heap_caps_malloc(pixel_num * sizeof(uint16_t), MALLOC_CAP_DMA);
    for(int i = 0; i < pixel_num; i++) {
        color_buf[i] = color;
    }
    st7789_flush(x1, x2, y1, y2, color_buf);
    // free(color_buf);
}

typedef struct {
    lv_style_t style_text;
    lv_obj_t* label_status;
    lv_obj_t* label_desc;
    lv_obj_t* label_time;
    lv_obj_t* label_latitude;
    lv_obj_t* label_longitude;
    lv_obj_t* label_velocity;
    lv_obj_t* label_altitude;
    lv_obj_t* label_accel;
    lv_obj_t* label_gyro;
    float latitude;
    float longitude;
    float accel[3];
    float gyro[3];
} lcd_status_t;

static lcd_status_t lcd_status;

void create_interface() {
    ESP_LOGI(TAG, "Creating interface...");

    lv_theme_default_init(NULL, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, &lv_font_montserrat_20);

    lv_style_init(&lcd_status.style_text);
    lv_style_set_text_font(&lcd_status.style_text, &lv_font_montserrat_20);

    // Create Status label
    lcd_status.label_status = lv_label_create(lv_scr_act());
    lv_obj_add_style(lcd_status.label_status, &lcd_status.style_text, 0);
    lv_label_set_text(lcd_status.label_status, "Status: Normal");
    lv_obj_align(lcd_status.label_status, LV_ALIGN_TOP_LEFT, 10, 10);

    // Create Description label
    lcd_status.label_desc = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_desc, "Desc: some string");
    lv_obj_align_to(lcd_status.label_desc, lcd_status.label_status, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

    // Create Time label
    lcd_status.label_time = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_time, "Time: 00:00:00 GTC");
    lv_obj_align_to(lcd_status.label_time, lcd_status.label_desc, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 20);

    // Create Longitude label
    lcd_status.label_latitude = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_latitude, "Longitude: 000.00");
    lv_obj_align_to(lcd_status.label_latitude, lcd_status.label_time, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 5);

    // Create Latitude label
    lcd_status.label_longitude = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_longitude, "Latitude: 00.00");
    lv_obj_align_to(lcd_status.label_longitude, lcd_status.label_latitude, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                    5);

    // Create Velocity label
    lcd_status.label_velocity = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_velocity, "Velocity: 00.00");
    lv_obj_align_to(lcd_status.label_velocity, lcd_status.label_longitude, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                    5);

    // Create Altitude label
    lcd_status.label_altitude = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_altitude, "Altitude: 00.00");
    lv_obj_align_to(lcd_status.label_altitude, lcd_status.label_velocity, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                    5);
    
    // Create Accel label
    lcd_status.label_accel = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_accel, "Accel: 00.00, 00.00, 00.00");
    lv_obj_align_to(lcd_status.label_accel, lcd_status.label_altitude, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                    5);
    
    // Create Altitude label
    lcd_status.label_gyro = lv_label_create(lv_scr_act());
    lv_label_set_text(lcd_status.label_gyro, "Gyro: 00.00, 00.00, 00.00");
    lv_obj_align_to(lcd_status.label_gyro, lcd_status.label_accel, LV_ALIGN_OUT_BOTTOM_LEFT, 0,
                    5);

    ESP_LOGI(TAG, "Interface created.");
}


void lcd_set_latitude(float x) {

}
void lcd_set_longitude(float y) {

}
void lcd_set_accel(float x, float y, float z) {

}
void lcd_set_gyro(float x, float y, float z) {
    lcd_status.gyro[0] = x;
    lcd_status.gyro[1] = y;
    lcd_status.gyro[2] = z;
}

static void lcd_update() {
    char buf[32];
    snprintf(buf, sizeof(buf), "Gyro: %7.3f, %7.3f, %7.3f", lcd_status.gyro[0], lcd_status.gyro[1], lcd_status.gyro[2]);
    lv_label_set_text(lcd_status.label_gyro, buf);
    vTaskDelay(pdMS_TO_TICKS(10));
    lv_task_handler();
}

void lcd_init() {
    lv_port_init();

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

    create_interface();

    const esp_timer_create_args_t periodic_timer_args = {
        .callback = lcd_update,
        .name = "lcd_update",
        .arg = &lcd_update_period,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = true,
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, lcd_update_period * 1000));
}
