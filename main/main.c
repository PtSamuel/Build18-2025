#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "lv_port.h"
#include "lvgl.h"
#include "lv_demos.h"
#include "st7789_driver.h"
#include "esp_log.h"
#include "stdlib.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#define LCD_WIDTH   280
#define LCD_HEIGHT  240

static QueueHandle_t uart_queue;
static uint8_t uart_buffer[1024];

extern lv_disp_drv_t disp_drv;

static const char* TAG = "main";

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

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    while (1)
    {
        if (xQueueReceive(uart_queue, (void*)&event, (TickType_t)portMAX_DELAY)) 
        {
            ESP_LOGI(TAG, "uart event:");
            switch (event.type) {
            case UART_PATTERN_DET:
                ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                uart_read_bytes(UART_NUM_2, uart_buffer, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT]: %s", uart_buffer);
                uart_write_bytes(UART_NUM_2, uart_buffer, event.size);
                break;
            case UART_DATA:
                uart_read_bytes(UART_NUM_2, uart_buffer, event.size, portMAX_DELAY);
                ESP_LOGI(TAG, "[DATA EVT (%d)]: %s", event.size, uart_buffer);
                uart_write_bytes(UART_NUM_2, uart_buffer, event.size);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                uart_flush_input(UART_NUM_2);
                xQueueReset(uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(uart_queue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                ESP_LOGI(TAG, "uart rx break");
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                ESP_LOGI(TAG, "uart parity error");
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                ESP_LOGI(TAG, "uart frame error");
                break;
            default:
                ESP_LOGI(TAG, "uart event type: %d", event.type);
                break;
            }
        }
    }
    vTaskDelete(NULL);
}


#define EXAMPLE_MAX_CHAR_SIZE    64

#define MOUNT_POINT "/sdcard"   //挂载点名称

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

// 主函数
void app_main(void)
{

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
    
    // const int pixel_num = 160 * 160;
    // uint8_t *color_buf = heap_caps_malloc(pixel_num * 2, MALLOC_CAP_DMA);
    // for(int i = 0; i < pixel_num; i++) {

    //     int sector_num = (int)((float)i / pixel_num * 16);

    //     if(sector_num < 8) {
    //         color_buf[2 * i] = 1UL << sector_num;
    //         color_buf[2 * i + 1] = 0;
    //     } else {
    //         color_buf[2 * i] = 0;
    //         color_buf[2 * i + 1] = 1UL << (sector_num - 8);
    //     }

    //     // if(3 * i < pixel_num) {
    //     //     color_buf[2 * i] = 0xF1;
    //     //     color_buf[2 * i + 1] = 0;
    //     // } else if(3 * i < pixel_num * 2) {
    //     //     color_buf[2 * i] = 0x07;
    //     //     color_buf[2 * i + 1] = 0xE0;
    //     // } else {
    //     //     color_buf[2 * i] = 0;
    //     //     color_buf[2 * i + 1] = 0x1F;
    //     // }
    // }

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

    uart_config_t uart_config = {
        .baud_rate = 9600, 
        .data_bits = UART_DATA_8_BITS, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits=UART_STOP_BITS_1
    };
    uart_driver_install(UART_NUM_2, 1024, 1024, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, GPIO_NUM_32, GPIO_NUM_33, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreatePinnedToCore(uart_event_task, "uart", 4096, NULL, 3, NULL, 1);

    uart_intr_config_t uart_intr = {
        .intr_enable_mask = UART_PATTERN_DET,
        .rxfifo_full_thresh = 100,
        .rx_timeout_thresh = 10,
    };
    ESP_ERROR_CHECK(uart_intr_config(UART_NUM_2, &uart_intr));
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_2));
    uart_enable_pattern_det_baud_intr(UART_NUM_2, '\n', 1, 1, 0, 0);
    
    // for(int i = 10; i > 0; i--) {
    //     ESP_LOGI(TAG, "Mounting SD card after %d seconds...", i);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,     //挂载失败是否执行格式化
        .max_files = 5,                     //最大可打开文件数
        .allocation_unit_size = 16 * 1024   //执行格式化时的分配单元大小（分配单元越大，读写越快）
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SDMMC peripheral");

    //默认配置，速度20MHz,使用卡槽1
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = 5000;
    ESP_LOGI(TAG, "Max frequency: %d kHz", host.max_freq_khz);

    //默认的IO管脚配置，
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.cd = SDMMC_SLOT_NO_CD;
    slot_config.wp = SDMMC_SLOT_NO_WP;

    //4位数据
    slot_config.width = 4;

    // slot_config.d0 = GPIO_NUM_16;
    // slot_config.d2 = GPIO_NUM_27;

    //不适用通过IO矩阵进行映射的管脚，只使用默认支持的SDMMC管脚，可以获得最大性能
    #if 0
    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
    #endif
    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    //管脚启用内部上拉
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. ");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");


    while(1)
    {
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
