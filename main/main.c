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

#include <math.h>

#define LCD_WIDTH   280
#define LCD_HEIGHT  240

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static QueueHandle_t uart_queue;
static uint8_t uart_buffer[1024];
static uint8_t sd_card_buffer[1024];
SemaphoreHandle_t sd_card_write_protect;

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

static void sd_card_task(void *pvParameters) {
    uart_event_t event;
    while (1)
    {
        if(pdTRUE == xSemaphoreTake(sd_card_write_protect, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Receiving SD card buffer: %s", sd_card_buffer);
        }
    }
    vTaskDelete(NULL);
}

// void parse_nmea_0813(const char *str, int length) {
    
// }

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    while (1)
    {
        if (xQueueReceive(uart_queue, (void*)&event, (TickType_t)portMAX_DELAY)) 
        {
            switch (event.type) {
            case UART_PATTERN_DET:
                int pos = uart_pattern_pop_pos(UART_NUM_2);
                if(pos != -1) {
                    int read_length = uart_read_bytes(UART_NUM_2, uart_buffer, pos + 1, portMAX_DELAY);
                    read_length = MIN(sizeof(uart_buffer), read_length);
                    uart_buffer[read_length] = '\0';
                    // parse_nmea_0813(uart_buffer, read_length);
                    const char* message_id = (const char*) &uart_buffer[3];
                    if(strstr(message_id, "RMC") || strstr(message_id, "GGA")) {
                        ESP_LOGI(TAG, "[DET (%d) (%d)] %s", event.size, read_length, uart_buffer);
                        memcpy(sd_card_buffer, uart_buffer, read_length + 1);
                        xSemaphoreGive(sd_card_write_protect);
                    }
                    break;
                } else {
                    ESP_LOGW(TAG, "Pattern Queue Size too small");
                    uart_flush_input(UART_NUM_2);
                }
            case UART_DATA:
                int read_length = uart_read_bytes(UART_NUM_2, uart_buffer, event.size, portMAX_DELAY);
                read_length = MIN(sizeof(uart_buffer), read_length);
                uart_buffer[read_length] = '\0';
                ESP_LOGI(TAG, "[DATA (%d)]: %s", event.size, uart_buffer);
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

    sd_card_write_protect = xSemaphoreCreateBinary();

    uart_config_t uart_config = {
        .baud_rate = 9600, 
        .data_bits = UART_DATA_8_BITS, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits=UART_STOP_BITS_1
    };
    uart_driver_install(UART_NUM_2, 1024, 1024, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreatePinnedToCore(uart_event_task, "uart", 4096, NULL, 3, NULL, 1);

    // uart_intr_config_t uart_intr = {
    //     .intr_enable_mask = UART_PATTERN_DET,
    //     .rxfifo_full_thresh = 100,
    //     .rx_timeout_thresh = 10,
    // };
    // ESP_ERROR_CHECK(uart_intr_config(UART_NUM_2, &uart_intr));
    // ESP_ERROR_CHECK(uart_enable_rx_intr(UART_NUM_2));
    uart_enable_pattern_det_baud_intr(UART_NUM_2, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(UART_NUM_2, 128);
    uart_flush(UART_NUM_2);

    xTaskCreatePinnedToCore(sd_card_task, "uart", 4096, NULL, 3, NULL, 0);
    
    // esp_err_t ret;
    // esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    //     .format_if_mount_failed = true,
    //     .max_files = 5,
    //     .allocation_unit_size = 16 * 1024
    // };
    // sdmmc_card_t *card;
    // const char mount_point[] = MOUNT_POINT;
    // ESP_LOGI(TAG, "Initializing SD card");

    // ESP_LOGI(TAG, "Using SDMMC peripheral");

    // sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // // host.max_freq_khz = SDMMC_FREQ_PROBING;
    // sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // slot_config.cd = SDMMC_SLOT_NO_CD;
    // slot_config.wp = SDMMC_SLOT_NO_WP;
    // slot_config.width = 1;
    // slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // ESP_LOGI(TAG, "Mounting filesystem");
    // ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    // if (ret != ESP_OK) {
    //     if (ret == ESP_FAIL) {
    //         ESP_LOGE(TAG, "Failed to mount filesystem. ");
    //     } else {
    //         ESP_LOGE(TAG, "Failed to initialize the card (%s). "
    //                  "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
    //     }
    //     return;
    // }
    // ESP_LOGI(TAG, "Filesystem mounted");

    // sdmmc_card_print_info(stdout, card);

    // const char *file_hello = MOUNT_POINT"/hello.txt";
    // char data[EXAMPLE_MAX_CHAR_SIZE];
    // snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
    // ret = s_example_write_file(file_hello, data);
    // if (ret != ESP_OK) {
    //     return;
    // }

    // ret = s_example_read_file(file_hello);
    // if (ret != ESP_OK) {
    //     return;
    // }

    // ESP_LOGI(TAG, "Using SPI peripheral");
    // sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    // spi_bus_config_t bus_cfg = {
    //     .mosi_io_num = GPIO_NUM_22,
    //     .miso_io_num = GPIO_NUM_23,
    //     .sclk_io_num = GPIO_NUM_27,
    //     .quadwp_io_num = -1,
    //     .quadhd_io_num = -1,
    //     .max_transfer_sz = 4000,
    // };
    // ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    // if (ret != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to initialize bus.");
    //     return;
    // }

    // sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    // slot_config.gpio_cs = GPIO_NUM_25;
    // slot_config.host_id = host.slot;

    // ESP_LOGI(TAG, "Mounting filesystem");
    // ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    // if (ret != ESP_OK) {
    //     if (ret == ESP_FAIL) {
    //         ESP_LOGE(TAG, "Failed to mount filesystem. "
    //                  "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
    //     } else {
    //         ESP_LOGE(TAG, "Failed to initialize the card (%s). "
    //                  "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
    //     }
    //     return;
    // }
    // ESP_LOGI(TAG, "Filesystem mounted");

    // sdmmc_card_print_info(stdout, card);

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
    //     .mode = 3,                              // SPI mode 0
    //     .spics_io_num = GPIO_NUM_25,             // CS pin
    //     .queue_size = 1,       
    //     // .pre_cb = cs_high,
    //     // .post_cb = cs_low,              
    // };
    // //Initialize the SPI bus
    // ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    // ESP_ERROR_CHECK(ret);
    // //Attach the LCD to the SPI bus
    // ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    // ESP_ERROR_CHECK(ret);

    // gpio_config_t cs_cfg = {
    //     .pin_bit_mask = BIT64(GPIO_NUM_25),
    //     .mode = GPIO_MODE_OUTPUT,
    // };
    // gpio_config(&cs_cfg);

    // spi_transaction_t t = {
    //     .length = 8,
    //     .tx_data = {0x80 | 0x34},
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
