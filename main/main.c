#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "gps.h"
#include "lcd.h"
#include "esp_log.h"
#include "stdlib.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "lvgl.h"

#include <math.h>


uint8_t sd_card_buffer[1024];
SemaphoreHandle_t sd_card_write_protect;

static const char* TAG = "main";

#define EXAMPLE_MAX_CHAR_SIZE    64

#define MOUNT_POINT "/sdcard"   //挂载点名称

static char filename[32] = MOUNT_POINT"/log_0.txt";
static FILE *f;

static void sd_card_task(void *pvParameters) {
    while (1)
    {
        if(pdTRUE == xSemaphoreTake(sd_card_write_protect, portMAX_DELAY))
        {
            ESP_LOGI(TAG, "Receiving SD card buffer: %s", sd_card_buffer);
            FILE *f = fopen(filename, "a");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file");
            }
            fprintf(f, (char*)sd_card_buffer);
            fclose(f);
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t create_file()
{
    int i = 0;
    while(true) {
        snprintf(filename, 32, MOUNT_POINT"/log_%d.txt", i);
        ESP_LOGI(TAG, "Scanning file %s", filename);
        f = fopen(filename, "r"); 
        if (f) {
            fclose(f);
            i++;
            continue;
        } else {
            ESP_LOGI(TAG, "Found nonexistent file: %s", filename);
            break;
        }
    }

    FILE *f = fopen(filename, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to create file");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Creating file: %s", filename);
    fclose(f);
    return ESP_OK;

    // FILE *f = fopen(filename, "w");
    // if (f == NULL) {
    //     ESP_LOGE(TAG, "Failed to create file");
    //     return ESP_FAIL;
    // }
    // fclose(f);
    // return ESP_OK;
}

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
    lcd_init();
    gps_init();

    sd_card_write_protect = xSemaphoreCreateBinary();
    
    esp_err_t ret;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SDMMC peripheral");

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // host.max_freq_khz = SDMMC_FREQ_PROBING;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.cd = SDMMC_SLOT_NO_CD;
    slot_config.wp = SDMMC_SLOT_NO_WP;
    slot_config.width = 1;
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

    sdmmc_card_print_info(stdout, card);

    xTaskCreatePinnedToCore(sd_card_task, "sd_card", 4096, NULL, 3, NULL, 0);

    // const char *file_hello = MOUNT_POINT"/hello2.txt";
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

    ESP_ERROR_CHECK(create_file());

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
