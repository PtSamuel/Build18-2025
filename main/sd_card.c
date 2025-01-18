#include "sd_card.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

static const char *TAG = "SD_CARD";

#define EXAMPLE_MAX_CHAR_SIZE    64

#define MOUNT_POINT "/sdcard"   //挂载点名称

static char filename[32] = MOUNT_POINT"/log_0.txt";
static FILE *f;

uint8_t sd_card_buffer[1024];
SemaphoreHandle_t sd_card_write_protect;

static void dump_file(FILE *file)
{
    int i = 0;
    char buffer[64];
    size_t bytes_read;

    while ((bytes_read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        ESP_LOGI(TAG, "%s", buffer);
    }
}

void sd_card_dump_files()
{
    int i = 0;
    FILE *file;
    while(true) {
        snprintf(filename, 32, MOUNT_POINT"/log_%d.txt", i);
        ESP_LOGI(TAG, "Scanning file %s", filename);
        file = fopen(filename, "r"); 
        if (file) {
            dump_file(file);
            fclose(file);
            i++;
        } else {
            ESP_LOGI(TAG, "File: %s does not exist", filename);
            break;
        }
    }
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

void sd_card_init() {
    
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

    xTaskCreatePinnedToCore(sd_card_task, "sd_card", 4096, NULL, 3, NULL, 0);
}

uint8_t* sd_card_get_buffer() {
    return sd_card_buffer;
}

void sd_card_set_write_ready() {
    xSemaphoreGive(sd_card_write_protect);
}
