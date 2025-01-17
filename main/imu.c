#include "gps.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_timer.h"

static const char *TAG = "IMU";

static spi_device_handle_t spi; 

static uint8_t rx_buf[14];


static void swap_16_bit_bytes(void *arr, int size) {
    uint8_t *data = arr;
    int num_elements = size / 2;
    for(int i = 0; i < num_elements; i++) {
        uint8_t temp = data[2 * i];
        data[2 * i] = data[2 * i + 1];
        data[2 * i + 1] = temp;
    }
}

static void spi_result_task(void *pvParameters) {

    spi_transaction_t *t;
    for(;;) {
        esp_err_t err = spi_device_get_trans_result(spi, &t, portMAX_DELAY);
        if(err != ESP_OK) {
            ESP_LOGE(TAG, "spi_device_get_trans_result failed: %s", esp_err_to_name(err));
            continue;
        }
        swap_16_bit_bytes(t->rx_buffer, sizeof(rx_buf));
        int16_t *registers_signed = t->rx_buffer;
        ESP_LOGI(TAG, "Register values: (%d, %d, %d, %d, %d, %d, %d)", registers_signed[0], registers_signed[1], registers_signed[2], registers_signed[3], registers_signed[4], registers_signed[5], registers_signed[6]);
    }
}

static void imu_read_interrupt() {
    gpio_set_level(GPIO_NUM_2, !gpio_get_level(GPIO_NUM_2));

    spi_transaction_t t = {
        .addr = 0x80 | 0x1D,
        .length = 8,
        .rxlength = 8 * sizeof(rx_buf),
        .rx_buffer = rx_buf,
    };

    spi_device_queue_trans(spi, &t, portMAX_DELAY);
}

void imu_init() {
    esp_err_t ret;
    
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_NUM_22,
        .mosi_io_num = GPIO_NUM_23,
        .sclk_io_num = GPIO_NUM_27,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 8,
        .dummy_bits = 0,
        .mode = 0,                              // SPI mode 0
        .clock_speed_hz = 1 * 1000 * 1000,     // Clock out at 1 MHz
        .spics_io_num = GPIO_NUM_25,             // CS pin
        .queue_size = 16,       
        .flags = SPI_DEVICE_HALFDUPLEX,        
    };
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    esp_err_t err;
    err = spi_device_acquire_bus(spi, portMAX_DELAY);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failure acquiring bus", esp_err_to_name(err));
    }

    spi_transaction_t t = {
        .addr = 0x4E,
        .length = 8 + 8,
        .tx_buffer = {0x1F},
        .flags = SPI_TRANS_USE_TXDATA,
    };

    err = spi_device_polling_transmit(spi, &t);

    vTaskDelay(100);

    gpio_config_t led_gpio_cfg = 
    {
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE, 
        .pin_bit_mask = BIT(GPIO_NUM_2),
    };
    gpio_config(&led_gpio_cfg);

    xTaskCreatePinnedToCore(spi_result_task, "spi_result_task", 4096, NULL, 3, NULL, 1);

    static uint32_t tick_inc_period_ms = 500;
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = imu_read_interrupt,
        .name = "imu_read_interrupt",
        .arg = &tick_inc_period_ms,
        .dispatch_method = ESP_TIMER_TASK,
        .skip_unhandled_events = true,
    };

    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, tick_inc_period_ms * 1000));


}

void imu_read() {
    spi_transaction_t t = {
        .addr = 0x80 | 0x1D,
        .length = 8,
        // .rxlength = 8 * sizeof(rx_buf) * sizeof(uint16_t),
        .rxlength = 8 * sizeof(rx_buf),
        .rx_buffer = rx_buf,
        // .flags = SPI_TRANS_USE_RXDATA,
    };

    esp_err_t err;

    err = spi_device_polling_transmit(spi, &t);
    // err = spi_device_transmit(spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failure reading register: %s", esp_err_to_name(err));
    } else {
        swap_16_bit_bytes(rx_buf, sizeof(rx_buf));
        int16_t *registers_signed = rx_buf;
        ESP_LOGI(TAG, "Register values: (%d, %d, %d, %d, %d, %d, %d)", registers_signed[0], registers_signed[1], registers_signed[2], registers_signed[3], registers_signed[4], registers_signed[5], registers_signed[6]); 
    }

}
