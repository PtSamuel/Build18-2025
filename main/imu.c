#include "gps.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

static const char *TAG = "IMU";

static spi_device_handle_t spi;

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
        .queue_size = 1,       
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
    spi_device_release_bus(spi);

    vTaskDelay(100);
}

static void swap_16_bit_bytes(void *arr, int size) {
    uint8_t *data = arr;
    int num_elements = size / 2;
    for(int i = 0; i < num_elements; i++) {
        uint8_t temp = data[2 * i];
        data[2 * i] = data[2 * i + 1];
        data[2 * i + 1] = temp;
    }
}

void imu_read() {
    uint8_t rx_buf[14];
    spi_transaction_t t = {
        .addr = 0x80 | 0x1D,
        .length = 8,
        // .rxlength = 8 * sizeof(rx_buf) * sizeof(uint16_t),
        .rxlength = 8 * sizeof(rx_buf),
        .rx_buffer = rx_buf,
        // .flags = SPI_TRANS_USE_RXDATA,
    };

    esp_err_t err;
    err = spi_device_acquire_bus(spi, portMAX_DELAY);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Failure acquiring bus", esp_err_to_name(err));
    }

    err = spi_device_polling_transmit(spi, &t);
    // err = spi_device_transmit(spi, &t);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failure reading register: %s", esp_err_to_name(err));
    } else {
        swap_16_bit_bytes(rx_buf, sizeof(rx_buf));
        uint16_t *registers = rx_buf;
        int16_t *registers_signed = rx_buf;
        ESP_LOGI(TAG, "Register values: (%x/%d, %x/%d, %x/%d, %x/%d, %x/%d, %x/%d, %x/%d)", registers[0], registers_signed[0], registers[1], registers_signed[1], registers[2], registers_signed[2], registers[3], registers_signed[3], registers[4], registers_signed[4], registers[5], registers_signed[5], registers[6], registers_signed[6]); 
        // ESP_LOGI(TAG, "Register value: (%d, %d, %d, %d)", t.rx_data[0], t.rx_data[1], t.rx_data[2], t.rx_data[3]);
    }

    spi_device_release_bus(spi);
}
