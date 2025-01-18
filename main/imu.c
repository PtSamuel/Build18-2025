#include "gps.h"
#include "lcd.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "esp_timer.h"
#include "math.h"

#define G (9.81)
#define ACCEL_HALF_RANGE (16 * G)
#define ACCEL_FULL_RANGE (2 * ACCEL_HALF_RANGE)

#define DEGREE_TO_RADIAN (M_PI / 180.0)
#define GYRO_HALF_RANGE (2000 * DEGREE_TO_RADIAN)
#define GYRO_FULL_RANGE (2 * GYRO_HALF_RANGE)

#define INT16_RANGE ((double) ((int) INT16_MAX - (int) INT16_MIN))
#define INT16_TO_MINUS_PLUS_ONE(x) ((double) ((int) x - (int) (INT16_MIN)) / INT16_RANGE) 

#define GET_ACCEL(x) (-ACCEL_HALF_RANGE + INT16_TO_MINUS_PLUS_ONE(x) * ACCEL_FULL_RANGE)
#define GET_GYRO(x) (-GYRO_HALF_RANGE + INT16_TO_MINUS_PLUS_ONE(x) * GYRO_FULL_RANGE)

static const char *TAG = "IMU";

static spi_device_handle_t spi; 

static uint8_t rx_buf[14];

const static uint32_t tick_inc_period_ms = 500;

static void swap_16_bit_bytes(void *arr, int size) {
    uint8_t *data = arr;
    int num_elements = size / 2;
    for(int i = 0; i < num_elements; i++) {
        uint8_t temp = data[2 * i];
        data[2 * i] = data[2 * i + 1];
        data[2 * i + 1] = temp;
    }
}

static void imu_read_interrupt() {
    static bool led_status = 0;
    if(led_status) {
        gpio_set_level(GPIO_NUM_2, 1);
    } else {
        gpio_set_level(GPIO_NUM_2, 0);
    }
    led_status = !led_status;

    spi_transaction_t t = {
        .addr = 0x80 | 0x1D,
        .length = 8,
        .rxlength = 8 * sizeof(rx_buf),
        .rx_buffer = rx_buf,
    };

    esp_err_t err = spi_device_transmit(spi, &t);

    if(err != ESP_OK) {
        // ESP_LOGE(TAG, "spi_device_get_trans_result failed: %s", esp_err_to_name(err));
        return;
    }
    swap_16_bit_bytes(rx_buf, sizeof(rx_buf));
    int16_t *registers_signed = rx_buf;

    float temperature = registers_signed[0] / 132.48f + 25.0f;
    double accel_x = GET_ACCEL(registers_signed[1]);
    double accel_y = GET_ACCEL(registers_signed[2]);
    double accel_z = GET_ACCEL(registers_signed[3]);
    double gyro_x = GET_GYRO(registers_signed[4]);
    double gyro_y = GET_GYRO(registers_signed[5]);
    double gyro_z = GET_GYRO(registers_signed[6]);
    
    // ESP_LOGI(TAG, "Register values: (%d, %d, %d, %d, %d, %d, %d)", registers_signed[0], registers_signed[1], registers_signed[2], registers_signed[3], registers_signed[4], registers_signed[5], registers_signed[6]);
    // ESP_LOGI(TAG, "%f %d %f", INT16_RANGE, registers_signed[1], INT16_TO_MINUS_PLUS_ONE(registers_signed[1]));
    ESP_LOGI(TAG, "Register values: (%f, %f, %f, %f, %f, %f, %f)", temperature, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
    lcd_set_gyro(gyro_x, gyro_x, gyro_z);
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
