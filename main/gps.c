#include "gps.h"
#include "sd_card.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

static QueueHandle_t uart_queue;
static uint8_t uart_buffer[1024];

static const char *TAG = "GPS";

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
                    // ESP_LOGI(TAG, "%s", uart_buffer);
                    const char* message_id = (const char*) &uart_buffer[3];
                    if(strstr(message_id, "RMC") || strstr(message_id, "GGA")) {
                        // ESP_LOGI(TAG, "[DET (%d)] %s", event.size, read_length, uart_buffer);
                        if(sd_card_inited()) {
                            memcpy(sd_card_get_buffer(), uart_buffer, read_length + 1);
                            sd_card_set_write_ready();
                        }
                    }
                } else {
                    ESP_LOGW(TAG, "Pattern Queue Size too small");
                    uart_flush_input(UART_NUM_2);
                }
                break;
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
                // ESP_LOGI(TAG, "uart rx break");
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

void gps_init() {
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

    uart_enable_pattern_det_baud_intr(UART_NUM_2, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(UART_NUM_2, 128);
    uart_flush(UART_NUM_2);
}
