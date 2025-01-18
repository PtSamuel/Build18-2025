#include "gps.h"
#include "sd_card.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <string.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

typedef struct {
    QueueHandle_t uart_queue;
    volatile uint8_t uart_buffer[1024];
    volatile gps_parse_result_t gps_parse_result;
} gps_status_t;

static gps_status_t gps_status;

static const char *TAG = "GPS";

char* find_char(const char* str, char c) {
    for( ; *str != '\0' && *str != ','; str++);
    if(*str == '\0')
        return NULL;
    return str;
}

typedef enum {
    TIME,
    LATITUDE,
    LATITUDE_NS,
    LONGITUDE,
    LONGITUDE_EW,
    FIX_TYPE,
    NUM_SATELLITES,
    HORIZONTAL_DILUTION,
    ALTITUDE,
    ALTITUDE_UNIT,
} gga_field_t;

static bool parse_gga_sentence(const char* sentence) {
    char *start, *end;
    
    start = find_char(sentence, ',');
    if(start == NULL) {
        return false;
    }

    start += 1;

    char buf[16];

    gps_status.gps_parse_result.time_valid = true;
    gps_status.gps_parse_result.lat_long_valid = true;
    gps_status.gps_parse_result.altitude_valid = true;
    gps_status.gps_parse_result.velocity_valid = true;

    for(gga_field_t field = TIME; field <= ALTITUDE_UNIT; field++) {   
        end = find_char(start, ',');
        if(end == NULL) {
            return false;
        }

        // if(end == start) {
        //     return false;
        // }

        switch(field) {
            case TIME:
                if(start + 8 != end) {
                    gps_status.gps_parse_result.time_valid = false;
                    break;
                }
                memcpy(gps_status.gps_parse_result.hour, start, 2);
                memcpy(gps_status.gps_parse_result.minute, &start[2], 2);
                memcpy(gps_status.gps_parse_result.second, &start[4], 2);
                break;
            case LATITUDE:
            {
                if(start + 9 != end) {
                    gps_status.gps_parse_result.lat_long_valid = false;
                    break;
                }

                memcpy(start + 2, buf, 7);
                buf[7] = '\0';
                float minutes = atof(buf);

                memcpy(start, buf, 2);
                buf[2] = '\0';
                float latitude = (float) atoi(buf) + minutes / 60.0f;
                gps_status.gps_parse_result.latitude = latitude;
                break;
            }
            case LATITUDE_NS:
                if(start + 1 != end) {
                    gps_status.gps_parse_result.lat_long_valid = false;
                    break;
                }
                gps_status.gps_parse_result.latitude_ns = *start;
                break;
            case LONGITUDE:
            {
                if(start + 10 != end) {
                    gps_status.gps_parse_result.lat_long_valid = false;
                    break;
                }

                memcpy(start + 3, buf, 7);
                buf[7] = '\0';
                float minutes = atof(buf);

                memcpy(start, buf, 3);
                buf[3] = '\0';
                float longitutde = (float) atoi(buf) + minutes / 60.0f;
                gps_status.gps_parse_result.longitude = longitutde;
                break;
            }
            case LONGITUDE_EW:
                if(start + 1 != end) {
                    gps_status.gps_parse_result.lat_long_valid = false;
                    break;
                }
                gps_status.gps_parse_result.longitude_ew = *start;
                break;
            case NUM_SATELLITES:
                memcpy(buf, start, end - start);
                buf[end - start] = '\0';
                gps_status.gps_parse_result.num_satellites = atoi(buf);
                break;
            case ALTITUDE:
                memcpy(buf, start, end - start);
                buf[end - start] = '\0';
                gps_status.gps_parse_result.altitude = atof(buf);
                break;
            case ALTITUDE_UNIT:
                if(start + 1 != end) {
                    gps_status.gps_parse_result.altitude_valid = false;
                    break;
                }
                gps_status.gps_parse_result.altitude_unit = *start;
                break;
            default:
                break;
        }

        start = end + 1;
    }

    return true;
}

void nmea_0813_sentence_handler() {
    int pos = uart_pattern_pop_pos(UART_NUM_2);
    if(pos != -1) {
        int read_length = uart_read_bytes(UART_NUM_2, gps_status.uart_buffer, pos + 1, portMAX_DELAY);
        read_length = MIN(sizeof(gps_status.uart_buffer), read_length);
        gps_status.uart_buffer[read_length] = '\0';
        // ESP_LOGI(TAG, "%s", uart_buffer);
        const char* message_id = (const char*) &gps_status.uart_buffer[3];
        if(strstr(message_id, "RMC") || strstr(message_id, "GGA")) {
            // ESP_LOGI(TAG, "[DET (%d)] %s", read_length, gps_status.uart_buffer);
            if(sd_card_inited()) {
                memcpy(sd_card_get_buffer(), gps_status.uart_buffer, read_length + 1);
                sd_card_set_write_ready();
            }
        }
        if(strstr(message_id, "GGA")) {
            ESP_LOGI(TAG, "[DET (%d)] %s", read_length, gps_status.uart_buffer);
            parse_gga_sentence(gps_status.uart_buffer);
            volatile gps_parse_result_t *result = &gps_status.gps_parse_result;
            ESP_LOGI(TAG, "Parse result: %c%c:%c%c:%c%c, valid: %d, %f %c, %f %c, %d, %f %c, %f", 
                result->hour[0], result->hour[1],
                result->minute[0], result->minute[1],
                result->second[0], result->second[1],
                result->lat_long_valid,
                result->latitude, result->latitude_ns,
                result->longitude, result->longitude_ew,
                result->num_satellites,
                result->altitude,
                result->altitude_unit,
                result->velocity
            );
        }
    } else {
        ESP_LOGW(TAG, "Pattern Queue Size too small");
        uart_flush_input(UART_NUM_2);
    }
}

gps_parse_result_t* gps_get_parse_result() {
    return &gps_status.gps_parse_result;
}

static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    while (1)
    {
        if (xQueueReceive(gps_status.uart_queue, (void*)&event, (TickType_t)portMAX_DELAY)) 
        {
            switch (event.type) {
            case UART_PATTERN_DET:
                nmea_0813_sentence_handler();
                break;
            case UART_DATA:
                int read_length = uart_read_bytes(UART_NUM_2, gps_status.uart_buffer, event.size, portMAX_DELAY);
                read_length = MIN(sizeof(gps_status.uart_buffer), read_length);
                gps_status.uart_buffer[read_length] = '\0';
                ESP_LOGI(TAG, "[DATA (%d)]: %s", event.size, gps_status.uart_buffer);
                break;
            //Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                ESP_LOGI(TAG, "hw fifo overflow");
                uart_flush_input(UART_NUM_2);
                xQueueReset(gps_status.uart_queue);
                break;
            //Event of UART ring buffer full
            case UART_BUFFER_FULL:
                ESP_LOGI(TAG, "ring buffer full");
                // If buffer full happened, you should consider increasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(gps_status.uart_queue);
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

void gps_init() {
    uart_config_t uart_config = {
        .baud_rate = 9600, 
        .data_bits = UART_DATA_8_BITS, 
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, 
        .parity = UART_PARITY_DISABLE, 
        .stop_bits=UART_STOP_BITS_1
    };
    uart_driver_install(UART_NUM_2, 1024, 1024, 20, &gps_status.uart_queue, 0);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreatePinnedToCore(uart_event_task, "uart", 4096, NULL, 3, NULL, 1);

    uart_enable_pattern_det_baud_intr(UART_NUM_2, '\n', 1, 9, 0, 0);
    uart_pattern_queue_reset(UART_NUM_2, 128);
    uart_flush(UART_NUM_2);
}
