
#ifndef _GPS_H_
#define _GPS_H_

#include <stdbool.h>

typedef struct {
    char hour[2];
    char minute[2];
    char second[2];
    bool time_valid;
    
    float latitude;
    char latitude_ns;
    float longitude;
    char longitude_ew;
    bool lat_long_valid;

    int num_satellites;
    
    float altitude;
    char altitude_unit;
    bool altitude_valid;

    float velocity;
    bool velocity_valid;
} gps_parse_result_t;

void gps_init();
gps_parse_result_t* gps_get_parse_result();

#endif