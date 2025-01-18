
#ifndef _IMU_H_
#define _IMU_H_

void imu_init();
void imu_read();

typedef enum {
    AXIS_X = 0,
    AXIS_Y,
    AXIS_Z,
} axis_t;

float imu_get_temperature();
double imu_get_gyro(axis_t axis);
double imu_get_accel(axis_t axis);

#endif