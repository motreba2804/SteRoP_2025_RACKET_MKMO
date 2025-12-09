#ifndef ICM20948_H
#define ICM20948_H

#include "pico/stdlib.h"

void icm20948_init(void);

void read_accel_gyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);


#endif
