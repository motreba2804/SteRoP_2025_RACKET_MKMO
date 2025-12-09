#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "icm20948.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_PORT i2c0

#define DT 0.01f
#define ALPHA 0.5f

int main() {
    stdio_init_all();
    sleep_ms(2000);

    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    icm20948_init();

    float pitch = 0, roll = 0;

    while (1) {
        float ax, ay, az, gx, gy, gz;

        read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz);

        float acc_pitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
        float acc_roll  = atan2f(-ax, az) * 57.2958f;

        pitch = ALPHA * (pitch + gx * DT) + (1 - ALPHA) * acc_pitch;
        roll  = ALPHA * (roll  + gy * DT) + (1 - ALPHA) * acc_roll;

        printf("Pitch: %.2f  Roll: %.2f\n", pitch, roll);
        sleep_ms(10);
    }
}
