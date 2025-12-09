#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "icm20948.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_PORT i2c0

// --- filtr komplementarny ---
float complementary_filter(float prev_angle, float gyro, float acc, float other_acc, float z_acc, float dt) {
    const float alpha = 0.5f;
    float acc_angle = atan2f(acc, z_acc);
    float gyro_angle = prev_angle + gyro * dt;
    return alpha * gyro_angle + (1.0f - alpha) * acc_angle;
}

// --- przeliczenie kąta na prędkość z deadzone ---
float angle_to_velocity(float angle, float k, float angle_min) {
    if (fabs(angle) < angle_min) return 0.0f;
    if (angle > 0.0f) return k * (angle - angle_min);
    else              return k * (angle + angle_min);
}

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Start!\n");

    // --- I2C ---
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    icm20948_init();
    sleep_ms(100);

    float pitch = 0.0f;
    float roll = 0.0f;
    float x = 0.0f, y = 0.0f;

    const float k_pitch = 0.5f;      // prędkość m/s na rad pitch
    const float k_roll  = 0.5f;      // prędkość m/s na rad roll
    const float v_max   = 1.0f;      // maksymalna prędkość m/s
    const float angle_deadzone = 0.05f; // minimalny kąt do startu (~2.8°)

    absolute_time_t last = get_absolute_time();

    while (true) {
        float ax, ay, az;
        float gx, gy, gz;

        read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz);

        gx *= M_PI / 180.0f;
        gy *= M_PI / 180.0f;
        gz *= M_PI / 180.0f;

        absolute_time_t now = get_absolute_time();
        float dt = absolute_time_diff_us(last, now) / 1e6f;
        last = now;

        // --- filtry komplementarne ---
        pitch = complementary_filter(pitch, gy, ax, ay, az, dt);
        roll  = complementary_filter(roll, gx, ay, ax, az, dt);

        // --- prędkości zależne od kąta z deadzone ---
        float vx = angle_to_velocity(roll, k_roll, angle_deadzone);
        float vy = angle_to_velocity(pitch,  k_pitch,  angle_deadzone);

        // --- ograniczenie maksymalnej prędkości ---
        if (vx > v_max) vx = v_max;
        if (vx < -v_max) vx = -v_max;
        if (vy > v_max) vy = v_max;
        if (vy < -v_max) vy = -v_max;

        // --- integracja do położenia ---
        x += vx * dt;
        y += vy * dt;

        if (x < -1.0f) x = -1.0;
        if (x > 1.0f) x = 1.0f;
        if (y < -1.0f) y = -1.0;
        if (y > 1.0f) y = 1.0f;
        
        printf("pitch=%.2f deg | roll=%.2f deg | x=%.3f m | y=%.3f m | vx=%.2f | vy=%.2f\n",
               pitch*180.0f/M_PI, roll*180.0f/M_PI, x, y, vx, vy);

        sleep_ms(10);
    }
}
