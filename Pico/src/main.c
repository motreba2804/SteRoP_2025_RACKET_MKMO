#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "icm20948.h"
#include "platform.h"

#define SDA_PIN 16
#define SCL_PIN 17
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define I2C_PORT i2c0
#define UART_PORT uart0

int main() {
    stdio_init_all();
    sleep_ms(2000);
    printf("Start!\n");

    // I2C
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // UART
    uart_init(UART_PORT, 115200);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    gpio_disable_pulls(UART_TX_PIN);
    gpio_disable_pulls(UART_RX_PIN);

    icm20948_init();
    sleep_ms(100);

    platform_state state;
    motion_init(&state);

    const float k_pitch = 0.5f;
    const float k_roll  = 0.5f;
    const float v_max   = 1.0f;
    const float angle_deadzone = 0.05f;

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

        motion_update(&state, ax, ay, az, gx, gy, gz, dt, k_pitch, k_roll, angle_deadzone, v_max);

        char buf[100];
        int len = snprintf(buf, sizeof(buf),
                       "Pitch: %.2f\tRoll: %.2f\tPos: x: %.2f, y: %.2f\n",
                       state.pitch, state.roll, state.x, state.y);

        // Wysyłka po UART
        uart_write_blocking(UART_PORT, (const uint8_t*)buf, len);
    
        printf("pitch=%.2f deg | roll=%.2f deg | x=%.3f m | y=%.3f m\n",
               state.pitch*180.0f/M_PI, state.roll*180.0f/M_PI, state.x, state.y);    
        sleep_ms(10);
    }
}
