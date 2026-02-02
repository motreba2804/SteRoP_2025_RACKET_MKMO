#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "icm20948.h"
#include "platform.h"
#include "msg.h"

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

    const float k_pitch = 2.0f;
    const float k_roll  = 2.0f;
    const float v_max   = 2.5f;
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
        
        UART_Packet pkt;
        pkt.startByte = 0xAA;

        pkt.posX = map_to_u16(state.x);
        pkt.posY = map_to_u16(state.y);

        pkt.rotation = (int16_t)((state.roll+1.5f) * 1000.f);

        uint8_t crc = 0;
        uint8_t* ptr = (uint8_t*)&pkt;

        for (size_t i = 0; i < sizeof(UART_Packet)-1; i++) {
            crc ^= ptr[i];
        }
        pkt.checksum = crc;

        uart_write_blocking(UART_PORT, (const uint8_t*)&pkt, sizeof(pkt));
    
        // printf("SRCX: %.3f | SENDX: %u | SRCY: %.3f | SENDY: %u | ROTsrc: %.3f | ROTsend: %u\n", state.x, pkt.posX, state.y, pkt.posY, state.roll ,pkt.rotation);  
        // sleep_ms(16);
    }
}
