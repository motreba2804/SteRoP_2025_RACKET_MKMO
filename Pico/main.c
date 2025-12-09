#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// ===================== KONFIGURACJA =====================
#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_PORT i2c0
#define ICM20948_ADDR 0x69   // jeśli AD0=GND, zmień na 0x68

// Rejestry
#define REG_BANK_SEL       0x7F
#define WHO_AM_I           0x00
#define PWR_MGMT_1         0x06
#define ODR_ALIGN_EN       0x09
#define GYRO_SMPLRT_DIV    0x00
#define GYRO_CONFIG_1      0x01
#define ACCEL_SMPLRT_DIV_1 0x10
#define ACCEL_SMPLRT_DIV_2 0x11
#define ACCEL_CONFIG       0x14
#define GYRO_DATA_REG      0x33
#define ACCEL_DATA_REG     0x2D

// Stałe fizyczne i filtr
#define GYRO_SENS 65.5f         // dla ±500 dps
#define ACCEL_SENS 16384.0f     // dla ±2g
#define DT 0.01f                // 10ms = 100Hz
#define ALPHA 0.98f             // współczynnik filtru komplementarnego

// ===================== FUNKCJE POMOCNICZE =====================
void write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, data, 2, false);
}

void set_bank(uint8_t bank) {
    write_register(REG_BANK_SEL, bank << 4);
}

// ===================== INICJALIZACJA CZUJNIKA =====================
void icm20948_init(void) {
    sleep_ms(100);

    set_bank(0);
    uint8_t who_am_i = 0;
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, (uint8_t[]){WHO_AM_I}, 1, true);
    i2c_read_blocking(I2C_PORT, ICM20948_ADDR, &who_am_i, 1, false);

    printf("WHO_AM_I = 0x%02X\n", who_am_i);
    if (who_am_i != 0xEA) {
        printf("Nie wykryto ICM20948! Sprawdź połączenie.\n");
        while (1) sleep_ms(1000);
    }

    // Reset
    write_register(PWR_MGMT_1, 0x80);
    sleep_ms(100);

    // Wybudzenie
    write_register(PWR_MGMT_1, 0x01);
    sleep_ms(50);

    // --- Konfiguracja Bank 2 ---
    set_bank(2);

    write_register(ODR_ALIGN_EN, 0x01);

    // Gyro 112 Hz, ±500 dps, BW 5 Hz
    write_register(GYRO_SMPLRT_DIV, 0x09);
    write_register(GYRO_CONFIG_1, 0x05);

    // Accel 112 Hz, ±2g, BW 5 Hz
    write_register(ACCEL_SMPLRT_DIV_1, 0x00);
    write_register(ACCEL_SMPLRT_DIV_2, 0x09);
    write_register(ACCEL_CONFIG, 0x01);

    // Powrót do banku 0
    set_bank(0);

    printf("ICM20948 zainicjalizowany.\n");
}

// ===================== ODCZYT DANYCH =====================
void read_accel_gyro(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    uint8_t reg;
    uint8_t data[6];

    // Akcelerometr
    reg = ACCEL_DATA_REG;
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ICM20948_ADDR, data, 6, false);

    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];

    // Żyroskop
    reg = GYRO_DATA_REG;
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ICM20948_ADDR, data, 6, false);

    int16_t raw_gx = (data[0] << 8) | data[1];
    int16_t raw_gy = (data[2] << 8) | data[3];
    int16_t raw_gz = (data[4] << 8) | data[5];

    *ax = raw_ax / ACCEL_SENS;
    *ay = raw_ay / ACCEL_SENS;
    *az = raw_az / ACCEL_SENS;

    *gx = raw_gx / GYRO_SENS;
    *gy = raw_gy / GYRO_SENS;
    *gz = raw_gz / GYRO_SENS;
}

// ===================== GŁÓWNA PĘTLA =====================
int main() {
    stdio_init_all();
    sleep_ms(2000); // czas na połączenie przez UART

    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    icm20948_init();


    float pitch = 0, roll = 0;
    absolute_time_t last = get_absolute_time();


    while (1) {
        float ax, ay, az, gx, gy, gz;
        read_accel_gyro(&ax, &ay, &az, &gx, &gy, &gz);
    
        // Przybliżone kąty z akcelerometru (stopnie)
        float acc_pitch = atan2f(ay, sqrtf(ax * ax + az * az)) * 180.0f / M_PI;
        float acc_roll  = atan2f(-ax, az) * 180.0f / M_PI;

        // Integracja żyroskopu (dt = 10ms)
        pitch = ALPHA * (pitch + gx * DT) + (1 - ALPHA) * acc_pitch;
        roll  = ALPHA * (roll  + gy * DT) + (1 - ALPHA) * acc_roll;

        printf("Pitch: %7.2f°\tRoll: %7.2f° | Accel: (%.2f,\t%.2f,\t%.2f) g\n",
               pitch, roll, ax, ay, az);

        sleep_ms(10);
    }
}