#include "icm20948.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <math.h>

// ===================== KONFIGURACJA =====================
#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_PORT i2c0
#define ICM20948_ADDR 0x69   // AD0=HIGH → 0x69

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

// Parametry przeliczeń
#define GYRO_SENS 65.5f         // ±500dps
#define ACCEL_SENS 16384.0f     // ±2g

// ===================== Funkcje pomocnicze =====================
static void write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, data, 2, false);
}

static void set_bank(uint8_t bank) {
    write_register(REG_BANK_SEL, bank << 4);
}

// ===================== Inicjalizacja =====================
void icm20948_init(void) {
    sleep_ms(100);

    // Test WHO_AM_I
    set_bank(0);
    uint8_t who_am_i = 0;

    uint8_t reg = WHO_AM_I;
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ICM20948_ADDR, &who_am_i, 1, false);

    printf("WHO_AM_I = 0x%02X\n", who_am_i);

    if (who_am_i != 0xEA) {
        printf("Nie wykryto ICM20948!\n");
        while (1) sleep_ms(1000);
    }

    // Reset
    write_register(PWR_MGMT_1, 0x80);
    sleep_ms(100);

    // Wybudzenie
    write_register(PWR_MGMT_1, 0x01);
    sleep_ms(50);

    // Bank 2 – konfiguracja
    set_bank(2);
    write_register(ODR_ALIGN_EN, 0x01);

    write_register(GYRO_SMPLRT_DIV, 0x09);
    write_register(GYRO_CONFIG_1, 0x05);

    write_register(ACCEL_SMPLRT_DIV_1, 0x00);
    write_register(ACCEL_SMPLRT_DIV_2, 0x09);
    write_register(ACCEL_CONFIG, 0x01);

    set_bank(0);

    printf("ICM20948 zainicjalizowany.\n");
}

// ===================== Odczyt danych =====================
void read_accel_gyro(float *ax, float *ay, float *az,
                     float *gx, float *gy, float *gz) 
{
    uint8_t reg;
    uint8_t data[6];

    // ---- Akcelerometr ----
    reg = ACCEL_DATA_REG;
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ICM20948_ADDR, data, 6, false);

    *ax = ((int16_t)(data[0] << 8 | data[1])) / ACCEL_SENS;
    *ay = ((int16_t)(data[2] << 8 | data[3])) / ACCEL_SENS;
    *az = ((int16_t)(data[4] << 8 | data[5])) / ACCEL_SENS;

    // ---- Żyroskop ----
    reg = GYRO_DATA_REG;
    i2c_write_blocking(I2C_PORT, ICM20948_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, ICM20948_ADDR, data, 6, false);

    *gx = ((int16_t)(data[0] << 8 | data[1])) / GYRO_SENS;
    *gy = ((int16_t)(data[2] << 8 | data[3])) / GYRO_SENS;
    *gz = ((int16_t)(data[4] << 8 | data[5])) / GYRO_SENS;
}
