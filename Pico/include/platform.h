#ifndef PLATFORM_H
#define PLATFORM_H

#include "pico/stdlib.h"

// Struktura do przechowywania stanu platformy
typedef struct {
    float pitch;
    float roll;
    float x;
    float y;
} platform_state;

// inicjalizacja stanu
void motion_init(platform_state *state);

// filtr komplementarny
float complementary_filter(float prev_angle, float gyro, float acc, float other_acc, float z_acc, float dt);

// przeliczenie kąta na prędkość z deadzone
float angle_to_velocity(float angle, float k, float angle_min);

// aktualizacja pozycji (pitch → vx, roll → vy)
void motion_update(platform_state *state, float ax, float ay, float az, float gx, float gy, float gz, float dt,
                   float k_pitch, float k_roll, float angle_deadzone, float v_max);
#endif