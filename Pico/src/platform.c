#include "platform.h"
#include <math.h>

void motion_init(platform_state *state) {
    state->pitch = 0.0f;
    state->roll  = 0.0f;
    state->x     = 0.0f;
    state->y     = 0.0f;
}

float complementary_filter(float prev_angle, float gyro, float acc, float other_acc, float z_acc, float dt) {
    const float alpha = 0.5f;
    float acc_angle = atan2f(acc, z_acc);
    float gyro_angle = prev_angle + gyro * dt;
    return alpha * gyro_angle + (1.0f - alpha) * acc_angle;
}

float angle_to_velocity(float angle, float k, float angle_min) {
    if (fabs(angle) < angle_min) return 0.0f;
    if (angle > 0.0f) return k * (angle - angle_min);
    else              return k * (angle + angle_min);
}

void motion_update(platform_state *state, float ax, float ay, float az, float gx, float gy, float gz, float dt,
                   float k_pitch, float k_roll, float angle_deadzone, float v_max) {
    // filtry komplementarne
    state->pitch = complementary_filter(state->pitch, gy, ax, ay, az, dt);
    state->roll  = complementary_filter(state->roll, gx, ay, ax, az, dt);

    // prędkości zależne od kąta
    float vx = angle_to_velocity(state->roll, k_roll, angle_deadzone);
    float vy = angle_to_velocity(state->pitch,  k_pitch,  angle_deadzone);

    // ograniczenie prędkości
    if (vx > v_max) vx = v_max;
    if (vx < -v_max) vx = -v_max;
    if (vy > v_max) vy = v_max;
    if (vy < -v_max) vy = -v_max;

    // integracja do pozycji
    state->x += vx * dt;
    state->y += vy * dt;

    float platformHalfSizeX = 40.0f/240.0f;
    float platformHalfSizeY = 10.0f/320.0f;
    float marginX = fabs(cos(state->roll))*platformHalfSizeX + fabs(sin(state->roll))*platformHalfSizeY;
    float marginY = fabs(sin(state->roll))*platformHalfSizeX + fabs(cos(state->roll))*platformHalfSizeY;

    if(state->x <= (-1.0f+marginX)) state->x = -1.0f+marginX;
    if(state->x >= (1.0f-marginX)) state->x = 1.0f-marginX;
    if(state->y <= (0.2f+marginY)) state->y = 0.2f+marginY;
    if(state->y >= (1.0f-marginY)) state->y = 1.0f-marginY;

    if(state->roll <= -1.0f) state->roll = -1.0f;
    if(state->roll >= 1.0f) state->roll = 1.0f;

}
