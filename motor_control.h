#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize PWM, PIO, ADC, and direction pins
void motor_init(void);

// Update target velocity vectors (called by UDP callback in main_ap)
void motor_set_velocity_reference(float vx, float vy);

// The 10kHz control tick (PID update, encoder read, PWM output)
// This will be called by a repeating timer.
void motor_control_tick(void);

// Safe shutdown
void motor_shutdown(void);

#ifdef __cplusplus
}
#endif

