#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize PWM, PIO encoder, and direction pins
void motor_init(void);

// Update target velocity vectors (called from the main loop in main_ap)
void motor_set_velocity_reference(float vx, float vy);

// The 10kHz control tick (PI update, encoder read, PWM output)
// This will be called by a repeating timer. Must not call printf/stdio.
void motor_control_tick(void);

// Latest encoder speed and control outputs, for printing outside the ISR
void motor_get_telemetry(float *v_enc, float *c_left, float *c_right);

// Zero the reference and controller state, cut PWM, and coast the motors
void motor_stop(void);

// Same as motor_stop, but short-brakes the motors (IN1 = IN2 = 1) instead of coasting
void motor_brake(void);

// Cut PWM, coast, and put both drivers in standby (STBY low) before deep sleep
void motor_shutdown(void);

#ifdef __cplusplus
}
#endif

