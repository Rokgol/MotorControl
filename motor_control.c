#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "encoder_period.pio.h"
#include "motor_control.h"

// ============================================================
// MODUS ENUMERATION
// ============================================================
typedef enum {
    MODUS_COAST   = 0,
    MODUS_FORWARD = 1,
    MODUS_REVERSE = 2,
    MODUS_BRAKE   = 3
} MotorModus;

// ============================================================
// PINS
// ============================================================
// One PWM and one IN1/IN2 pair per motor (no parallel wiring).
// All motors use the same truth table; wiring makes IN1=1/IN2=0 drive forward.
// Left motors on GPIO 0-15, right motors on GPIO 16-28 (IN1, IN2, PWM in a row).
// Each PWM pin is on its own slice/channel: 15=7B, 12=6A, 21=2B, 27=5B.

// --- Left side ---
#define MOTOR_FL_IN1_PIN    13      // Front-left
#define MOTOR_FL_IN2_PIN    14
#define MOTOR_FL_PWM_PIN    15

#define MOTOR_RL_IN1_PIN    10      // Rear-left
#define MOTOR_RL_IN2_PIN    11
#define MOTOR_RL_PWM_PIN    12

// --- Right side ---
#define MOTOR_FR_IN1_PIN    19      // Front-right
#define MOTOR_FR_IN2_PIN    20
#define MOTOR_FR_PWM_PIN    21

#define MOTOR_RR_IN1_PIN    26      // Rear-right (23-25 are used by the Wi-Fi chip)
#define MOTOR_RR_IN2_PIN    27
#define MOTOR_RR_PWM_PIN    22

#define ENCODER_PIN         18
#define MOTOR_STBY_PIN      16      // Shared by both drivers

// Compile-time masks so all 8 direction pins are written in one SIO operation
#define LEFT_IN1_MASK   ((1u << MOTOR_FL_IN1_PIN) | (1u << MOTOR_RL_IN1_PIN))
#define LEFT_IN2_MASK   ((1u << MOTOR_FL_IN2_PIN) | (1u << MOTOR_RL_IN2_PIN))
#define RIGHT_IN1_MASK  ((1u << MOTOR_FR_IN1_PIN) | (1u << MOTOR_RR_IN1_PIN))
#define RIGHT_IN2_MASK  ((1u << MOTOR_FR_IN2_PIN) | (1u << MOTOR_RR_IN2_PIN))
#define ALL_DIR_MASK    (LEFT_IN1_MASK | LEFT_IN2_MASK | RIGHT_IN1_MASK | RIGHT_IN2_MASK)

enum { SIDE_LEFT = 0, SIDE_RIGHT = 1 };

// ============================================================
// CONTROL PARAMETERS
// ============================================================
#define TS                  0.0001f   // 100 us sample period = 10 kHz

#define KP                  1.0f
#define ALPHA               3.0f      // Integral gain; Ti = KP/ALPHA ~ 0.33 s
#define I_LIMIT             0.5f      // Integral only trims the feedforward

#define CONTROL_LIMIT       1.0f
#define V_MAX               0.44f

#define REVERSE_DEAD_TIME_TICKS 100U

#define WHEEL_RADIUS        0.03f
#define ENCODER_MARKS       20

#define ENCODER_US_PER_COUNT    2U      // PIO runs at 1 MHz, 2 cycles per count
#define ENCODER_TIMEOUT_TICKS   2000U   // 200 ms without a pulse -> speed is 0

#define PWM_CLKDIV          125.0f
#define PWM_WRAP            1000U

// Distance between encoder marks
static const float DX_MARK = 2.0f * (float)M_PI * WHEEL_RADIUS / (float)ENCODER_MARKS;

// ============================================================
// STATE VARIABLES
// ============================================================
static float v_ref_x = 0.0f;
static float v_ref_y = 0.0f;
static float v_ref_abs = 0.0f;

static float c_previous = 0.0f;
static int motor_direction = 0;
static int last_active_direction = 1;
static uint32_t brake_until_ticks = 0;
static float integral_Controller = 0.0f;

// Common drive modus chosen by apply_direction_logic; applied to the pins
// once per tick by the output stage
static MotorModus drive_modus = MODUS_COAST;

// Per-side reversal guard (needed for spin turns, where one side can reverse
// while drive_modus stays the same)
static int side_last_dir[2] = {0, 0};
static uint32_t side_dead_until[2] = {0, 0};

static PIO encoder_pio = pio1;
static int encoder_sm = -1;
static float last_measured_speed = 0.0f;
static uint32_t last_pulse_ticks = 0;

static uint32_t sample_ticks = 0;

// Latest values for telemetry, printed from the main loop (never from the ISR)
static volatile float telem_v = 0.0f;
static volatile float telem_c_left = 0.0f;
static volatile float telem_c_right = 0.0f;

// ============================================================
// PIO ENCODER READER
// ============================================================
static float read_pio_encoder_velocity(uint32_t current_ticks) {
    bool new_pulse = false;
    uint32_t pulse_iterations = 0;

    while (!pio_sm_is_rx_fifo_empty(encoder_pio, encoder_sm)) {
        pulse_iterations = pio_sm_get(encoder_pio, encoder_sm);
        new_pulse = true;
    }

    if (new_pulse && pulse_iterations > 0) {
        float delta_t = (float)(pulse_iterations * ENCODER_US_PER_COUNT) * 1e-6f;
        if (delta_t > 0.0f) {
            last_measured_speed = DX_MARK / delta_t;
            last_pulse_ticks = current_ticks;
        }
    } else if ((current_ticks - last_pulse_ticks) > ENCODER_TIMEOUT_TICKS) {
        last_measured_speed = 0.0f;
    }

    float direction_sign = (motor_direction != 0) ? (float)motor_direction : 1.0f;
    return last_measured_speed * direction_sign;
}

// ============================================================
// PWM SETUP
// ============================================================
static void setup_pwm(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, PWM_CLKDIV);
    pwm_config_set_wrap(&config, PWM_WRAP);
    pwm_init(slice, &config, true);
}

static void set_pwm_duty(uint gpio, float duty) {
    duty = fminf(1.0f, fabsf(duty));   // Direction is set by the modus pins
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    uint level = (uint)(duty * (float)PWM_WRAP);
    pwm_set_chan_level(slice, channel, level);
}

// ============================================================
// DIRECTION PINS
// ============================================================
// Truth table, identical for every motor:
//   FORWARD = 1/0, REVERSE = 0/1, COAST = 0/0, BRAKE = 1/1
static inline uint32_t modus_bits(MotorModus m, uint32_t in1_mask, uint32_t in2_mask) {
    uint32_t bits = 0;
    if (m == MODUS_FORWARD || m == MODUS_BRAKE) bits |= in1_mask;
    if (m == MODUS_REVERSE || m == MODUS_BRAKE) bits |= in2_mask;
    return bits;
}

// Sets IN1/IN2 of all four motors in a single register write, so every
// pin switches in the same cycle
static void set_motor_modus(MotorModus left, MotorModus right) {
    uint32_t value = modus_bits(left,  LEFT_IN1_MASK,  LEFT_IN2_MASK)
                   | modus_bits(right, RIGHT_IN1_MASK, RIGHT_IN2_MASK);
    gpio_put_masked(ALL_DIR_MASK, value);
}

static inline MotorModus opposite_modus(MotorModus m) {
    if (m == MODUS_FORWARD) return MODUS_REVERSE;
    if (m == MODUS_REVERSE) return MODUS_FORWARD;
    return m;   // COAST and BRAKE are never flipped
}

// Coasts a side for REVERSE_DEAD_TIME_TICKS when it would flip directly
// between FORWARD and REVERSE. BRAKE and COAST pass through unchanged.
static MotorModus guard_side(int side, MotorModus m, float *duty, uint32_t current_ticks) {
    if (m != MODUS_FORWARD && m != MODUS_REVERSE) {
        side_last_dir[side] = 0;
        return m;
    }

    int dir = (m == MODUS_FORWARD) ? +1 : -1;
    if (current_ticks < side_dead_until[side]) {
        *duty = 0.0f;
        return MODUS_COAST;
    }
    if (side_last_dir[side] != 0 && dir != side_last_dir[side]) {
        side_last_dir[side] = dir;
        side_dead_until[side] = current_ticks + REVERSE_DEAD_TIME_TICKS;
        *duty = 0.0f;
        return MODUS_COAST;
    }
    side_last_dir[side] = dir;
    return m;
}

// ============================================================
// DIRECTION LOGIC (shared by both controllers)
// ============================================================
// Brakes on a hard sign change of the control output, inserts a dead time
// before reversing, and picks drive_modus from the sign of dir_ref (the pins
// are written later by the output stage). Returns the (possibly zeroed) output.
static float apply_direction_logic(float c, float dir_ref, uint32_t current_ticks) {
    int requested_direction = 0;
    if (dir_ref > 0.0f)
        requested_direction = +1;
    else if (dir_ref < 0.0f)
        requested_direction = -1;

    if (fabsf(c) >= CONTROL_LIMIT / 2.0f && copysignf(1.0f, c) != copysignf(1.0f, c_previous)) {
        drive_modus = MODUS_BRAKE;
        c_previous = c;
        return c;
    }

    if (current_ticks < brake_until_ticks) {
        c = 0.0f;
        motor_direction = 0;
    } else if (requested_direction != 0) {
        if (last_active_direction != 0 && requested_direction != last_active_direction) {
            motor_direction = 0;
            c = 0.0f;
            brake_until_ticks = current_ticks + REVERSE_DEAD_TIME_TICKS;
            last_active_direction = requested_direction;
        } else {
            motor_direction = requested_direction;
            last_active_direction = requested_direction;
        }
    } else {
        motor_direction = 0;
    }

    if (motor_direction > 0)
        drive_modus = MODUS_FORWARD;
    else if (motor_direction < 0)
        drive_modus = MODUS_REVERSE;
    else
        drive_modus = MODUS_COAST;

    c_previous = c;
    return c;
}

// ============================================================
// PI CONTROLLER (straight driving, closed loop)
// ============================================================
static float PI_controller_update(float measured_velocity, float v_ref, uint32_t current_ticks) {
    float c_ff = v_ref / V_MAX;   // Feedforward operating point duty (same as the free controller)
    float e = v_ref - measured_velocity;

    integral_Controller += ALPHA * TS * e;
    integral_Controller = fmaxf(-I_LIMIT, fminf(I_LIMIT, integral_Controller));
    float c = c_ff + KP * e + integral_Controller;

    c = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, c));

    // Sign of c selects forward/reverse drive
    return apply_direction_logic(c, c, current_ticks);
}

// ============================================================
// FREE PROPORTIONAL CONTROLLER (turning, open loop)
// ============================================================
static float free_controller_update(uint32_t current_ticks){

    float c = v_ref_abs / V_MAX;   // Pure feedforward; KP is a feedback gain only
    c = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, c));

    // A pure turn (vy == 0) counts as forward; the output stage spins it in place
    float v_dir = (v_ref_y != 0.0f) ? v_ref_y : 1.0f;

    // The PI's feedforward picks up from here, so its integral starts fresh
    integral_Controller = 0.0f;

    return apply_direction_logic(c, v_dir, current_ticks);
}


// ============================================================
// PUBLIC API
// ============================================================

void motor_init(void) {
    setup_pwm(MOTOR_FL_PWM_PIN);
    setup_pwm(MOTOR_RL_PWM_PIN);
    setup_pwm(MOTOR_FR_PWM_PIN);
    setup_pwm(MOTOR_RR_PWM_PIN);

    const uint32_t out_mask = ALL_DIR_MASK | (1u << MOTOR_STBY_PIN);
    gpio_init_mask(out_mask);
    gpio_set_dir_out_masked(out_mask);
    set_motor_modus(MODUS_COAST, MODUS_COAST);
    gpio_put(MOTOR_STBY_PIN, 1);

    encoder_sm = pio_claim_unused_sm(encoder_pio, true);
    uint offset = pio_add_program(encoder_pio, &encoder_period_program);
    encoder_period_program_init(encoder_pio, encoder_sm, offset, ENCODER_PIN);
}

void motor_set_velocity_reference(float vx, float vy) {
    v_ref_x = vx;
    v_ref_y = vy;
    v_ref_abs = sqrtf(v_ref_x * v_ref_x + v_ref_y * v_ref_y);
}

void motor_control_tick(void) {
    gpio_put(MOTOR_STBY_PIN, 1);
    sample_ticks++;

    float measured_v = read_pio_encoder_velocity(sample_ticks);
    float c_left = 0.0f;
    float c_right = 0.0f;
    MotorModus left_modus, right_modus;

    if(v_ref_x == 0){
        c_left = PI_controller_update(measured_v, v_ref_y, sample_ticks);
        c_right = c_left;
        left_modus = right_modus = drive_modus;
    }
    else{
        bool spin = (v_ref_y == 0.0f);   // Pure turn: sides counter-rotate in place
        // Inner wheel runs at the forward share of the command: 1 = straight, 0 = pivot
        float k = spin ? 1.0f : fabsf(v_ref_y) / (v_ref_abs > 0.001f ? v_ref_abs : 1.0f);
        if (v_ref_x > 0) {
            c_left = free_controller_update(sample_ticks);
            c_right = c_left * k;
            left_modus = drive_modus;
            right_modus = spin ? opposite_modus(drive_modus) : drive_modus;
        } else {
            c_right = free_controller_update(sample_ticks);
            c_left = c_right * k;
            right_modus = drive_modus;
            left_modus = spin ? opposite_modus(drive_modus) : drive_modus;
        }
    }

    left_modus  = guard_side(SIDE_LEFT,  left_modus,  &c_left,  sample_ticks);
    right_modus = guard_side(SIDE_RIGHT, right_modus, &c_right, sample_ticks);

    set_pwm_duty(MOTOR_FL_PWM_PIN, c_left);
    set_pwm_duty(MOTOR_RL_PWM_PIN, c_left);
    set_pwm_duty(MOTOR_FR_PWM_PIN, c_right);
    set_pwm_duty(MOTOR_RR_PWM_PIN, c_right);
    set_motor_modus(left_modus, right_modus);

    telem_v = measured_v;
    telem_c_left = c_left;
    telem_c_right = c_right;
}

void motor_get_telemetry(float *v_enc, float *c_left, float *c_right) {
    *v_enc = telem_v;
    *c_left = telem_c_left;
    *c_right = telem_c_right;
}

// Zero the reference and controller state, cut PWM, and set the given modus
static void halt_motors(MotorModus m) {
    motor_set_velocity_reference(0.0f, 0.0f);
    integral_Controller = 0.0f;
    c_previous = 0.0f;
    motor_direction = 0;
    drive_modus = m;
    side_last_dir[SIDE_LEFT] = side_last_dir[SIDE_RIGHT] = 0;
    side_dead_until[SIDE_LEFT] = side_dead_until[SIDE_RIGHT] = 0;

    set_pwm_duty(MOTOR_FL_PWM_PIN, 0.0f);
    set_pwm_duty(MOTOR_RL_PWM_PIN, 0.0f);
    set_pwm_duty(MOTOR_FR_PWM_PIN, 0.0f);
    set_pwm_duty(MOTOR_RR_PWM_PIN, 0.0f);
    set_motor_modus(m, m);   // BRAKE = 1/1 on all 8 IN pins, COAST = 0/0
}

//=================================================================================
//Translation functions using local struct MotorModus to initiate braking/coasting.
//=================================================================================

void motor_stop(void) {
    halt_motors(MODUS_COAST);
}

void motor_brake(void) {
    gpio_put(MOTOR_STBY_PIN, 1);   // Driver must be enabled for short brake
    halt_motors(MODUS_BRAKE);
}

void motor_shutdown(void) {
    halt_motors(MODUS_COAST);
    gpio_put(MOTOR_STBY_PIN, 0);   // Drivers in standby: all outputs high-impedance
}
