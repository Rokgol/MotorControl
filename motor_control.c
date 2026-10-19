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
#define MOTOR_L_PWM_PIN     15
#define MOTOR_R_PWM_PIN     17

#define MOTOR_L_DIR_PIN0    13
#define MOTOR_L_DIR_PIN1    14
#define MOTOR_R_DIR_PIN0    11
#define MOTOR_R_DIR_PIN1    12

#define ENCODER_PIN         18
#define MOTOR_STBY_PIN      16

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
// Drives IN1/IN2 of both H-bridge channels:
//   FORWARD = 1/0, REVERSE = 0/1, COAST = 0/0, BRAKE = 1/1
static void set_motor_modus(MotorModus m) {
    bool in1 = (m == MODUS_FORWARD || m == MODUS_BRAKE);
    bool in2 = (m == MODUS_REVERSE || m == MODUS_BRAKE);

    gpio_put(MOTOR_L_DIR_PIN0, in1);
    gpio_put(MOTOR_L_DIR_PIN1, in2);
    gpio_put(MOTOR_R_DIR_PIN0, in2);    //Direction flipped in physical motor connection,
    gpio_put(MOTOR_R_DIR_PIN1, in1);    //So this is way easier to fix in code.
}

// ============================================================
// DIRECTION LOGIC (shared by both controllers)
// ============================================================
// Brakes on a hard sign change of the control output, inserts a dead time
// before reversing, and sets the direction pins from the sign of dir_ref.
// Returns the (possibly zeroed) control output.
static float apply_direction_logic(float c, float dir_ref, uint32_t current_ticks) {
    int requested_direction = 0;
    if (dir_ref > 0.0f)
        requested_direction = +1;
    else if (dir_ref < 0.0f)
        requested_direction = -1;

    if (fabsf(c) >= CONTROL_LIMIT / 2.0f && copysignf(1.0f, c) != copysignf(1.0f, c_previous)) {
        set_motor_modus(MODUS_BRAKE);
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
        set_motor_modus(MODUS_FORWARD);
    else if (motor_direction < 0)
        set_motor_modus(MODUS_REVERSE);
    else
        set_motor_modus(MODUS_COAST);

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

    // A pure turn (vy == 0) pivots forward around the stopped inner wheel
    float v_dir = (v_ref_y != 0.0f) ? v_ref_y : 1.0f;

    // The PI's feedforward picks up from here, so its integral starts fresh
    integral_Controller = 0.0f;

    return apply_direction_logic(c, v_dir, current_ticks);
}


// ============================================================
// PUBLIC API
// ============================================================

void motor_init(void) {
    setup_pwm(MOTOR_L_PWM_PIN);
    setup_pwm(MOTOR_R_PWM_PIN);

    const uint out_pins[] = {
        MOTOR_L_DIR_PIN0, MOTOR_L_DIR_PIN1,
        MOTOR_R_DIR_PIN0, MOTOR_R_DIR_PIN1,
        MOTOR_STBY_PIN
    };
    for (uint i = 0; i < count_of(out_pins); i++) {
        gpio_init(out_pins[i]);
        gpio_set_dir(out_pins[i], GPIO_OUT);
    }
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
    if(v_ref_x == 0){
        c_left = PI_controller_update(measured_v, v_ref_y, sample_ticks);
        c_right = c_left;
    }
    else{
        // Inner wheel runs at the forward share of the command: 1 = straight, 0 = pivot
        float k = fabsf(v_ref_y) / (v_ref_abs > 0.001f ? v_ref_abs : 1.0f);
        if (v_ref_x > 0) {
            c_left = free_controller_update(sample_ticks);
            c_right = c_left * k;
        } else {
            c_right = free_controller_update(sample_ticks);
            c_left = c_right * k;
        }
    }

    set_pwm_duty(MOTOR_L_PWM_PIN, c_left);
    set_pwm_duty(MOTOR_R_PWM_PIN, c_right);

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

    set_pwm_duty(MOTOR_L_PWM_PIN, 0.0f);
    set_pwm_duty(MOTOR_R_PWM_PIN, 0.0f);
    set_motor_modus(m);
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
