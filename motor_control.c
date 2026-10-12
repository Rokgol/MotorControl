#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "encoder_period.pio.h"
#include "pico/low_power.h"
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

#define SENSOR_PWM_PIN      1
#define ENCODER_PIN         18

#define SENSOR_ADC_GPIO     26       
#define SENSOR_ADC_CHANNEL  0

#define MOTOR_STBY_PIN      16

// ============================================================
// CONTROL PARAMETERS
// ============================================================
#define TS                  0.0001f   // 100 us sample period = 10 kHz
#define TS_US               100U      

#define KP                  1.0f
#define ALPHA               0.625f

#define CONTROL_LIMIT       1.0f
#define V_MAX               0.663f  

#define REVERSE_DEAD_TIME_TICKS 100U   

#define MASS                0.4f
#define WHEEL_RADIUS        0.03f
#define ENCODER_MARKS       20
#define MU_TIRE             0.89f
#define G                   9.81f

// Distance between encoder marks
static const float DX_MARK = 2.0f * (float)M_PI * WHEEL_RADIUS / (float)ENCODER_MARKS;

// ============================================================
// STATE VARIABLES
// ============================================================
static float v_ref_x = 0.0f;
static float v_ref_y = 0.44f; 
static float v_ref_abs = 0.0f;
static float v_ref_angle = 0.0f;

static MotorModus modus = MODUS_COAST;
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
static uint32_t counter = 0;

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
        float delta_t = (float)(pulse_iterations * 2) * 1e-6f;
        if (delta_t > 0.0f) {
            last_measured_speed = DX_MARK / delta_t;
            last_pulse_ticks = current_ticks;
        }
    } else if ((current_ticks - last_pulse_ticks) > 2000U) {
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
    pwm_config_set_clkdiv(&config, 125.0f);
    pwm_config_set_wrap(&config, 1000);
    pwm_init(slice, &config, true);
}

static void set_pwm_duty(uint gpio, float duty) {
    duty = fmaxf(0.0f, fminf(1.0f, duty));
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    uint level = (uint)(duty * 1000.0f);
    pwm_set_chan_level(slice, channel, level);
}

// ============================================================
// PID CONTROLLER
// ============================================================
static float PI_controller_update(float measured_velocity, float v_ref, uint32_t current_ticks) {
    float e = v_ref - measured_velocity; 

    integral_Controller += ALPHA * TS * e;
    integral_Controller = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, integral_Controller));
    float c = KP * e + integral_Controller;

    c = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, c)); 

    int requested_direction = 0;
    if (v_ref_y > 0.0f)
        requested_direction = +1;
    else if (v_ref_y < 0.0f)
        requested_direction = -1;

    if (fabsf(c) >= CONTROL_LIMIT / 2.0f && copysignf(1.0f, c) != copysignf(1.0f, c_previous)) {
        gpio_put(MOTOR_L_DIR_PIN0, 1);
        gpio_put(MOTOR_L_DIR_PIN1, 1);
        gpio_put(MOTOR_R_DIR_PIN0, 1);
        gpio_put(MOTOR_R_DIR_PIN1, 1);

        c_previous = c;
        modus = MODUS_BRAKE;
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

    if (motor_direction > 0) {
        gpio_put(MOTOR_L_DIR_PIN0, 1);
        gpio_put(MOTOR_L_DIR_PIN1, 0);
        gpio_put(MOTOR_R_DIR_PIN0, 1);
        gpio_put(MOTOR_R_DIR_PIN1, 0);
        modus = MODUS_FORWARD;
    } else if (motor_direction < 0) {
        gpio_put(MOTOR_L_DIR_PIN0, 0);
        gpio_put(MOTOR_L_DIR_PIN1, 1);
        gpio_put(MOTOR_R_DIR_PIN0, 0);
        gpio_put(MOTOR_R_DIR_PIN1, 1);
        modus = MODUS_REVERSE;
    } else {
        gpio_put(MOTOR_L_DIR_PIN0, 0);
        gpio_put(MOTOR_L_DIR_PIN1, 0);
        gpio_put(MOTOR_R_DIR_PIN0, 0);
        gpio_put(MOTOR_R_DIR_PIN1, 0);
        modus = MODUS_COAST;
    }

    c_previous = c;
    return c;
}

// ============================================================
// Free Proportional Controller
// ============================================================
static float free_controller_update(uint32_t current_ticks){

    float c = KP * v_ref_abs / V_MAX;

    c = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, c)); 

    int requested_direction = 0;
    if (v_ref_y > 0.0f)
        requested_direction = +1;
    else if (v_ref_y < 0.0f)
        requested_direction = -1;

    if (fabsf(c) >= CONTROL_LIMIT / 2.0f && copysignf(1.0f, c) != copysignf(1.0f, c_previous)) {
        gpio_put(MOTOR_L_DIR_PIN0, 1);
        gpio_put(MOTOR_L_DIR_PIN1, 1);
        gpio_put(MOTOR_R_DIR_PIN0, 1);
        gpio_put(MOTOR_R_DIR_PIN1, 1);

        c_previous = c;
        modus = MODUS_BRAKE;
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

    if (motor_direction > 0) {
        gpio_put(MOTOR_L_DIR_PIN0, 1);
        gpio_put(MOTOR_L_DIR_PIN1, 0);
        gpio_put(MOTOR_R_DIR_PIN0, 1);
        gpio_put(MOTOR_R_DIR_PIN1, 0);
        modus = MODUS_FORWARD;
    } else if (motor_direction < 0) {
        gpio_put(MOTOR_L_DIR_PIN0, 0);
        gpio_put(MOTOR_L_DIR_PIN1, 1);
        gpio_put(MOTOR_R_DIR_PIN0, 0);
        gpio_put(MOTOR_R_DIR_PIN1, 1);
        modus = MODUS_REVERSE;
    } else {
        gpio_put(MOTOR_L_DIR_PIN0, 0);
        gpio_put(MOTOR_L_DIR_PIN1, 0);
        gpio_put(MOTOR_R_DIR_PIN0, 0);
        gpio_put(MOTOR_R_DIR_PIN1, 0);
        modus = MODUS_COAST;
}
}


// ============================================================
// PUBLIC API
// ============================================================

void motor_init(void) {
    setup_pwm(MOTOR_L_PWM_PIN);
    setup_pwm(MOTOR_R_PWM_PIN);
    setup_pwm(SENSOR_PWM_PIN);

    gpio_init(MOTOR_L_DIR_PIN0);
    gpio_set_dir(MOTOR_L_DIR_PIN0, GPIO_OUT);
    gpio_init(MOTOR_L_DIR_PIN1);
    gpio_set_dir(MOTOR_L_DIR_PIN1, GPIO_OUT);
    
    gpio_init(MOTOR_R_DIR_PIN0);
    gpio_set_dir(MOTOR_R_DIR_PIN0, GPIO_OUT);
    gpio_init(MOTOR_R_DIR_PIN1);
    gpio_set_dir(MOTOR_R_DIR_PIN1, GPIO_OUT);

    gpio_init(MOTOR_STBY_PIN);
    gpio_set_dir(MOTOR_STBY_PIN, GPIO_OUT);
    gpio_put(MOTOR_STBY_PIN, 1);

    adc_init();
    adc_gpio_init(SENSOR_ADC_GPIO);
    adc_select_input(SENSOR_ADC_CHANNEL);

    encoder_sm = pio_claim_unused_sm(encoder_pio, true);
    uint offset = pio_add_program(encoder_pio, &encoder_period_program);
    encoder_period_program_init(encoder_pio, encoder_sm, offset, ENCODER_PIN);

    motor_direction = copysignf(1.0f, v_ref_x);
    last_active_direction = motor_direction;
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
        c_left = free_controller_update(sample_ticks);
        if (v_ref_x > 0) {
            c_right = c_left * (v_ref_x / (v_ref_abs > 0.001f ? v_ref_abs : 1.0f));
        } else if (v_ref_x < 0) {
            c_right = c_left * fabsf(v_ref_abs / (v_ref_x != 0.0f ? v_ref_x : 1.0f));
        }
    }
    float duty_L = c_left;
    float duty_R = c_right; // In MotorControl.c, duty_R was set to c_left right before setting pwm, which seemed like a bug, but I'll replicate the differential steering properly.

    set_pwm_duty(MOTOR_L_PWM_PIN, duty_L);
    set_pwm_duty(MOTOR_R_PWM_PIN, duty_R);

    if (++counter >= 10000) {
        counter = 0;
        printf("v_enc=%+.3f  c_left=%+.3f  c_right=%+.3f\n", measured_v, duty_L, duty_R);
    }
}

void motor_shutdown(void) {
    gpio_put(MOTOR_STBY_PIN, 0);
    set_pwm_duty(MOTOR_L_PWM_PIN, 0.0f);
    set_pwm_duty(MOTOR_R_PWM_PIN, 0.0f);

    low_power_pstate_until_aon_timer(at_the_end_of_time, NULL, NULL);
    while (true) tight_loop_contents();
}

