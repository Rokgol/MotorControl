#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"
#include "hardware/pio.h"
#include "pico/cyw43_arch.h"
#include "encoder_period.pio.h"
#include "pico/low_power.h"


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

// Actual motor-control PWMs
#define MOTOR_L_PWM_PIN     15
#define MOTOR_R_PWM_PIN     17

// Direction pins for Driver IC (H-bridge)
#define MOTOR_L_DIR_PIN0    13
#define MOTOR_L_DIR_PIN1    14
#define MOTOR_R_DIR_PIN0    11
#define MOTOR_R_DIR_PIN1    12


// Simulated encoder / velocity output
#define SENSOR_PWM_PIN      1

// Real Physical Optical Encoder Input Pin
#define ENCODER_PIN         18

// ADC input for simulated sensor
#define SENSOR_ADC_GPIO     26       // ADC0
#define SENSOR_ADC_CHANNEL  0

//STBY Pin
#define MOTOR_STBY_PIN      16


// ============================================================
// CONTROL PARAMETERS
// ============================================================

#define TS                  0.0001f   // 100 us sample period = 10 kHz
#define TS_US               100U      // 100 us in discrete integer microseconds

#define KP                  1.0f
#define ALPHA               0.625f

#define CONTROL_LIMIT       1.0f
#define V_MAX               0.663f  // Maximum velocity per direction. 
                                    // Maximum abs velocity: 0.44 ~ (0.663)^2

#define REVERSE_DEAD_TIME_TICKS 100U   // 5 ms deadtime = 50 ticks of 100 us


// ============================================================
// VEHICLE / ENCODER MODEL
// ============================================================

#define MASS                0.4f
#define WHEEL_RADIUS        0.03f

#define ENCODER_MARKS       20

#define MU0                 0.89f
#define TAU0                8e-3f

#define MU_TIRE             0.89f
#define G                   9.81f

#define CRR                 0.015f
#define RHO                 1.225f
#define CDA                 0.03f

//===============================================
//v_ref directions (should be inputted from pico)
//===============================================
// Velocity reference vectors (can be updated via remote control)
static float v_ref_x = 0.0f;
static float v_ref_y = 0.44; //Total Velocity Should be kept under 0.44
static float v_ref_abs = 0.0f;
static float v_ref_angle = 0.0f;




// Distance between encoder marks (single precision)
static const float DX_MARK = 2.0f * (float)M_PI * WHEEL_RADIUS / (float)ENCODER_MARKS;

// Maximum tire force
static const float F_MAX =
    MU_TIRE * MASS * G;


// ============================================================
// SIMULATED PLANT STATE
// ============================================================

static float position = 0.0f;
static float velocity = 0.0f;
static float acceleration = 0.0f;

static MotorModus modus = MODUS_COAST;

// ============================================================
// SIMULATED ENCODER STATE
// ============================================================

static float next_mark_position;
static float previous_mark_time = -1.0f;

static float encoder_velocity = 0.0f;


// ============================================================
// CONTROLLER STATE
// ============================================================

static float c_previous = 0.0f;

static int motor_direction = 0;
static int last_active_direction = 1;
static uint32_t brake_until_ticks = 0;
static float integral_Controller = 0.0f;

// ============================================================
// PIO ENCODER READER
// ============================================================

static PIO encoder_pio = pio1;
static int encoder_sm = -1;
static float last_measured_speed = 0.0f;
static uint32_t last_pulse_ticks = 0;

static float read_pio_encoder_velocity(uint32_t current_ticks) {
    bool new_pulse = false;
    uint32_t pulse_iterations = 0;

    // Drain FIFO to get the latest period measurement
    while (!pio_sm_is_rx_fifo_empty(encoder_pio, encoder_sm)) {
        pulse_iterations = pio_sm_get(encoder_pio, encoder_sm);
        new_pulse = true;
    }

    if (new_pulse && pulse_iterations > 0) {
        // PIO SM clock runs at 1 MHz (1 us/cycle)
        // Each loop iteration takes 2 PIO clock cycles (2 us)
        float delta_t = (float)(pulse_iterations * 2) * 1e-6f;
        if (delta_t > 0.0f) {
            last_measured_speed = DX_MARK / delta_t;
            last_pulse_ticks = current_ticks;
        }
    }

    // Timeout check: If no pulse arrived in 0.20s (2000 ticks), wheel is stationary
    else if ((current_ticks - last_pulse_ticks) > 2000U) {
        last_measured_speed = 0.0f;
    }

    // Apply directional sign from motor state (Photo-interrupters only measure speed magnitude)
    float direction_sign = (motor_direction != 0) ? (float)motor_direction : 1.0f;
    
    return last_measured_speed * direction_sign;
}

// ============================================================
// PWM SETUP
// ============================================================

static void setup_pwm(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    uint slice = pwm_gpio_to_slice_num(gpio);

    pwm_config config = pwm_get_default_config();

    // 1 MHz PWM:
    // 125 MHz / 125 = 1 MHz
    pwm_config_set_clkdiv(&config, 125.0f);

    // 1000 counts -> 1 kHz PWM
    pwm_config_set_wrap(&config, 1000);

    pwm_init(slice, &config, true);
}


// ============================================================
// SET PWM DUTY
// duty = 0.0 ... 1.0
// ============================================================

static void set_pwm_duty(uint gpio, float duty)
{
    duty = fmaxf(0.0f, fminf(1.0f, duty));

    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);

    uint level = (uint)(duty * 1000.0f);

    pwm_set_chan_level(slice, channel, level);
}

// ============================================================
// RP CONTROLLER (Optimized Tick-Based Deadtime)
// ============================================================
//How should we make controller updates per direction? 
//We calculate the controller via the absolute velocity, 
//then in main we will configure PWM'ing via sine and cosine to the absolute
//of C.
static float controller_update(float measured_velocity, float v_ref, uint32_t current_ticks)
{
    float e = v_ref - measured_velocity; //Error Signal

    integral_Controller += ALPHA * TS * e;
    integral_Controller = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, integral_Controller));
    float c = KP * e + integral_Controller;  // C(z): c[k] = KP * e[k] + ALPHA * TS * c[k-1]

    c = fmaxf(-CONTROL_LIMIT, fminf(CONTROL_LIMIT, c)); // Saturation

    int requested_direction = 0;

    //deciding direction of wheel motion, forwards or backwards.
    if (v_ref_y > 0.0f)
        requested_direction = +1;
    else if (v_ref_y < 0.0f)
        requested_direction = -1;

    // Should include a signal from system for hard brake condition, but for now just check if the sign of c has changed and the magnitude is above half the control limit.
    // should be c_abs and a big enough c_angle (i.e change in direction).
    if (fabsf(c) >= CONTROL_LIMIT / 2.0f && copysignf(1.0f, c) != copysignf(1.0f, c_previous))  // Hard Brake condition
    {
        gpio_put(MOTOR_L_DIR_PIN0, 1);
        gpio_put(MOTOR_L_DIR_PIN1, 1);
        gpio_put(MOTOR_R_DIR_PIN0, 1);
        gpio_put(MOTOR_R_DIR_PIN1, 1);

        c_previous = c;
        modus = MODUS_BRAKE;
        return c;
    }
    /*
     * --------------------------------------------------------
     * TICK-BASED REVERSE DEAD-TIME
     * --------------------------------------------------------
     */ 

    if (current_ticks < brake_until_ticks)
    {
        c = 0.0f;
        motor_direction = 0; // ensure coasting during deadtime
    }
    else if (requested_direction != 0)
    {
        if (last_active_direction != 0 && requested_direction != last_active_direction)
        {
            // Direction reversal: trigger deadtime
            motor_direction = 0;
            c = 0.0f;
            brake_until_ticks = current_ticks + REVERSE_DEAD_TIME_TICKS;
            last_active_direction = requested_direction;
        }
        else
        {
            motor_direction = requested_direction;
            last_active_direction = requested_direction;
        }
    }
    else
    {
        motor_direction = 0; // coast
    }

    /*
     * Output direction state to Driver IC
     */
    if (motor_direction > 0)
    {
        gpio_put(MOTOR_L_DIR_PIN0, 1);
        gpio_put(MOTOR_L_DIR_PIN1, 0);
        gpio_put(MOTOR_R_DIR_PIN0, 1);
        gpio_put(MOTOR_R_DIR_PIN1, 0);
        modus = MODUS_FORWARD;
    }
    else if (motor_direction < 0)
    {
        gpio_put(MOTOR_L_DIR_PIN0, 0);
        gpio_put(MOTOR_L_DIR_PIN1, 1);
        gpio_put(MOTOR_R_DIR_PIN0, 0);
        gpio_put(MOTOR_R_DIR_PIN1, 1);
        modus = MODUS_REVERSE;
    }
    else
    {
        gpio_put(MOTOR_L_DIR_PIN0, 0);
        gpio_put(MOTOR_L_DIR_PIN1, 0);
        gpio_put(MOTOR_R_DIR_PIN0, 0);
        gpio_put(MOTOR_R_DIR_PIN1, 0);
        modus = MODUS_COAST;
    }

    /*
     * Controller memory
     */
    c_previous = c;

    return c;
}


// ============================================================
// INITIALIZATION
// ============================================================
//should add additional PWM (one for left side, one for right side)
static void hardware_init(void)
{
    stdio_init_all();

    cyw43_arch_init();
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

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

    next_mark_position = DX_MARK;
}


// ============================================================
// HARDWARE SHUTDOWN FUNCTION
// ============================================================
static void shutdown_pico(void)
{
    // Optional: make the motor controller explicitly safe first.
    gpio_put(MOTOR_STBY_PIN, 0);
    set_pwm_duty(MOTOR_L_PWM_PIN, 0.0f);
    set_pwm_duty(MOTOR_R_PWM_PIN, 0.0f);

    // Enter the lowest-power RP2350 state.
    // at_the_end_of_time is ~300,000 years away.
    //
    // NULL = keep the minimum persistent power domains alive.
    // NULL resume function = just restart normally if ever woken.
    int rc = low_power_pstate_until_aon_timer(
        at_the_end_of_time,
        NULL,
        NULL
    );

    // We should never get here under normal operation.
    printf("Pstate failed! rc=%d\n", rc);

    while (true) {
        tight_loop_contents();
    }
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    hardware_init();
    v_ref_abs = sqrtf(v_ref_x * v_ref_x + v_ref_y * v_ref_y);
    v_ref_angle = atan2f(v_ref_y, v_ref_x);
    float v_target_left = v_ref_y + v_ref_x;
    float v_target_right = v_ref_y - v_ref_x;
    float measured_v = 0.0f;
    float c_left = 0.0f;
    float c_right = 0.0f;
    float duty_R = 0.0f;
    float duty_L = 0.0f;

    // 10-second startup delay before starting control loop
    //sleep_ms(1000);

    uint32_t sample_ticks = 0;
    absolute_time_t next_sample = make_timeout_time_us(TS_US);
    static uint32_t counter = 0;

    while (sample_ticks < 100000)  // Run for 10 seconds at 10 kHz
    {
        gpio_put(MOTOR_STBY_PIN, 1);
        sample_ticks++;

        // 1. Update reference parameters (dynamically handle remote requests)

        // 2. Read Velocity directly from PIO hardware queue
        measured_v = read_pio_encoder_velocity(sample_ticks);

        // 3. Controller Update
        c_left = controller_update(measured_v, v_target_left, sample_ticks);

        // 4. Drive Motor PWM (Differential steering)
        c_right = c_left;

        if(v_ref_x > 0){
            c_left = c_left;
            c_right = c_right * (v_ref_x/v_ref_abs);
        }
        else if(v_ref_x < 0){
            c_left = c_left;
            c_right = c_right * fabsf(v_ref_abs/v_ref_x);
        }
        else{
            duty_R = c_left;
            duty_L = c_right;
        }
        duty_R = c_left;
        duty_L = c_right;

        set_pwm_duty(MOTOR_L_PWM_PIN, duty_L);
        set_pwm_duty(MOTOR_R_PWM_PIN, duty_R);

        // 5. Debug Output (Every 1s)
        if (++counter >= 10000)
        {
            counter = 0;
            printf(
                "v_enc=%+.3f  "
                "c=%+.3f  \n",
                measured_v,
                c_left
            );
        }
        sleep_until(next_sample);
        next_sample = delayed_by_us(next_sample, TS_US);
    }

    shutdown_pico();
}
