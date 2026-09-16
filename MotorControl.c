#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/timer.h"

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

// Actual motor-control PWM
#define MOTOR_PWM_PIN       15

// Direction pins for Driver IC (H-bridge)
#define MOTOR_DIR_PIN0      13
#define MOTOR_DIR_PIN1      14

// Simulated encoder / velocity output
#define SENSOR_PWM_PIN      1

// ADC input for simulated sensor
#define SENSOR_ADC_GPIO     26       // ADC0
#define SENSOR_ADC_CHANNEL  0


// ============================================================
// CONTROL PARAMETERS
// ============================================================

#define TS                  0.0001f   // 100 us sample period = 10 kHz
#define TS_US               100U      // 100 us in discrete integer microseconds

#define V_REF               3.0f      // desired velocity [m/s]

#define KP                  1.0f
#define ALPHA               0.0625f

#define CONTROL_LIMIT       1.0f

#define REVERSE_DEAD_TIME_TICKS 100U   // 5 ms deadtime = 50 ticks of 100 us


// ============================================================
// VEHICLE / ENCODER MODEL
// ============================================================

#define MASS                1.0f
#define WHEEL_RADIUS        0.025f

#define ENCODER_MARKS       24

#define MU0                 0.89f
#define TAU0                3.0f

#define MU_TIRE             0.89f
#define G                   9.81f

#define CRR                 0.015f
#define RHO                 1.225f
#define CDA                 0.03f


// Distance between encoder marks (single precision)
static const float DX_MARK =
    2.0f * (float)M_PI * WHEEL_RADIUS / (float)ENCODER_MARKS;

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
static uint32_t brake_until_ticks = 0;


// ============================================================
// MOTOR MODEL
// ============================================================

static float motor_acceleration(float c)
{
    if (fabsf(c) < 1e-12f)
        return 0.0f;

    float magnitude =
        (4.0f * MU0 / MASS) *
        sqrtf(fabsf(c) * TAU0);

    return copysignf(magnitude, c);
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
// SIMULATED SENSOR PWM
//
// Encode velocity into duty:
//      0 m/s -> 0%
//      5 m/s -> 100%
// ============================================================

#define SENSOR_MAX_VELOCITY 5.0f

static void output_simulated_sensor(float v)
{
    float duty = v / SENSOR_MAX_VELOCITY;

    duty = fmaxf(0.0f, fminf(1.0f, duty));

    set_pwm_duty(SENSOR_PWM_PIN, duty);
}


// ============================================================
// ADC READ
// ============================================================

static float read_sensor_velocity(void)
{
    adc_select_input(SENSOR_ADC_CHANNEL);

    uint16_t raw = adc_read();

    float voltage =
        ((float)raw / 4095.0f) * 3.3f;

    float duty =
        voltage / 3.3f;

    float v =
        duty * SENSOR_MAX_VELOCITY;

    return v;
}


// ============================================================
// ENCODER SIMULATION
// ============================================================

static void simulate_encoder(float t)
{
    while (position >= next_mark_position)
    {
        if (previous_mark_time >= 0.0f)
        {
            float delta_t =
                t - previous_mark_time;

            if (delta_t > 0.0f)
            {
                encoder_velocity =
                    DX_MARK / delta_t;
            }
        }

        previous_mark_time = t;

        next_mark_position += DX_MARK;
    }
}


// ============================================================
// PLANT UPDATE
// ============================================================

static void update_plant(float c)
{
    float a_drive =
        motor_acceleration(c);

    float F_drive =
        MASS * a_drive;

    if (F_drive > F_MAX)
        F_drive = F_MAX;

    if (F_drive < -F_MAX)
        F_drive = -F_MAX;

    float F_roll = 0.0f;

    if (fabsf(velocity) > 1e-10f)
    {
        F_roll =
            CRR *
            MASS *
            G *
            copysignf(1.0f, velocity);
    }

    float F_drag =
        0.5f *
        RHO *
        CDA *
        velocity *
        fabsf(velocity);

    float F_net =
        F_drive -
        F_roll -
        F_drag;

    acceleration =
        F_net / MASS;

    velocity += acceleration * TS;
    position += velocity * TS;
}


// ============================================================
// RP CONTROLLER (Optimized Tick-Based Deadtime)
// ============================================================

static float controller_update(float measured_velocity, uint32_t current_ticks)
{
    /*
     * Error
     */
    float e =
        V_REF - measured_velocity;

    /*
     * C(z): c[k] = KP * e[k] + ALPHA * c[k-1]
     */
    float c =
        KP * e +
        ALPHA * c_previous;

    /*
     * Saturation
     */
    c = fmaxf(
        -CONTROL_LIMIT,
        fminf(CONTROL_LIMIT, c));

    /*
     * Requested direction
     */
    int requested_direction = 0;

    if (c > 0.0f)
        requested_direction = +1;
    else if (c < 0.0f)
        requested_direction = -1;

    /*
     * --------------------------------------------------------
     * TICK-BASED REVERSE DEAD-TIME
     * --------------------------------------------------------
     */
    if (current_ticks < brake_until_ticks)
    {
        c = 0.0f;
    }
    else if (
        requested_direction != 0 &&
        motor_direction != 0 &&
        requested_direction != motor_direction
    )
    {
        motor_direction = 0;
        c = 0.0f;
        brake_until_ticks = current_ticks + REVERSE_DEAD_TIME_TICKS;
    }
    else if (requested_direction != 0)
    {
        motor_direction = requested_direction;
    }

    /*
     * Output direction state to Driver IC
     */
    if (motor_direction > 0)
    {
        gpio_put(MOTOR_DIR_PIN0, 1);
        gpio_put(MOTOR_DIR_PIN1, 0);
        modus = MODUS_FORWARD;
    }
    else if (motor_direction < 0)
    {
        gpio_put(MOTOR_DIR_PIN0, 0);
        gpio_put(MOTOR_DIR_PIN1, 1);
        modus = MODUS_REVERSE;
    }
    else
    {
        gpio_put(MOTOR_DIR_PIN0, 0);
        gpio_put(MOTOR_DIR_PIN1, 0);
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

static void hardware_init(void)
{
    stdio_init_all();

    setup_pwm(MOTOR_PWM_PIN);
    setup_pwm(SENSOR_PWM_PIN);

    gpio_init(MOTOR_DIR_PIN0);
    gpio_set_dir(MOTOR_DIR_PIN0, GPIO_OUT);

    gpio_init(MOTOR_DIR_PIN1);
    gpio_set_dir(MOTOR_DIR_PIN1, GPIO_OUT);

    adc_init();
    adc_gpio_init(SENSOR_ADC_GPIO);
    adc_select_input(SENSOR_ADC_CHANNEL);

    next_mark_position = DX_MARK;
}


// ============================================================
// MAIN
// ============================================================

int main(void)
{
    hardware_init();

    printf("\n");
    printf("=====================================\n");
    printf(" Pico Closed-Loop Controller (RP2350 Optimized)\n");
    printf("=====================================\n");
    printf("Ts             = %.6f s\n", TS);
    printf("Reference      = %.3f m/s\n", V_REF);
    printf("Encoder marks  = %d\n", ENCODER_MARKS);
    printf("DX mark        = %.6f m\n", DX_MARK);
    printf("Deadtime Ticks = %u (%u us)\n", REVERSE_DEAD_TIME_TICKS, REVERSE_DEAD_TIME_TICKS * TS_US);
    printf("\n");

    uint32_t sample_ticks = 0;
    absolute_time_t next_sample = make_timeout_time_us(TS_US);

    sleep_ms(10000);

    while (true)
    {
        sample_ticks++;

        /*
         * 1. SIMULATED PLANT
         */
        update_plant(c_previous);

        /*
         * 2. SIMULATED ENCODER
         */
        float current_time = (float)sample_ticks * TS;
        simulate_encoder(current_time);

        /*
         * 3. SIMULATED SENSOR OUTPUT
         */
        output_simulated_sensor(encoder_velocity);

        /*
         * 4. CONTROLLER (Tick-Based Deadtime)
         */
        float c = controller_update(encoder_velocity, sample_ticks);

        /*
         * 5. MOTOR PWM
         */
        float duty = fabsf(c);
        

        set_pwm_duty(MOTOR_PWM_PIN, duty);

        /*
         * Debug Output
         */
        static uint32_t counter = 0;
        //if (++counter >= 500)
        {
            counter = 0;
            printf(
                "v_true=%+.3f  "
                "v_enc=%+.3f  "
                "c=%+.3f  "
                "mode=%d  "
                "ticks=%u\n",
                velocity,
                encoder_velocity,
                c,
                (int)modus,
                sample_ticks
            );
        }

        /*
         * 6. WAIT FOR NEXT CONTROL SAMPLE (100 us = 10 kHz)
         */
        if (modus == MODUS_BRAKE)
        {
            next_sample = delayed_by_us(next_sample, 25U);
        }

        sleep_until(next_sample);
        next_sample = delayed_by_us(next_sample, TS_US);
    }
}