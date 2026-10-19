#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "motor_control.h"
#include "pico/critical_section.h"

#define AP_SSID "Pico2W_Net"
#define AP_PASS "RA15612?!"
#define UDP_PORT 4444

#define CMD_TIMEOUT_MS 1000   // Stop the car if no command arrives for this long
#define TELEMETRY_INTERVAL_MS 1000
#define RX_BUF_SIZE 64

// State Machine Enum
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_BRAKING
} AppState;

// Must be volatile since it's modified in callbacks and read in interrupts/main loop
static volatile AppState current_state = STATE_INIT;
static volatile uint32_t last_cmd_ms = 0;

critical_section_t motor_crit_sec;

// One-slot mailbox: the UDP callback (Wi-Fi IRQ context) only copies the
// payload here; the main loop parses it. This keeps sscanf/printf out of
// interrupt context. A newer packet overwrites an unprocessed older one.
static char rx_buf[RX_BUF_SIZE];
static volatile bool rx_pending = false;


// Hardware timer callback for 10 kHz motor loop
bool motor_timer_callback(struct repeating_timer *t) {
    if (current_state == STATE_RUNNING) {
        motor_control_tick();
    }
    return true; // Keep repeating
}

// Stop the motors and return to IDLE
static void enter_idle(void) {
    critical_section_enter_blocking(&motor_crit_sec);
        current_state = STATE_IDLE;
        motor_stop();
    critical_section_exit(&motor_crit_sec);
}

// Short-brake the motors and hold until the next velocity command
static void enter_braking(void) {
    critical_section_enter_blocking(&motor_crit_sec);
        current_state = STATE_BRAKING;
        motor_brake();
    critical_section_exit(&motor_crit_sec);
}

// UDP Receive Callback (runs in Wi-Fi IRQ context: copy only, no parsing)
static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                              const ip_addr_t *addr, u16_t port) {
    if (p == NULL) return;

    critical_section_enter_blocking(&motor_crit_sec);
        u16_t len = pbuf_copy_partial(p, rx_buf, RX_BUF_SIZE - 1, 0);
        rx_buf[len] = '\0';
        rx_pending = true;
    critical_section_exit(&motor_crit_sec);

    pbuf_free(p);
}

// Parse and apply a command received over UDP (main loop context)
static void handle_command(const char *cmd) {
    float vx = 0.0f, vy = 0.0f;
    if (sscanf(cmd, "%f,%f", &vx, &vy) == 2) {
        critical_section_enter_blocking(&motor_crit_sec);
            motor_set_velocity_reference(vx, vy);
            last_cmd_ms = to_ms_since_boot(get_absolute_time());
            current_state = STATE_RUNNING;
        critical_section_exit(&motor_crit_sec);
    } else if (strncmp(cmd, "STOP", 4) == 0) {
        enter_braking();
    }
}

int main() {
    stdio_init_all();

    // 1. Initialize Wi-Fi
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }

    // Enable Access Point Mode
    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASS, CYW43_AUTH_WPA2_AES_PSK);
    printf("AP active | SSID: %s | IP: 192.168.4.1\n", AP_SSID);

    // Disable Wi-Fi power saving so packets are handled without wake-up latency
    if (cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM) != 0) {
        printf("Failed to disable Wi-Fi power saving\n");
    }

    // 2. Initialize Motor Hardware & Add Critical Section
    motor_init();
    critical_section_init(&motor_crit_sec);

    // 3. Setup UDP
    // All lwIP calls must be made while holding the lwIP lock
    cyw43_arch_lwip_begin();
    struct udp_pcb *pcb = udp_new();
    err_t bind_err = pcb ? udp_bind(pcb, IP_ADDR_ANY, UDP_PORT) : ERR_MEM;
    if (bind_err == ERR_OK) {
        udp_recv(pcb, udp_recv_callback, NULL);
    }
    cyw43_arch_lwip_end();

    if (bind_err != ERR_OK) {
        printf("Failed to set up UDP on port %d (err %d)\n", UDP_PORT, bind_err);
        return 1;
    }

    printf("Listening on UDP port %d...\n", UDP_PORT);

    // 4. Start 10kHz Hardware Timer for Motor Control
    struct repeating_timer timer;
    add_repeating_timer_us(-100, motor_timer_callback, NULL, &timer);

    current_state = STATE_IDLE;

    // Variables for non-blocking LED toggle
    absolute_time_t next_led_toggle = delayed_by_ms(get_absolute_time(), 500);
    bool led_state = false;
    absolute_time_t next_telemetry = delayed_by_ms(get_absolute_time(), TELEMETRY_INTERVAL_MS);

    // 5. Main Loop
    while (true) {
        // Handle the latest UDP command, if any
        if (rx_pending) {
            char cmd[RX_BUF_SIZE];
            critical_section_enter_blocking(&motor_crit_sec);
                memcpy(cmd, rx_buf, sizeof(cmd));
                rx_pending = false;
            critical_section_exit(&motor_crit_sec);
            handle_command(cmd);
        }

        // Command timeout: stop if the controller goes silent
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (current_state == STATE_RUNNING && (now_ms - last_cmd_ms) > CMD_TIMEOUT_MS) {
            printf("Command timeout, stopping\n");
            enter_idle();
        }

        // Telemetry (printed here, never from the 10 kHz ISR)
        if (time_reached(next_telemetry)) {
            next_telemetry = delayed_by_ms(get_absolute_time(), TELEMETRY_INTERVAL_MS);
            if (current_state == STATE_RUNNING) {
                float v, cl, cr;
                motor_get_telemetry(&v, &cl, &cr);
                printf("v_enc=%+.3f  c_left=%+.3f  c_right=%+.3f\n", v, cl, cr);
            }
        }

        // Non-blocking Heartbeat LED
        if (time_reached(next_led_toggle)) {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            next_led_toggle = delayed_by_ms(get_absolute_time(), 500);
        }

        sleep_ms(1);
    }
    return 0;
}
