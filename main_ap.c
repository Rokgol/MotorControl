#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/udp.h"
#include "motor_control.h"

#define AP_SSID "Pico2W_Net"
#define AP_PASS "password123"
#define UDP_PORT 4444

// State Machine Enum
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_FAULT
} AppState;

static AppState current_state = STATE_INIT;

// Hardware timer callback for 10 kHz motor loop
bool motor_timer_callback(struct repeating_timer *t) {
    if (current_state == STATE_RUNNING) {
        motor_control_tick();
    }
    return true; // Keep repeating
}

// UDP Receive Callback
static void udp_recv_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                              const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        printf("Received %d bytes from %s:%d -> %.*s\n", 
               p->len, ipaddr_ntoa(addr), port, p->len, (char *)p->payload);
        
        // Parse UDP payload to extract velocity references (vx, vy)
        float vx = 0.0f, vy = 0.0f;
        if (sscanf((char *)p->payload, "%f,%f", &vx, &vy) == 2) {
            motor_set_velocity_reference(vx, vy);
            current_state = STATE_RUNNING; 
            printf("Set motor reference: vx=%.3f, vy=%.3f\n", vx, vy);
        } else {
            // If the command is stop
            if (p->len >= 4 && strncmp((char*)p->payload, "STOP", 4) == 0) {
                current_state = STATE_IDLE;
                motor_set_velocity_reference(0.0f, 0.0f);
                printf("Motor stopped via UDP command.\n");
            }
        }
        
        pbuf_free(p);
    }
}

int main() {
    stdio_init_all();
    
    // 1. Initialize Wi-Fi
    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return 1;
    }
    
    // Enable Access Point Mode (Default IP is 192.168.4.1)
    cyw43_arch_enable_ap_mode(AP_SSID, AP_PASS, CYW43_AUTH_WPA2_AES_PSK);
    printf("AP active | SSID: %s | IP: 192.168.4.1\n", AP_SSID);

    // 2. Initialize Motor Hardware
    motor_init();

    // 3. Setup UDP
    struct udp_pcb *pcb = udp_new();
    if (!pcb) {
        printf("Failed to create UDP PCB\n");
        return 1;
    }
    cyw43_arch_lwip_begin();
    udp_bind(pcb, IP_ADDR_ANY, UDP_PORT);
    udp_recv(pcb, udp_recv_callback, NULL);
    cyw43_arch_lwip_end();
    
    printf("Listening on UDP port %d...\n", UDP_PORT);

    // 4. Start 10kHz Hardware Timer for Motor Control
    struct repeating_timer timer;
    add_repeating_timer_us(-100, motor_timer_callback, NULL, &timer);

    current_state = STATE_IDLE;

    // 5. General State Machine Loop
    while (true) {
        switch(current_state) {
            case STATE_IDLE:
                // Wait for commands
                break;
            case STATE_RUNNING:
                // Monitor timeouts or faults
                break;
            case STATE_FAULT:
                motor_shutdown();
                break;
            default:
                break;
        }
        
        // Heartbeat LED
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(500);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(500);
    }
    return 0;
}
