# MotorControl

Firmware for a small four-wheeled car built on a **Raspberry Pi Pico 2 W** (RP2350). The Pico opens its own Wi-Fi access point, takes velocity commands over UDP, and drives four DC motors through 2 dual Toshiba TB6612FNG H-bridges. A 10 kHz control loop uses wheel-encoder feedback.

## Features

- **Wi-Fi access point.** No router needed: connect straight to the Pico. Wi-Fi power saving is disabled, so commands arrive without the radio's wake-up delay.
- **UDP command interface.** Plain-text `vx,vy` and `STOP` packets.
- **10 kHz control loop** driven by a hardware repeating timer.
- **PIO-based encoder period measurement**, with 2 µs resolution and no CPU polling.
- **Two control modes:**
  - *Straight driving* (`vx == 0`): closed-loop PI velocity control on encoder feedback.
  - *Turning* (`vx != 0`): open-loop, tank-style differential drive. The outer wheel gets full power and the inner wheel is slowed.
- **Manual brake.** A `STOP` command short-brakes both motors and holds the brake until the next velocity command.
- **Command timeout.** The motors coast to a stop if no command arrives for 1 s.
- **Direction-reversal protection.** The motors brake and wait through a dead time (10 ms) before they reverse.
- **Heartbeat LED.** The onboard LED blinks at 1 Hz while the main loop is running.

## Hardware

Each motor has its own PWM pin and IN1/IN2 pair (no parallel wiring). Left motors use GPIO 0–15 and right motors use GPIO 16–28, with each motor on three pins in a row (IN1, IN2, PWM):

| Side  | Motor            | IN1 | IN2 | PWM (slice) |
|-------|------------------|-----|-----|-------------|
| Left  | Front-left (FL)  | 13  | 14  | 15 (7B)     |
| Left  | Rear-left (RL)   | 10  | 11  | 12 (6A)     |
| Right | Front-right (FR) | 19  | 20  | 21 (2B)     |
| Right | Rear-right (RR)  | 26  | 27  | 22 (3A)     |

Toshiba TB6612FNG H-Bridge has AIN2 | AIN1 | STBY | BIN1 | BIN2,
essentially expecting opposite motor directions due to layout symmetry.
Since we also have layout anti-symmetry, this should work well for us, so we will connect IN1|IN2 -> AIN2|AIN1 -> + - which should parallel
IN1|IN2 -> BIN1|BIN2 -> - +.

GPIO 23–25 belong to the Wi-Fi chip, so RR uses 22 for PWM and 26/27 for IN1/IN2. Every PWM pin is on a different PWM slice/channel, so all four motors' duty cycles can be set independently.

| Function                | GPIO | Notes                                  |
|-------------------------|------|----------------------------------------|
| Driver standby (STBY)   | 16   | Shared by both drivers. High = enabled |
| Wheel encoder           | 18   | Input with pull-up, read by PIO1       |

All 8 direction pins are written with a single register write (`gpio_put_masked`), so every motor switches in the same cycle.

The pin scheme (IN1/IN2/PWM per channel plus STBY) matches a **TB6612FNG**-style dual H-bridge.

Direction truth table, identical for every motor. There are no software inversions: wire each motor so that IN1 = 1, IN2 = 0 drives its wheel forward. If a wheel runs backwards, swap its motor leads.

| IN1 | IN2 | Mode    |
|-----|-----|---------|
| 1   | 0   | Forward |
| 0   | 1   | Reverse |
| 0   | 0   | Coast   |
| 1   | 1   | Brake   |

Physical parameters live in [motor_control.c](motor_control.c): a 30 mm wheel radius and 20 encoder marks per revolution, which gives about 9.4 mm of travel per mark.

## Building and flashing

The project uses the **Pico SDK 2.3.1** and was generated with the [Raspberry Pi Pico VS Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico).

### With VS Code (recommended)
1. Install the Raspberry Pi Pico extension. It downloads the SDK, the toolchain and picotool to `~/.pico-sdk`.
2. Open this folder, then **Compile Project**.
3. Hold BOOTSEL, plug in the Pico, then **Run Project** (or copy `build/MotorControl.uf2` to the `RP2350` drive).

### From the command line
```sh
mkdir build && cd build
cmake .. -G Ninja        # PICO_BOARD defaults to pico2_w
ninja
picotool load MotorControl.uf2 -fx
```

Serial output goes over **USB CDC** (UART stdio is disabled).

## Usage

1. Power the Pico. It starts an access point:
   - **SSID:** `Pico2W_Net`
   - **Password:** see `AP_PASS` in `main_ap.c`
   - **Pico IP:** `192.168.4.1`
2. Connect your computer or phone to that network. The firmware does **not** run a DHCP server, so give your device a static IP such as `192.168.4.2`, netmask `255.255.255.0`.
3. Send UDP packets to `192.168.4.1:4444`.

### Command protocol

Commands are ASCII, one command per datagram, at most 63 bytes.

| Packet       | Effect                                                          |
|--------------|-----------------------------------------------------------------|
| `vx,vy`      | Set the velocity reference (floats, m/s) and enter `RUNNING`    |
| `STOP`       | Short-brake both motors and hold until the next `vx,vy` (`BRAKING`) |

- `vy`: forward/backward speed. Its sign selects the drive direction. The maximum is about 0.44 m/s (`V_MAX`, the measured speed at full duty).
- `vx`: steering component (positive = right). When it is non-zero, the controller switches to the open-loop turning mode.

**The client must keep sending commands.** If nothing arrives for `CMD_TIMEOUT_MS` (1 s, set in `main_ap.c`), the firmware cuts PWM and lets the motors coast (it does not brake). Resend the current command at about 10–20 Hz. The joystick remote sends at 20 Hz.

Examples:
```sh
# netcat
# A single packet drives for only 1 s before the timeout kicks in
echo -n "0,0.3"  | nc -u -w0 192.168.4.1 4444   # straight ahead at 0.3 m/s
echo -n "0.2,0.3"| nc -u -w0 192.168.4.1 4444   # forward while turning
echo -n "STOP"   | nc -u -w0 192.168.4.1 4444
```
```python
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
PICO = ("192.168.4.1", 4444)

end = time.time() + 3.0              # drive straight for 3 s
while time.time() < end:
    s.sendto(b"0,0.3", PICO)
    time.sleep(0.05)                 # 20 Hz keeps the timeout from firing
s.sendto(b"STOP", PICO)
```

While the car is running, the firmware prints a status line about once per second:
```
v_enc=+0.297  c_left=+0.412  c_right=+0.412
```

## Software architecture

```
main_ap.c          Wi-Fi AP, UDP server, app state machine, 10 kHz timer
motor_control.c/h  Encoder read-out, controllers, PWM + direction output
encoder_period.pio PIO program that measures time between encoder edges
lwipopts.h         lwIP configuration (UDP only, no TCP)
ancillary/         Older reference implementations (git-ignored)
```

### Application state machine (`main_ap.c`)

```
INIT ──► IDLE ◄────── 1 s timeout ──────┐
          │                            │
          └────────── "vx,vy" ──► RUNNING ◄──┐
                                       │     │ "vx,vy"
                                "STOP" └─► BRAKING
```

Entering `IDLE` calls `motor_stop()`, which cuts PWM, coasts the motors and resets the controller. Entering `BRAKING` calls `motor_brake()`, which does the same but short-brakes the motors (IN1 = IN2 = 1). The timeout does not apply in `BRAKING`, so the brake holds until a new velocity command arrives.

**`STOP` has priority.** The receive callback runs in the Wi-Fi interrupt and checks every packet for `STOP` first. On a match it brakes immediately, inside the callback, without waiting for the main loop. Braking only writes GPIO/PWM registers, so it is safe in an interrupt. It also throws away any velocity command still waiting in the mailbox, and a stop counter makes sure a command taken from the mailbox before the `STOP` cannot release the brake.

Velocity packets are handled in two steps. The receive callback only copies the payload into a one-slot mailbox. The main loop, which runs about every 1 ms, then parses the command and applies it. The status line is also printed from the main loop. As a result, no `printf` or `sscanf` runs inside an interrupt: USB output can block for up to 500 ms, and the C library's float routines are not safe to call from interrupts.

`motor_control_tick()` runs only in `RUNNING`. State changes and reference updates happen inside a critical section, so they cannot race with the timer interrupt.

### Velocity measurement

The PIO state machine runs at 1 MHz. It waits for a rising edge, then counts down X in a 2-cycle loop until the next rising edge, and pushes the elapsed count to the RX FIFO. Each tick, the CPU drains the FIFO and computes:

```
v = DX_MARK / (count × 2 µs)
```

If no pulse arrives for 200 ms, the speed is set to 0. The encoder is single-channel, so the speed sign comes from the commanded direction.

### Controller

Turning mode (tank-style, no steering axle):

```
outer duty = |v| / V_MAX                  |v| = sqrt(vx² + vy²)
inner duty = outer duty · |vy| / |v|      1 = straight, 0 = pivot on inner wheel (vy = 0: spin, see below)
```

The outer wheel is the left one for `vx > 0` and the right one for `vx < 0`. A pure turn (`vy = 0`) spins the car in place: both sides run at `|vx| / V_MAX`, with the inner side in reverse. If one side would flip directly between forward and reverse (for example arc → spin, or spin left → spin right), that side first coasts for 10 ms (`REVERSE_DEAD_TIME_TICKS`). The turning output equals the PI's feedforward term, and the integral is reset to 0 while turning. Switching back to straight driving therefore starts from the right duty for the new command, with no dip or surge.

PI law with feedforward (straight mode), sampled at `Ts = 100 µs`:

```
e  = v_ref − v_meas
I += α · Ts · e                     (clamped to ±I_LIMIT)
c  = v_ref / V_MAX + Kp · e + I     (clamped to ±1)

sign(c) → forward / reverse,   |c| → PWM duty
```

Tunable constants are at the top of [motor_control.c](motor_control.c):

| Constant                  | Value  | Meaning                               |
|---------------------------|--------|---------------------------------------|
| `KP`                      | 1.0    | Proportional gain                     |
| `ALPHA`                   | 3.0    | Integral gain (Ti = Kp/α ≈ 0.33 s)    |
| `I_LIMIT`                 | 0.5    | Integral clamp (trims the feedforward)|
| `V_MAX`                   | 0.44   | Measured speed at full duty (m/s)     |
| `CONTROL_LIMIT`           | 1.0    | Output saturation                     |
| `REVERSE_DEAD_TIME_TICKS` | 100    | Brake time before reversal (10 ms)    |

The PWM runs from `clk_sys / 125` with wrap 1000, which gives roughly 1.2 kHz at the RP2350's default 150 MHz clock.

## Known limitations / TODO

- Only one encoder is read. Both wheels share the same feedback.
- The Wi-Fi credentials are hard-coded in `main_ap.c`. Change them before you use the car anywhere public.
