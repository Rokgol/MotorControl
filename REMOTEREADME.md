# RemoteControllerModule

Joystick remote controller firmware for the **Raspberry Pi Pico 2 W**. It reads a two-axis analog joystick, converts the stick position into X/Y velocity references, and sends them over Wi-Fi as UDP packets to a receiver Pico that runs a Wi-Fi access point.

## How it works

1. Measures the joystick's resting (center) voltage on each axis at power-up. **Don't touch the stick while the board boots.**
2. Joins the Wi-Fi network `Pico2W_Net` in station mode with a **static IP** (`192.168.4.2`). DHCP is turned off because the access point doesn't run a DHCP server.
3. Every ~50 ms:
   - Reads both joystick axes with the ADC. It discards the first conversion after each channel switch, then averages 16 samples.
   - Converts each voltage to a velocity in the range `[-V_MAX, +V_MAX]` (default `V_MAX = 0.44`).
   - Sends a UDP packet to `192.168.4.1:4444`.
4. Prints telemetry to USB serial about every 250 ms.
5. Blinks the onboard LED (500 ms toggle) as a heartbeat once connected.

### UDP payload

Plain ASCII, no terminator:

| Condition                    | Payload           | Example         |
|------------------------------|-------------------|-----------------|
| Normal                       | `<vx>,<vy>` (3 dp)| `0.214,-0.050`  |
| Joystick button pressed      | `STOP`            | `STOP`          |

The joystick push button works as an **emergency stop**. While it's held, only `STOP` is sent.

### Voltage-to-velocity mapping

With a 3.3 V supply, a centered joystick reads about 1.6 V. `c` is the center voltage measured at boot:

| Voltage                      | Output                                           |
|------------------------------|--------------------------------------------------|
| `c ± DEADZONE_V` (±0.10 V)   | `0.0` (dead zone)                                |
| above the dead zone          | scaled linearly from 0 up to `+V_MAX` at 3.3 V   |
| below the dead zone          | scaled linearly from 0 down to `-V_MAX` at 0 V   |

The dead zone has to be wider than the joystick's spring-return error. If it isn't, the stick can come to rest just outside it and keep sending a small velocity after you let go.

Both axes reading `-V_MAX` with the stick centered means the joystick sees ~0 V. Usually its VCC pin isn't connected to 3V3(OUT).

## Hardware

| Signal            | Pico 2 W pin       | Notes                          |
|-------------------|--------------------|--------------------------------|
| Joystick X        | GPIO 27 (ADC1)     | `X_PIN`                        |
| Joystick Y        | GPIO 26 (ADC0)     | `Y_PIN`                        |
| Joystick button   | GPIO 22            | Active low, internal pull-up   |
| Joystick VCC      | 3V3 (OUT)          |                                |
| Joystick GND      | GND                |                                |

## Configuration

All settings are `#define`s at the top of [RemoteControllerModule.c](RemoteControllerModule.c):

| Define                      | Default         | Purpose                                   |
|-----------------------------|-----------------|-------------------------------------------|
| `AP_SSID` / `AP_PASS`       | `Pico2W_Net` / *(set in source)* | Credentials of the receiver's AP |
| `UDP_PORT`                  | `4444`          | Destination port on the receiver          |
| `BUTTON_PIN`, `X_PIN`, `Y_PIN` | 22, 27, 26   | GPIO assignments                          |
| `X_ADC_CHAN`, `Y_ADC_CHAN`  | 1, 0            | ADC channels read for each axis           |
| `DEADZONE_V`                | 0.10 V          | Dead-zone half-width around the calibrated center |
| `V_MAX`                     | 0.44            | Maximum velocity magnitude sent           |
| `INVERT_X` / `INVERT_Y`     | `false`         | Flip an axis if it's mounted reversed     |

The static IP (`192.168.4.2`), gateway, and target IP (`192.168.4.1`) are set in `main()`. Change them there if the receiver uses a different subnet.

## Building

Requirements:
- [Raspberry Pi Pico VS Code extension](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico) (recommended). It installs the right SDK and toolchain automatically.
- Or install these yourself: Pico SDK **2.3.1**, Arm GNU toolchain **15.2.Rel1**, picotool **2.3.1**, CMake ≥ 3.13, and Ninja.

**With VS Code:** open the folder, then run **Compile Project** from the Pico extension.

**From the command line:**

```sh
mkdir build && cd build
cmake -G Ninja -DPICO_BOARD=pico2_w ..
ninja
```

The output is `build/RemoteControllerModule.uf2`. There are also `.elf`, `.bin`, and `.hex` files.

## Flashing

- **UF2:** hold **BOOTSEL** while plugging in the board, then copy `RemoteControllerModule.uf2` onto the `RP2350` drive that appears.
- **picotool:** `picotool load build/RemoteControllerModule.uf2 -fx`
- **VS Code:** use **Run Project** or the debug configuration in `.vscode/launch.json`.

## Serial monitor

stdio goes over **USB CDC** (UART is turned off). Open the Pico's COM port at any baud rate to see output like:

```
Connecting to AP 'Pico2W_Net'...
Connected! Local IP: 192.168.4.2
Joy ADC: X=1.62V (vx=+0.000) | Y=2.40V (vy=+0.300) -> UDP: 0.000,0.300
```

## Troubleshooting

| Symptom                             | Likely cause                                              |
|-------------------------------------|-----------------------------------------------------------|
| `Failed to join network`            | Receiver AP isn't running, or the SSID/password is wrong  |
| Connects but the receiver gets nothing | Receiver isn't at `192.168.4.1` or isn't listening on UDP 4444 |
| Stick at rest shows nonzero velocity | Stick was touched during boot calibration (restart it hands-off), or increase `DEADZONE_V` |
| Axis moves the wrong way            | Set `INVERT_X` / `INVERT_Y`                               |
| No serial output                    | Board isn't enumerating over USB; check the cable and COM port |

## Project layout

```
RemoteControllerModule.c   Main firmware
CMakeLists.txt             Build configuration (board: pico2_w)
lwipopts.h                 lwIP network stack options
pico_sdk_import.cmake      Locates the Pico SDK
.vscode/                   Pico VS Code extension settings
```
