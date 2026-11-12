# EBS-SBR

A compact two-wheeled **self-balancing robot** with custom electronics: an ESP32-S3 controller board running an LQR balance loop, and two STM32G431 field-oriented-control (FOC) BLDC driver boards talking to it over CAN.

<p align="center">
  <img src="images/EBS-SBR_CAD_Image.png" alt="EBS-SBR CAD render" width="48%">
  <img src="images/robot.png" alt="Assembled EBS-SBR robot" width="40%">
</p>
<p align="center"><em>CAD render (left) and the assembled robot (right)</em></p>

## Architecture

```
            ┌──────────────────────────────┐
            │      Controller Board        │
            │  ESP32-S3 (Waveshare Zero)   │
            │  BMI160 IMU · LQR · OLED     │
            └──────────────┬───────────────┘
                           │ CAN 2.0B @ 1 Mbit/s
             ┌─────────────┴─────────────┐
   ┌─────────┴─────────┐       ┌─────────┴─────────┐
   │ Driver Board (L)  │       │ Driver Board (R)  │
   │ STM32G431CBU6     │       │ STM32G431CBU6     │
   │ DRV8316 · AS5048A │       │ DRV8316 · AS5048A │
   └─────────┬─────────┘       └─────────┬─────────┘
          BLDC motor                  BLDC motor
```

### Controller Board (`firmware/`)

- **MCU:** Waveshare ESP32-S3 Zero (Arduino framework via pioarduino)
- **IMU:** Bosch BMI160 over SPI, fused with a complementary filter (Madgwick AHRS also included)
- **Control:** full-state LQR (wheel displacement, tilt, yaw and their rates) running as FreeRTOS tasks. The LQR runs once per IMU sample and sends the motor currents straight away.
- **Motor driver supervision:** tracks each driver's heartbeat, starts FOC on the drivers one at a time, retries failed starts and restarts a driver that resets. The motors are only driven when both drivers are running.
- **Comms:** CAN via [ACAN-ESP32FD](https://github.com/clawntl/ACAN-ESP32FD), Wi-Fi web interface, status WS2812 LED, button and buzzer
- **Display:** optional 128×64 SSD1306/SSD1315 OLED (I²C, 0x3C) showing balance state, tilt, motor currents, driver status and faults
- Default rate: IMU and control loop at 500 Hz (motor currents are sent once per sample)

### Driver Boards

The motor drivers are the [**BLDC-Driver**](https://github.com/clawntl/BLDC-Driver) board, a compact 39 mm × 36 mm FOC driver. Its KiCad hardware, firmware and full documentation live in that repo.

> **Note:** this controller firmware needs the current BLDC-Driver firmware, which sends the `CMD_DRIVER_STATUS` heartbeat. Drivers running older firmware are never started.

- **MCU:** STM32G431CBU6 with hardware CORDIC acceleration
- **Power stage:** TI DRV8316 with integrated FETs (6-PWM), low-side current sensing on all three phases
- **Encoder:** AS5048A magnetic sensor over SPI
- **Control:** [SimpleFOC](https://simplefoc.com/) current (torque) control, 11 pole-pair motors on a 12 V supply
- **Comms:** FDCAN (classic CAN 2.0B, 1 Mbit/s), receiving target currents, sending shaft velocity every 10 ms and a status heartbeat every 100 ms
- **Setup:** one firmware build for both sides. The side (left/right) and direction invert are set over serial and saved to flash.
- **Flashing:** USB DFU (hold the DFU button, or trigger it over serial)

See the BLDC-Driver [hardware guide](https://github.com/clawntl/BLDC-Driver/blob/main/docs/hardware.md), [firmware guide](https://github.com/clawntl/BLDC-Driver/blob/main/docs/firmware.md) and [CAN protocol](https://github.com/clawntl/BLDC-Driver/blob/main/docs/can-protocol.md) for details.

## CAN protocol

| ID      | Direction             | Purpose                         |
|---------|-----------------------|---------------------------------|
| `0x020` | Controller → both     | Broadcast to both motor drivers |
| `0x021` | Controller → left     | Left motor driver               |
| `0x022` | Controller → right    | Right motor driver              |
| `0x100` | Right driver → ctrl   | Right driver velocity, status, replies and errors |
| `0x101` | Left driver → ctrl    | Left driver velocity, status, replies and errors  |

Command bytes: `0x02` shaft velocity, `0x03` driver status heartbeat, `0x10` start FOC (and the driver's reply), `0xFF` error. See the BLDC-Driver [CAN protocol](https://github.com/clawntl/BLDC-Driver/blob/main/docs/can-protocol.md) for the frame layouts.

## Building and flashing

Both firmware projects use [PlatformIO](https://platformio.org/).

**Controller board:** flash the firmware, then upload the web page to LittleFS. Repeat `uploadfs` whenever `firmware/data/` changes.

```bash
cd firmware
pio run -t upload
pio run -t uploadfs
```

**Driver boards:** build and flash from the BLDC-Driver repo. All boards use the same build. Each board is then set to left or right over its USB serial console:

```bash
cd Firmware          # in the BLDC-Driver repo
pio run -e driver -t upload
```

```text
set side left        # or right
set invert on|off    # optional
save
```

See the BLDC-Driver [firmware guide](https://github.com/clawntl/BLDC-Driver/blob/main/docs/firmware.md) for details.

## Web interface

The controller runs a Wi-Fi access point:

- **SSID:** `BalanceBot`
- **Password:** `balancebot`

Connect, then open the IP address printed on the serial console at boot (normally `http://192.168.4.1`). The page updates live over a WebSocket and provides:

- sliders for the LQR gains, angle set point (with a fine adjust), deactivation angle, filter coefficient and drive gains
- the IMU / control rate (applied after a reboot)
- pause / resume
- a touch joystick for driving, or a gamepad connected to the phone or computer viewing the page. The browser's Gamepad API needs Chrome or Edge here: Firefox only allows it on HTTPS pages.

Drive input is zeroed if the page stops sending for 300 ms.

## Tuning over serial

The controller board accepts commands at 115200 baud, one per line:

| Command      | Effect                                           |
|--------------|--------------------------------------------------|
| `SP <deg>`   | Balance angle set point (degrees from upright)   |
| `DA <deg>`   | Tilt angle at which the motors are deactivated   |
| `K0 <value>` | Wheel displacement gain                          |
| `K1 <value>` | Tilt angle gain                                  |
| `K3 <value>` | Wheel velocity gain                              |
| `K4 <value>` | Tilt velocity gain                               |
| `CA <value>` | Complementary filter coefficient                 |
| `MS`         | Print each motor driver's link state, flags, start attempts and last error |
| `MR`         | Reset start attempts so a driver that gave up is tried again |

Default gains live in [`firmware/src/RobotParams.cpp`](firmware/src/RobotParams.cpp).

## Repository layout

```
firmware/    Controller board (ESP32-S3) PlatformIO project
hardware/    Full robot CAD (CAD/EBS-SBR.step). Driver board hardware: see BLDC-Driver
images/      Renders and photos
```

## License

[GNU AGPL-3.0](LICENSE)
