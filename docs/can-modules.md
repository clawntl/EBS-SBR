# CAN modules

Other boards on the robot's CAN bus can drive it, for example:
- a Raspberry Pi with a camera;
- a line-following sensor board;
- anything else with a CAN transceiver.

The robot keeps balancing itself. A module only asks for a **speed** and a **turn rate**, exactly like the joystick does.

- **Bus:** classic CAN 2.0, standard 11-bit IDs, **1 Mbit/s**, 87.5 % sample point. This is the same bus as the motor drivers; see `BLDC-Driver/docs/can-protocol.md`.
- **Byte order:** big-endian (most significant byte first).

## Who drives

The robot picks one source on every control step:

1. **The web page's joystick or the paired Xbox controller,** whenever its stick is off-centre. If both are, the one moved most recently wins. Picking up a controller always takes over from a module.
2. **Otherwise, the lowest-numbered module** that is asking to drive and has sent a frame in the last **250 ms**.
3. **Otherwise, nothing.** The robot holds its position and heading.

Modules are only listened to while the **Allow CAN modules** setting (Tune tab, Driving) is on. It's on by default.

The web page shows who is driving ("Driving from" on the Control tab). Recordings have it in the `drive_src` column.

## Module → robot: drive request, ID `0x200 + m`

`m` is the module number, 0–15. A lower number means higher priority, both between modules and on the bus.

| Byte | Content |
|---|---|
| 0–1 | Speed: int16, **0.01 rad/s of wheel speed**, + = forward |
| 2–3 | Turn rate: int16, **0.1 deg/s**, + = turning right (clockwise seen from above) |
| 4 | Flags: bit 0 = **drive** (asking to drive), bit 1 = **fraction** (see below) |
| 5 | Sequence counter (optional, not checked) |

- **Send at least every 250 ms while driving.** 20–50 Hz is typical. If the frames stop, the robot stops being driven by this module within 250 ms.
- **To hand control back,** send frames with the drive flag clear (or just stop sending).
- **Limits:** values beyond the robot's **Top speed** / **Turn rate** settings are limited to them.
- **Fraction flag:** with bit 1 set, bytes 0–1 and 2–3 are fractions of the robot's top speed and turn rate instead, scaled by 32767. So 32767 = full speed forward and −16384 = half rate turning left. This is handy for a module that doesn't care about units.

Wheel speed is the motors' shaft speed. Ground speed = wheel speed × wheel radius.

## Robot → modules

### State, ID `0x180`, every 20 ms

| Byte | Content |
|---|---|
| 0 | Balance state: 0 = motors not ready, 1 = IMU stalled, 2 = paused, 3 = tipped / lying down, 4 = battery low, 5 = **balancing**, 6 = waiting to be stood up, 7 = stopped (wheels ran away) |
| 1 | Who's driving: bits 0–3 = 0 none, 1 web page, 2 Xbox controller, 3 module; bits 4–7 = the module number |
| 2–3 | Tilt from the set point: int16, 0.01°. Same sign as the web page's "Tilt from set point" |
| 4–5 | Wheel speed: int16, 0.01 rad/s, + = forward |
| 6–7 | Turn rate: int16, 0.1 deg/s, + = turning right |

### Info, ID `0x181`, every 500 ms

| Byte | Content |
|---|---|
| 0–1 | Battery: uint16, mV (0 = no reading) |
| 2 | Battery %: 0–100, 255 = unknown |
| 3 | Flags: bit 0 = paused, bit 1 = battery low, bit 2 = modules allowed |
| 4–5 | Top speed setting: uint16, 0.01 rad/s |
| 6–7 | Turn rate setting: uint16, 0.1 deg/s |

A module should only drive while byte 0 of `0x180` is 5 (balancing), and should expect to be overridden at any time (byte 1).

## IDs on the bus

| IDs | Use |
|---|---|
| `0x010–0x017` | Motor current commands (robot → drivers) |
| `0x021–0x02F` | Driver commands |
| `0x101–0x10F` | Driver telemetry |
| `0x180`, `0x181` | Robot state for modules |
| `0x200–0x20F` | Module drive requests |

Module boards should use IDs in `0x200–0x2FF` for anything else of their own, and keep their total bus load modest. The motor current frames already use about 20 % of the bus.

## Example (Python, `python-can`, e.g. on a Raspberry Pi with a CAN HAT)

```python
import struct
import time

import can

MODULE = 0                      # 0-15
DRIVE, FRACTION = 0x01, 0x02

bus = can.Bus(interface="socketcan", channel="can0")  # ip link set can0 up type can bitrate 1000000

def drive(speed_rad_s, turn_deg_s, active=True):
    data = struct.pack(">hhBB", round(speed_rad_s * 100), round(turn_deg_s * 10), DRIVE if active else 0, 0)
    bus.send(can.Message(arbitration_id=0x200 + MODULE, data=data, is_extended_id=False))

balancing = False
t_end = time.time() + 5
while time.time() < t_end:      # 5 s: drive forward at 5 rad/s while turning gently right
    msg = bus.recv(timeout=0.02)
    if msg and msg.arbitration_id == 0x180:
        state, source, tilt, speed, turn = struct.unpack(">BBhhh", msg.data)
        balancing = state == 5
    drive(5.0 if balancing else 0.0, 20.0 if balancing else 0.0, active=balancing)
    time.sleep(0.02)
drive(0, 0, active=False)       # hand control back
```
