# Self-righting

The robot can get up by itself after a fall.

## How it works

**Lying down.** The robot rests on its hoops (the egg-shaped outer rings) at about ±66–70°, with both wheels in the air. So it can't simply drive back up, and one hard spin of the wheels isn't enough either.

**The shallow dip.** It lies in a shallow dip. Rolling on the hoop from the resting angle to where the tyres touch the ground (about 36°) raises the CoM by only ~1.6 mm, about 12 mJ.

**Rocking.** The robot spins its wheels back and forth as reaction wheels. Every spin-up tips the body the opposite way, and the direction always follows the way the body is swinging. Each swing therefore goes a little higher.

**Handover.** After two or three swings (about 1.3 s) the tyres touch the ground. Once it's inside the handover angle and still moving towards upright, the normal balance controller takes over and drives the robot up the rest of the way.

It works from either side.

## Starting it

There are three ways, and each also stops it:
- **Web page:** the **Self-right** button on the Control tab.
- **Xbox controller:** the **X** button.
- **CAN module:** set bit 2 of the flags byte in its drive frame (see `can-modules.md`).

With **Self-righting = 2** (Tune tab), it also gets up by itself once it has been lying still for the start delay. After giving up, or after being stopped, it waits until it has been moved (stood up, or turned over) before trying by itself again.

**Pause** stops it, as do a low battery and a motor driver dropping out.

The Control tab shows **self-righting** while it rocks. The OLED shows **SELF-RIGHT**. The Xbox controller buzzes twice when it starts.

## Before the first time: the push test

The firmware's choice of push direction comes from the sign conventions. It hasn't been tried on the robot yet. To check it:
1. Lay the robot on its hoop.
2. Press **Push once** on the Drivers tab (Self-righting push test).

Both wheels get the rocking current for the push time, in the direction that should rock the robot **towards upright**, and it's recorded.

If it rocks further over instead, set **Flip rocking direction** (`srInvert`) to 1. With the direction wrong, the rocking dies out instead of building, and each attempt just times out.

## Settings (Tune tab, Self-righting)

| Setting | Default | |
|---|---|---|
| Self-righting (`selfRight`) | 1 | 0 off, 1 on request, 2 also automatic |
| Rocking current (`srCurrent`) | 1.5 A | per wheel |
| Rocking wheel speed cap (`srWheelCap`) | 200 rad/s | no more push in the direction the wheels already spin this fast. Lower is quieter but slower (100 takes about twice as long) |
| Handover angle (`srHandover`) | 34° | the balance controller takes over inside this (always ≥ 2° inside the cut-out angle) |
| Time per attempt (`srTimeout`) | 6 s | |
| Attempts (`srTries`) | 3 | falling over again within 2 s of the handover also counts as a failed attempt |
| Automatic start delay (`srDelay`) | 2 s | |
| Flip rocking direction (`srInvert`) | 0 | see the push test |
| Record self-righting (`srRecord`) | 1 | records each self-righting (the attempt time + 4 s) |

`tools/tuning/self_right.py` simulates all of this, so changes can be tried there first. See the tuning README.

## In the recording

- **`state`:** `righting` while it rocks, then `balancing` after the handover.
- **Currents:** `cur_l` / `cur_r` are the rocking currents.
- **What to compare with the model:** the swings in `tilt` and the wheel speeds.
- **The log:** the device log has a line for each attempt, the handover (angle, rate, wheel speed, time) and the result.
