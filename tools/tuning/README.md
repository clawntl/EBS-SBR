# Balance tuning tools

Python tools for working out the balance controller's gains from the robot's physical properties and recordings, and for checking how well a set of gains works. They're meant for when the robot changes: a new battery, different wheels or motors, a taller body, extra payload. They're also useful for tuning from scratch.

The approach: describe the robot (mass, CoM, inertia from CAD), measure the motors with a spin test, design gains with a model, then check them against recordings of the real robot being pushed.

| File | What it does |
|---|---|
| `robot.json` | The robot's description: the only file you edit. The values in it are EBS-SBR's (November 2026) |
| `fusion/ListBodyMasses/` | Fusion script: lists every body's material, mass and centre of mass, to find wrong materials and duplicate parts before trusting the CAD's mass properties |
| `fit_spin.py` | Motor torque constant, friction and wheel-speed lag from a wheel spin test capture |
| `design_gains.py` | Designs K0, K1, K3, K4 and maxPosErr, and compares them with the gains you're running now |
| `check_log.py` | Summarises a balancing capture: jitter at rest, recovery from pushes, why it fell |
| `fit_nudge.py` | Cross-checks the body inertia and CoM height (and Kt) against balancing captures |
| `self_right.py` | Simulates self-righting (rocking up on the hoop, then the handover to balancing) for given settings; `hoop_outline.csv` is the hoop it rolls on |
| `sbr_model.py` | The model all of the above share. `python sbr_model.py robot.json` prints the robot as the model sees it |

## Setup

Python 3.9 or later:

```
cd tools/tuning
python -m pip install -r requirements.txt
```

Captures are the CSV files from the web page (Tune tab, Record, Download). Every capture records the controller settings in its header, so the tools read the gains that were in use from the file itself.

## Workflow

### 1. Describe the robot (`robot.json`)

Edit `robot.json`. Each section has a `_note` explaining it.

- **`mass_total_kg`**: weigh the whole robot, battery included.
- **`wheel_radius_m`**: half the tyre's outside diameter. **`track_m`**: tyre centre to centre (not used by the pitch model, kept for reference).
- **`rotating_per_side`**: everything that spins with one wheel, i.e. wheel, tyre, screws and the motor's rotor (bell + magnets). In Fusion, right-click each part, choose Properties, and take *Moment of Inertia at Center of Mass* about the axle axis. The parts sit on the axle, so their values just add. If you weighed the real wheel, scale its CAD inertia by real mass / CAD mass. EBS-SBR: wheel 9045 g·mm² × 22.28/20.512 + rotor 4584 g·mm² = 14 410 g·mm² = 1.441e-5 kg·m².
- **`cad`**: the whole robot's Fusion Properties, wheels included, with the design's origin on the axle centre:
  - `mass_g`: *Mass*;
  - `com_height_mm`: the *Center of Mass* component along "up" (z for EBS-SBR);
  - `inertia_axle_gmm2`: *Moment of Inertia at Origin* about the axle axis (ixx for EBS-SBR, whose axle is along x).

  The tools work out the body's own CoM height and inertia from these, and scale the CAD to the weighed mass. If your origin isn't on the axle, fill in **`body`** instead (body CoM height above the axle, body inertia about its own CoM) and the `cad` block is ignored.
- **`motor`**: the resistance **per phase**. Datasheets often give it line-to-line, which is twice as much: the GM3506's 5.6 Ω means 2.8 Ω here. `kt_nm_per_a` and `friction_nm` come from step 2.
- **`current_loop`**: the drivers' current loop P and I, as set on the Drivers tab (BLDC-Driver firmware 1.4+), and how much voltage the drivers can put on the motor. That depends on the driver firmware and the battery:
  - **Firmware 1.5+:** `"modulation": "svpwm"`, `"voltage_limit_v": 12`, `"max_duty": 0.92`. The motor gets up to 0.92 × supply / √3.
  - **Firmware 1.4 and older:** `"modulation": "sine"`, `"voltage_limit_v": 10`. The motor gets only 5 V.
  - **`supply_v`:** the battery voltage to model. For a 3S Li-ion pack that's 12.6 V full, about 11.1 V for most of a charge and 9.6 V at the cutoff.

  `python sbr_model.py robot.json` prints the resulting motor voltage and up to what wheel speed full current is available.
- **`control`**:
  - `rate_hz`: the `imuRate` setting;
  - `max_current_a`: the `maxCurrent` setting;
  - `wheel_speed_filter_s`: from step 2;
  - `delay_steps`: leave at 1;
  - `balance_priority`: the `balPriority` setting (simulated pushes then limit the position/speed terms as the firmware does).

**Check the CAD mass first.** It should be within a few percent of the weighed robot. If it isn't, run `fusion/ListBodyMasses` in Fusion:
1. Utilities → Add-Ins → Scripts and Add-Ins → Create, then Script (Python), named ListBodyMasses.
2. Replace its `.py` with the one here, and run it.

It shows the totals by material and the heaviest bodies, and writes the full list to `~/Desktop/fusion_body_masses.csv`. Common causes of a wrong mass:
- parts left as Fusion's default **Steel** (imported STEP parts, PCBs, connectors);
- a part included twice (EBS-SBR had a second battery model inside an imported assembly);
- 3D-printed parts modelled as solid. Give them a density of the slicer's filament weight ÷ the part's CAD volume.

`python sbr_model.py robot.json` prints what the model makes of it, including how fast the robot falls over and the wheel speed above which the motors can no longer get full current.

### 2. Measure the motors (`fit_spin.py`)

On the web page, Drivers tab, Wheel spin test:
1. Lay the robot down so both wheels spin freely.
2. Use **0.3 A, 100 ms** (at 0.1 A friction and cogging swamp the result).
3. Download the capture.

```
python fit_spin.py robot.json BalanceBot-xxxx-capture-179.csv
python fit_spin.py robot.json old-test.csv --cl-i 300     # recorded with a different current loop I
```

It fits each wheel's speed with the motor + current-loop model, and prints `kt_nm_per_a`, `friction_nm` and `wheel_speed_filter_s` for robot.json. The model includes the motors' voltage limit, worked out from `current_loop` in robot.json and the battery voltage in the capture's header, so long steps that reach top speed fit too.

**Driver firmware 1.6+.** The capture also has the currents the drivers measured (`iq_l`, `iq_r`, `id_l`, `id_r`), and the script fits Kt a second way, straight from them: wheel inertia × acceleration = Kt × measured current − friction. This needs no current-loop model. It also prints how the measured current compares with the command at low wheel speed (about 1.0 if the current loop does what it's told). For this a longer test works best, e.g. **0.5 A, 1000 ms**: hold or prop the robot so it can't rock on its hoop.

- **Kt** is the torque per *commanded* amp, so it includes any current-sensing scale error. That's what the controller needs.
- **Re-run the test after changing the current loop.** EBS-SBR measured 0.048 at I = 300 and 0.054 at I = 3000, even with the loop modelled.

EBS-SBR, current loop I = 3000: Kt ≈ 0.054 N·m/A, friction ≈ 3 mN·m, speed lag ≈ 12 ms.

### 3. Design gains (`design_gains.py`)

```
python design_gains.py robot.json
python design_gains.py robot.json --compare capture.csv --compare=-0.15,-28,-0.18,-1.5,3
```

The design picks the four firmware gains that minimise an LQR cost. It runs at the real control rate and includes the drivers' current loop, the wheel-speed lag and a control delay. The CoM height is usually the least certain number, so the design is made robust over ±30% of it (`--com-spread`), and every check is printed at all three heights.

**Each gain set is printed with:**

- **damping**: the lowest damping ratio of its oscillations. Near 0 it keeps ringing (the "wobble"), 0.3 or more is good, 0.5 or more is calm. Its frequency is given too.
- **slowest mode**: roughly how long it takes to settle back onto its spot.
- **margin** (min |1 + L|): distance from instability at the motors. 1 is very robust; under about 0.3, a small change to the robot could make it oscillate.
- **pushes**: simulated pushes (a sudden tilt rate), including the current limit and the drivers' voltage limit. Prints the peak tilt, peak current, drift and settling time. A simulated push is an ideal kick, not a hand, so the real robot leans further. Use these to compare gain sets, not as a prediction.

Two designs are printed:
- **Gentle** asks for up to about 0.5 A per motor in a typical push.
- **Firm** asks for up to about 0.75 A.

Change these with `--gentle` / `--firm`. `--max-tilt` and `--max-pos` set how much lean and wheel travel the design tolerates. Put negative numbers after `=` (`--compare=-0.15,...`); otherwise the command line reads them as options.

**maxPosErr** is suggested so that K0 × maxPosErr ≤ 0.5 A. When a push moves the robot further than that, the position hold drags its reference along instead of building up a large current.

Enter the gains on the Tune tab, then go to step 4. Change one set at a time.

### 4. Check on the robot (`check_log.py`)

Record 20–30 s on the Tune tab while you push the robot a few times, gently and harder, with a few seconds of standing still in between. Then:

```
python check_log.py capture.csv --robot robot.json
python check_log.py old.csv new.csv --robot robot.json     # compare captures
```

| Section | What it reports | What to look for |
|---|---|---|
| At rest | Tilt-rate jitter and its main frequency; current noise | Lower is better. A strong single frequency means a lightly damped oscillation |
| Position hold | How often it's at maxPosErr; what set point error the position hold is making up for | An offset over about 0.3° means the set point or learned trim is off: the robot leans and the position term holds it |
| Pushes | Per push: how hard (peak tilt rate), how far it leaned, settling time, time at the current limit; then a summary | Hardest push recovered, and how long it took |
| Balance priority, lifted wheels | How often `balPriority` limited the position/speed terms; each lifted/slipping wheel detection (firmware with these columns) | Lifted-wheel detections while driving straight on a flat floor mean `liftSpeed` is too low, or the wheel diameter/track settings are wrong |
| Falls | Time at the current limit and wheel speed just before | Wheels above the motors' full-current speed mean the push was more than the motors could catch |
| Model | Prediction for the capture's gains (with `--robot`) | Frequency should be near the measured one at rest (`--cl-i` if the capture used a different current loop) |

### 5. Optional: cross-check the model (`fit_nudge.py`)

```
python fit_nudge.py robot.json new.csv old.csv@300 --free-com --free-kt
```

Simulates the robot + controller from the measured state through the captures, and fits the body inertia and the IMU's tilt offset. With `--free-com` and `--free-kt` it fits the CoM height and Kt too. A capture recorded with a different current loop I gets it after `@`.

It prints:
- how much worse the robot.json values fit than the fitted ones;
- for each value, whether the captures actually pin it down.

While balancing, the controller hides most of gravity's effect, so the CoM height and inertia are usually only loosely determined. Treat this as a sanity check of the CAD, not a replacement. With four EBS-SBR captures, the fit wanted a CoM of 53 mm against the CAD's 63 mm and fitted 9% better. That's within the noise, so the CAD values stayed.

### 6. Self-righting (`self_right.py`)

```
python self_right.py                          # both sides, the firmware's default settings
python self_right.py --current 1.0 --wheel-cap 150
python self_right.py --sweep                  # rocking current x wheel cap x Kt x tyre grip
python self_right.py --side back --csv run.csv
```

Simulates the firmware's self-righting (the `Righting` state; settings on the Tune tab under Self-righting, and `docs/self-righting.md`):

1. **Lying on the hoop:** the wheels are in the air. The script spins them as reaction wheels, with the reaction torque on the body in the direction it's swinging, so each swing goes higher.
2. **Handover:** once the robot is inside the handover angle and moving towards upright, the balance controller takes over with the gains given (`--gains K0 K1 K3 K4`, default the current set). It drives the robot up the rest of the way.
3. **Tyres:** they can slip (`--mu`), and the robot falls back onto the hoop if it doesn't make it.

The motors and drivers are modelled as in the other tools.

It also prints what the geometry implies:
- the resting angle on the hoop;
- the tilt at which the tyres touch;
- how much higher the CoM is there.

The hoop comes from `hoop_outline.csv`: the outer ring's outline in the wheel plane, origin on the axle. To remake it from a STEP of the ring, positioned like `Ring.step` (origin on the axle, −x up), run `python self_right.py --step Ring.step` (needs `pip install cadquery-ocp`).

**Check against the robot.** The resting angle it prints should match the tilt the robot reads lying down. EBS-SBR: the model gives ±65.7°, measured +66.3° / −69.8°. That agreement also confirms the CAD CoM height: with a 33 mm CoM it would rest at 36° on the hoop's ends.

**Not modelled:**
- bounces of the hoop or tyres;
- the hoop slipping;
- the two hoops not lining up;
- turning.

The push direction is the one thing the model can't check. If it's wrong, the rocking dies out and the attempt times out; in the model it never gets close. Use the push test on the Drivers tab first.

## The model, briefly

- **The robot:** a body (everything that doesn't spin with the wheels) pivoting on the axle, plus two wheels rolling without slipping. Only pitch and forward motion are modelled; turning (K2, K5) isn't.
- **The motors:** torque = Kt × current, minus Coulomb friction.
- **The drivers:** SimpleFOC current control. A PI loop on a motor of resistance R (per phase), with back-EMF Ke = Kt / 1.5 and the q-axis voltage limit (see `current_loop` above). This is why the current lags the command, and why full current isn't available at high wheel speed.
- **The controller:** the firmware's law (`firmware/src/LQRControl.cpp`): current = −(K0·pos_err + K1·(tilt − set point) + K3·wheel speed + K4·tilt rate), in total for both wheels, clamped to maxCurrent per wheel. With `balPriority`, the K0/K3 part is limited to 1.5 × the total current limit, so after a hard push (wheels spinning fast) the K1/K4 part wins. It sees the wheel speed through the reported-speed lag and acts one control step late.
- **Gains:** in the firmware's units (A/rad, A/(rad/s), total current).
- **Signs:** a positive tilt leans the robot the way a positive wheel speed rolls it.

**Known limits:**
- The model's oscillation frequency has come out about 1 Hz below the measured one, and it's slightly pessimistic about damping.
- Pushes from a hand last longer than the simulated kicks.
- The inertia and CoM height can only be roughly checked from balancing data.

## Example: EBS-SBR, November 2026

| | Gains (K0, K1, K3, K4, maxPosErr) | Result |
|---|---|---|
| Original | −0.05, −25, −0.08, −0.6, 10 | Wobble at ~6 Hz; pos_err pinned at its limit, so it crept around |
| Step 1 | −0.15, −28, −0.18, −1.5, 3 | Back on its spot in ~1.3 s; leaned ~7° for a ~80°/s push |
| Step 2 | −0.23, −20, −0.24, −2.0, 2 | Settles in ~1 s; leans further (~12° for ~95°/s) |
| Current loop | drivers' current loop I raised from 300 to 3000 | ~50% more torque reaching the wheels in the spin test; a third less jitter at rest |

Falls only came from pushes of 450–600°/s, which drove the wheels past the ~45 rad/s above which the drivers can't push full current through the motors.
