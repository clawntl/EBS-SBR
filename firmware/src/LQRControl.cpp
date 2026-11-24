#include "LQRControl.h"

#include <Arduino.h>

#include "Battery.h"
#include "CANConfig.h"
#include "Capture.h"
#include "DeviceLog.h"
#include "CanBus.h"
#include "IMUDriver.h"
#include "MotorDrivers.h"
#include "RobotParams.h"
#include "DriveInput.h"
#include "Settings.h"

float leftTargetCurrent = 0;
float rightTargetCurrent = 0;


volatile BalanceState balanceState = BalanceState::MotorsNotReady;

namespace {
// RAD_TO_DEG is a double; the S3's FPU is single precision only.
constexpr float kRadToDeg = 57.2957795f;

portMUX_TYPE snapshotLock = portMUX_INITIALIZER_UNLOCKED;
ControlSample snapshot = {};
uint16_t measuredLoopHz = 0;

void publish(const ControlSample &s) {
  portENTER_CRITICAL(&snapshotLock);
  snapshot = s;
  portEXIT_CRITICAL(&snapshotLock);
}


// Set point trim learning (see learnTrim). Only while standing still: no
// drive input for TRIM_SETTLE_MS, balancing for at least that long and not
// pushed to the position-hold limit. The trim then integrates the lean
// (tilt minus set point) with time constant TRIM_TAU_S, i.e. it moves the
// set point towards the angle the robot actually balances at. Slow compared
// to the position hold's sway (2-5 s), so the two don't fight, and the sway
// itself averages out.
// Starting to balance. It only starts when it's near upright AND held still:
// within engageAngle of the set point, tilting slower than ENGAGE_MAX_RATE,
// wheels slower than ENGAGE_MAX_WHEEL, for ENGAGE_HOLD_MS. Once started it
// keeps going until it passes the cut-out angle (deactivationAngle).
// Without this it started the moment it was lifted past the cut-out angle,
// mid-swing with the wheels in the air, and the wheel-speed / position terms
// (which only stabilise with the wheels on the ground) spun the wheels up to
// full speed within ~0.2 s, too fast to recover once put down.
constexpr float ENGAGE_MAX_RATE = 30.0f * DEG_TO_RAD; // rad/s
constexpr float ENGAGE_MAX_WHEEL = 3.0f;              // rad/s
constexpr uint32_t ENGAGE_HOLD_MS = 300;
// Runaway: wheels faster than runawaySpeed (and 1.5x top speed) for this long
// while balancing -> stop. Catches being lifted while running, or a crash
// that leaves the wheels spinning free.
constexpr uint32_t RUNAWAY_MS = 300;
// A short IMU stall (e.g. a flash write) doesn't count as stopping; a longer
// one does, and it then has to be started again.
constexpr uint32_t STALL_DISENGAGE_MS = 200;

constexpr uint32_t TRIM_SETTLE_MS = 3000;

// Balance priority (balancePriority): the position/speed terms are limited to
// this multiple of the total current limit. See Task_LQR.
constexpr float HOLD_LIMIT_FACTOR = 1.5f;
// Lifted wheel (liftDetect): a wheel stays "lifted" until the wheels agree
// (mismatch under half the threshold) for this long.
constexpr uint32_t LIFT_CLEAR_MS = 100;
// maxSpeedLead compares the speed set point with the wheel speed filtered
// over this time constant, so single bumps don't move the set point around.
constexpr float SPEED_LEAD_FILTER_S = 0.05f;

// Wheel spin test (tuning, see requestSpinTest()). Set by the web task, run by
// Task_LQR.
volatile bool spinTestRequested = false;
volatile float spinTestAmps = 0;
volatile uint32_t spinTestPhaseMs = 0;
constexpr float TRIM_TAU_S = 20.0f;
constexpr float TRIM_LIMIT_RAD = 5.0f * DEG_TO_RAD;

// Self-righting (see requestSelfRight and the self-righting block in
// Task_LQR). Requests come from other tasks; Task_LQR acts on them.
volatile bool srStartRequested = false;
volatile bool srStopRequested = false;
volatile uint32_t srPushRequestMs = 0;   // push test length, 0 = none
volatile uint8_t srRecordSeconds = 0;    // recording wanted (serviceSelfRight starts it)
// The push direction follows the swing: it only changes once the tilt rate
// has passed this the other way, so gyro noise at the turning points doesn't
// make it chatter.
constexpr float SR_RATE_HYSTERESIS = 3.0f * DEG_TO_RAD;   // rad/s
// Between attempts: 0 A for at least this long, until the rocking has died
// down (tilt rate under SR_SETTLE_RATE), or SR_SETTLE_MAX_MS at most.
constexpr uint32_t SR_SETTLE_MS = 1500;
constexpr uint32_t SR_SETTLE_MAX_MS = 4000;
constexpr float SR_SETTLE_RATE = 20.0f * DEG_TO_RAD;
// After the handover, falling over again within this long counts as a
// failed attempt (and it tries again).
constexpr uint32_t SR_WATCH_MS = 2000;
// Automatic start: lying still means tilting slower than this, wheels
// slower than ENGAGE_MAX_WHEEL.
constexpr float SR_STILL_RATE = 20.0f * DEG_TO_RAD;
// After giving up (or being stopped) it doesn't start by itself again until
// it has been moved: stood up, or the tilt changed by this much.
constexpr float SR_MOVED_DEG = 20.0f;
// Beyond this it isn't lying on its hoop any more: stop.
constexpr float SR_MAX_TILT_DEG = 120.0f;

// If no IMU sample arrives for this long, stop driving the motors (0 A)
// rather than balancing on a stale tilt measurement. 10 ms = 5 missed
// samples at the default 500 Hz.
constexpr uint32_t IMU_SAMPLE_TIMEOUT_MS = 10;
} // namespace

const char *requestSpinTest(float amps, uint32_t phaseMs) {
  if (balanceState != BalanceState::Tipped)
    return "Lay the robot down first (past the cut-out angle), with its wheels free to turn";
  if (!(amps > 0) || amps > 0.5f) return "Test current must be 0-0.5 A";
  if (phaseMs < 50 || phaseMs > 2000) return "Test step must be 50-2000 ms";
  spinTestAmps = amps;
  spinTestPhaseMs = phaseMs;
  spinTestRequested = true;
  // Record it: 4 steps plus a little either side.
  const uint8_t secs = (uint8_t)constrain((int)((4 * phaseMs + 1500) / 1000), 1, (int)Capture::CAPTURE_MAX_SECONDS);
  Capture::start(secs, Capture::Trigger::Now);
  DeviceLog::printf("[test] wheel spin test: %.2f A, %lu ms steps (+, 0, -, 0), recording %u s\n",
                    amps, (unsigned long)phaseMs, secs);
  return nullptr;
}

const char *requestSelfRight(bool *stopping) {
  if (stopping) *stopping = false;
  if (balanceState == BalanceState::Righting) {
    srStopRequested = true;
    if (stopping) *stopping = true;
    return nullptr;
  }
  if (selfRightMode == 0) return "Self-righting is off (Tune tab: Self-righting)";
  if (balanceState != BalanceState::Tipped) return "It can only right itself while lying down (past the cut-out angle)";
  srStartRequested = true;
  return nullptr;
}

const char *requestSelfRightPush(uint32_t ms) {
  if (balanceState != BalanceState::Tipped) return "Lay the robot down on its hoop first";
  if (ms < 50 || ms > 500) return "Push length must be 50-500 ms";
  srPushRequestMs = ms;
  Capture::start(3, Capture::Trigger::Now);
  DeviceLog::printf("[selfright] push test: %.2f A for %lu ms, recording 3 s\n", selfRightCurrent, (unsigned long)ms);
  return nullptr;
}

void serviceSelfRight() {
  const uint8_t secs = srRecordSeconds;
  if (!secs) return;
  srRecordSeconds = 0;
  const Capture::State cs = Capture::state();
  if (cs == Capture::State::Recording || cs == Capture::State::Armed) return; // one is running already
  if (Capture::start(secs, Capture::Trigger::Now))
    DeviceLog::printf("[selfright] recording %u s\n", secs);
}

void serviceSetPointTrim() {
  static uint32_t lastPublishMs = 0, notBalancingSinceMs = 0;
  const uint32_t now = millis();
  const float trimDeg = setPointTrim * kRadToDeg;

  // Show the learned value on the page (and keep Settings in step) about
  // once a second, rather than on every control step.
  if (now - lastPublishMs >= 1000) {
    lastPublishMs = now;
    if (fabsf(trimDeg - Settings::value("spTrim")) >= 0.01f) Settings::set("spTrim", trimDeg);
  }

  // Save it once the robot has stopped balancing for a second (a flash
  // write stalls both cores for a moment, which would upset the balance).
  if (balanceState == BalanceState::Balancing) { notBalancingSinceMs = now; return; }
  if (now - notBalancingSinceMs < 1000) return;
  const float savedDeg = Settings::savedValue("spTrim");
  const float liveDeg = Settings::value("spTrim");
  if (isfinite(savedDeg) && fabsf(liveDeg - savedDeg) >= 0.05f) {
    const bool ok = Settings::saveKey("spTrim");
    DeviceLog::printf("[trim] set point trim %+.2f deg %s\n", liveDeg, ok ? "saved" : "save FAILED");
  }
}

const char *balanceStateName(BalanceState s) {
  switch (s) {
    case BalanceState::MotorsNotReady: return "motors-off";
    case BalanceState::ImuStalled:     return "imu-stall";
    case BalanceState::Paused:         return "paused";
    case BalanceState::Tipped:         return "tipped";
    case BalanceState::BatteryLow:     return "battery-low";
    case BalanceState::Balancing:      return "balancing";
    case BalanceState::Waiting:        return "waiting";
    case BalanceState::Runaway:        return "runaway";
    case BalanceState::Righting:       return "righting";
  }
  return "?";
}

ControlSample getControlSnapshot(uint16_t *loopHz) {
  portENTER_CRITICAL(&snapshotLock);
  const ControlSample s = snapshot;
  if (loopHz) *loopHz = measuredLoopHz;
  portEXIT_CRITICAL(&snapshotLock);
  return s;
}

void Task_LQR(void *pvParam) {
  // Run once per IMU sample: Task_BMI160 notifies this task after each one.
  // Running in between would only recompute the same output from the same
  // inputs, and (at equal priority) delay the other tasks on this core.
  setImuSampleListener(xTaskGetCurrentTaskHandle());

  // Started from the current time, so the first dt isn't "time since boot".
  unsigned long lastTime = micros();
  bool imuStalled = false;

  // Wheel angle [rad] and heading [rad], integrated while balancing and
  // reset each time balancing starts, and their references, which move
  // with the drive input (speed / turn-rate set points).
  float phi = 0;
  float phi_dot = 0;
  float phiDotSlow = 0; // phi_dot low-passed (SPEED_LEAD_FILTER_S), for maxSpeedLead
  float phiRef = 0;
  float speedRef = 0;   // [rad/s] ramped towards the requested speed
  float psi = 0;
  float psiRef = 0;
  uint32_t stillSinceMs = millis(); // for set point trim learning

  // Start/stop logic (see ENGAGE_* and RUNAWAY_MS).
  bool engaged = false;
  bool runaway = false;
  bool spinTesting = false;
  uint32_t spinStartMs = 0;
  uint32_t readySinceMs = 0, runawaySinceMs = 0, stallSinceMs = 0;

  // Loop rate measurement (steps per second).
  uint32_t rateWindowStartMs = millis();
  uint16_t stepsInWindow = 0;
  BalanceState previousState = BalanceState::MotorsNotReady;

  // Self-righting (see the self-righting block below).
  enum class SrPhase : uint8_t { Idle, Rocking, Settle, Watch, Push };
  SrPhase srPhase = SrPhase::Idle;
  uint32_t srPhaseStartMs = 0, srStillSinceMs = 0;
  uint32_t srPushMs = 0;
  uint8_t srTriesLeft = 0, srAttempt = 0;
  float srDir = 0;              // wanted direction of the reaction torque on the body: +1 = towards +tilt
  bool srAutoBlocked = false;   // don't start by itself until it has been moved
  float srBlockedTiltDeg = 0;

  // Lifted wheel detection: 0 = none, 1 = left lifted, 2 = right lifted.
  uint8_t lifted = 0;
  uint32_t liftOkSinceMs = 0, liftStartMs = 0;
  float liftPeak = 0;

  while (1) {
      // Wait for the next IMU sample. pdTRUE: clear the count, so if samples
      // were missed we just run once on the newest one.
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IMU_SAMPLE_TIMEOUT_MS)) == 0) {
        // IMU stalled: 0 A, and keep sending it so the drivers get an explicit
        // zero rather than relying on their own command timeout.
        if (!stallSinceMs) stallSinceMs = millis() | 1;
        if (engaged && millis() - stallSinceMs > STALL_DISENGAGE_MS) engaged = false;
        if (!imuStalled) {
          DeviceLog::printf("[lqr] no IMU sample for %lu ms - motors set to 0 A\n", (unsigned long)IMU_SAMPLE_TIMEOUT_MS);
          imuStalled = true;
        }
        leftTargetCurrent = 0;
        rightTargetCurrent = 0;
        balanceState = BalanceState::ImuStalled;
        previousState = BalanceState::ImuStalled;
        sendMotorCurrents(0, 0);
        {
          ControlSample s = getControlSnapshot();
          s.timeUs = micros();
          s.lqr = s.curL = s.curR = s.yawCur = 0;
          s.state = BalanceState::ImuStalled;
          publish(s);
        }
        lastTime = micros(); // don't integrate phi over the gap
        continue;
      }
      stallSinceMs = 0;
      if (imuStalled) {
        DeviceLog::println("[lqr] IMU samples resumed");
        imuStalled = false;
      }

      unsigned long currentTime = micros();
      float dt = (currentTime - lastTime) / 1e6f;  // Convert to seconds
      lastTime = currentTime;
      if (dt > 0.05f) dt = 0.05f; // e.g. after a flash write stalled us

      // Only trust the wheel velocities while both drivers are running FOC:
      // a stopped driver stops sending them, so the last value would go stale.
      const bool motorsOk = motorsReady();

      // Yaw rate [rad/s], positive in the direction a positive differential
      // current (left wheel forwards, right backwards) turns the robot.
      const float psi_dot = yawInvert ? -theta_dot[0] : theta_dot[0];

      // ---- Wheel speed, with lifted / slipping wheel detection ------------
      // On the ground the wheels' speed difference is set by the turn rate:
      // wheel_l - wheel_r = (track / r) * psi_dot. A wheel that leaves the
      // ground (a bump) or slips spins away from that within a few ms, and
      // the average would make the controller think the robot is rolling
      // fast. Then use the slower wheel (the one still gripping), corrected
      // for the turn, until they agree again.
      const float wl = motorsOk ? leftShaftVelocity : 0.0f;
      const float wr = motorsOk ? rightShaftVelocity : 0.0f;
      const float turnK = trackWidth / fmaxf(wheelRadius, 0.001f);
      const float mismatch = (wl - wr) - turnK * psi_dot;
      {
        const uint32_t nowMs = millis();
        if (!liftDetect || !motorsOk || !engaged) {
          lifted = 0;
          liftOkSinceMs = 0;
        } else if (fabsf(mismatch) > liftSpeed) {
          const uint8_t which = fabsf(wl) > fabsf(wr) ? 1 : 2;
          if (!lifted) {
            liftStartMs = nowMs;
            liftPeak = 0;
          }
          lifted = which;
          liftPeak = fmaxf(liftPeak, fabsf(mismatch));
          liftOkSinceMs = 0;
        } else if (lifted) {
          if (fabsf(mismatch) >= 0.5f * liftSpeed) liftOkSinceMs = 0;
          else if (!liftOkSinceMs) liftOkSinceMs = nowMs | 1;
          else if ((int32_t)(nowMs - liftOkSinceMs) >= (int32_t)LIFT_CLEAR_MS) {
            DeviceLog::printf("[lqr] %s wheel lifted/slipping for %lu ms (up to %.0f rad/s off) - used the other wheel's speed\n",
                              lifted == 1 ? "left" : "right", (unsigned long)(liftOkSinceMs - liftStartMs), liftPeak);
            lifted = 0;
            liftOkSinceMs = 0;
          }
        }
      }
      phi_dot = lifted == 1 ? wr + 0.5f * turnK * psi_dot   // left in the air: right wheel's speed
              : lifted == 2 ? wl - 0.5f * turnK * psi_dot   // right in the air: left wheel's
              : (wl + wr) / 2;

      // Drive input from whichever source is in charge (web joystick, BLE
      // gamepad, CAN module - see DriveInput.h). Stale sources read as 0.
      const DriveInput::Demand demand = DriveInput::current();
      const float axisX = demand.turn;
      const float axisY = demand.speed;

      // ---- Set points from the drive input -------------------------------
      // Speed: ramp towards the request at maxWheelAccel. The position
      // reference moves at that speed, so the position hold (K0) follows the
      // drive input instead of fighting it, and holds the spot on release.
      // The drive input is +1 = forward, but the drivers report a NEGATIVE
      // wheel speed when the robot rolls forward (phi, phi_dot, K0 and K3
      // all use the drivers' sign), hence the minus.
      //
      // The set point may not run more than maxSpeedLead ahead of the actual
      // (filtered) wheel speed in the direction of travel. On rough ground
      // the robot can't keep up with the ramp; without this the growing speed
      // error made it lean further and further forward (8-10° on the bumpy
      // test area) until one bigger bump tipped it over. The set point then
      // waits for the wheels (still ramping at maxWheelAccel at most), so on
      // smooth ground, where the wheels keep up, nothing changes.
      const float speedWanted = -axisY * maxWheelSpeed;
      const float maxStep = maxWheelAccel * dt;
      phiDotSlow += (phi_dot - phiDotSlow) * fminf(1.0f, dt / SPEED_LEAD_FILTER_S);
      float speedTarget = speedWanted;
      if (maxSpeedLead > 0) {
        if (speedWanted > 0 && speedRef >= 0)      speedTarget = fminf(speedWanted, phiDotSlow + maxSpeedLead);
        else if (speedWanted < 0 && speedRef <= 0) speedTarget = fmaxf(speedWanted, phiDotSlow - maxSpeedLead);
      }
      speedRef += constrain(speedTarget - speedRef, -maxStep, maxStep);
      // Turning: the requested yaw rate, and the heading it integrates to.
      const float yawRateRef = axisX * maxTurnRate;

      // Balancing starts (again): hold the spot and heading it starts at.
      const bool wasBalancing = (previousState == BalanceState::Balancing);
      if (!wasBalancing) {
        phi = phiRef = 0;
        psi = psiRef = 0;
        speedRef = 0;
      } else {
        phi += phi_dot * dt;
        phiRef += speedRef * dt;
        psi += psi_dot * dt;
        psiRef += yawRateRef * dt;
      }

      // Errors for the hold loops, limited: if the robot is pushed or turned
      // further than the limit, drag the reference along rather than wind up.
      float posErr = phi - phiRef;
      if (posErr > maxPositionError)  { phiRef = phi - maxPositionError; posErr = maxPositionError; }
      if (posErr < -maxPositionError) { phiRef = phi + maxPositionError; posErr = -maxPositionError; }
      float headErr = psi - psiRef;
      if (headErr > maxHeadingError)  { psiRef = psi - maxHeadingError; headErr = maxHeadingError; }
      if (headErr < -maxHeadingError) { psiRef = psi + maxHeadingError; headErr = -maxHeadingError; }

      // Set point in use: configured + learned trim.
      const float setPoint = angleSetPoint + setPointTrim;

      // ---- Balance (common-mode) current: LQR on the forward states -------
      // X = [phi, tilt, psi, phi_dot, tilt_dot, psi_dot]; u = -K X, split into
      // the part both wheels share (balance + position) and the part they
      // take with opposite signs (heading).
      //   K0: wheel position    K1: tilt         K2: heading
      //   K3: wheel speed       K4: tilt rate    K5: yaw rate
      //
      // Split into the tilt terms (catching a fall) and the position/speed
      // terms (getting back to the spot / speed). After a hard push they can
      // ask for current in opposite directions; at the current limit their
      // sum may then push the wrong way and it falls: after a hard push the
      // wheels are spinning fast, and the speed term (K3 x 50-100 rad/s) can
      // ask for 10 A or more, swamping the tilt terms. With balancePriority
      // the position/speed terms are limited to HOLD_LIMIT_FACTOR x the total
      // current limit, so once the tilt terms ask for more than that, they
      // win. Ordinary pushes and driving rarely reach the limit (a few % of
      // the time in the logs, briefly).
      // A plain clamp keeps the current a smooth, monotonic function of the
      // lean. (A first version faded these terms by a factor that depended
      // on the tilt demand: inside the fade more lean could give LESS current,
      // which made the current chatter at ~70 Hz under a sustained push - a
      // gear-like noise from the motors - and let the robot roll away.)
      const float iMax = constrain(maxWheelCurrent, 0.1f, 2.0f);
      const float tiltCurrent = -((K[1] * (theta[2] - setPoint)) + (K[4] * theta_dot[2]));
      const float holdWanted = -((K[0] * posErr) + (K[3] * (phi_dot - speedRef)));
      float holdCurrent = holdWanted;
      if (balancePriority) {
        const float holdMax = HOLD_LIMIT_FACTOR * 2 * iMax;
        holdCurrent = constrain(holdWanted, -holdMax, holdMax);
      }
      // Share of the position/speed terms used (recorded as "hold").
      const float holdScale = fabsf(holdWanted) > 1e-6f ? holdCurrent / holdWanted : 1.0f;
      const float current_LQR = tiltCurrent + holdCurrent;

      // ---- Heading (differential) current ---------------------------------
      // Feed-forward for the requested turn, plus feedback on the heading and
      // yaw-rate errors. Positive = left wheel forwards.
      const float yawCurrentWanted = turnFeedForward * axisX
                                   - (K[2] * headErr + K[5] * (psi_dot - yawRateRef));

      // Each wheel is limited to +-maxWheelCurrent. Balance comes first: the turning
      // current only gets what's left, so a hard turn can't starve the
      // balance loop and tip the robot over.
      const float half = constrain(current_LQR / 2, -iMax, iMax);
      const float yawRoom = iMax - fabsf(half);
      const float yawCurrent = constrain(yawCurrentWanted, -yawRoom, yawRoom);
      const float currentLeft = half + yawCurrent;
      const float currentRight = half - yawCurrent;

      // ---- Start / stop ----------------------------------------------------
      const uint32_t stepMs = millis();
      const bool upright = ((theta[2]) < (deactivationAngle * DEG_TO_RAD)) && ((theta[2])  > (-deactivationAngle * DEG_TO_RAD));
      // Don't drive one wheel on its own if the other driver is down.
      const bool batteryOk = !Battery::isLow();
      const bool canRun = !paused && motorsOk && batteryOk && upright;
      if (!canRun) engaged = false;

      if (engaged) {
        const float limit = fmaxf(runawaySpeed, 1.5f * maxWheelSpeed);
        if (fabsf(phi_dot) > limit) {
          if (!runawaySinceMs) runawaySinceMs = stepMs | 1;
          else if (stepMs - runawaySinceMs >= RUNAWAY_MS) {
            engaged = false;
            runaway = true;
            DeviceLog::printf("[lqr] wheels ran away (%.0f rad/s) - motors stopped; stand it up and hold it still to restart\n", phi_dot);
          }
        } else {
          runawaySinceMs = 0;
        }
      }

      if (!engaged && canRun) {
        const bool ready = fabsf(theta[2] - setPoint) < engageAngle * DEG_TO_RAD
                        && fabsf(theta_dot[2]) < ENGAGE_MAX_RATE
                        && fabsf(phi_dot) < ENGAGE_MAX_WHEEL;
        if (!ready) readySinceMs = 0;
        else if (!readySinceMs) readySinceMs = stepMs | 1;
        else if (stepMs - readySinceMs >= ENGAGE_HOLD_MS) {
          engaged = true;
          runaway = false;
          runawaySinceMs = 0;
        }
      } else {
        readySinceMs = 0;
      }

      if (engaged) {
        leftTargetCurrent = currentLeft;
        rightTargetCurrent = currentRight;
        balanceState = BalanceState::Balancing;
      } else {
        leftTargetCurrent = 0;
        rightTargetCurrent = 0;
        balanceState = !motorsOk  ? BalanceState::MotorsNotReady
                     : paused     ? BalanceState::Paused
                     : !batteryOk ? BalanceState::BatteryLow
                     : !upright   ? BalanceState::Tipped
                     : runaway    ? BalanceState::Runaway
                                  : BalanceState::Waiting;
      }

      // ---- Self-righting -------------------------------------------------
      // Lying down, the robot rests on its hoop with the wheels in the air
      // (CoM about 56 mm above the axle: it rests at about +-66-70 deg).
      // Driving can't lift it, and one reaction push isn't enough, but it
      // sits in a shallow dip: the wheels touch the ground at about 36 deg,
      // only ~1.6 mm higher for the CoM. So it rocks itself on the hoop:
      // the wheels are spun up as reaction wheels with the reaction torque
      // on the body in the direction it's swinging (bang-bang), which adds
      // energy every half swing. In the model (tools/tuning/self_right.py)
      // it reaches the wheels in about 3 half swings, ~1.5 s. Once it's
      // inside the handover angle, moving towards upright, the normal
      // balance controller takes over (with its references reset) and
      // drives it up the rest of the way.
      //
      // Signs: a positive current spins the wheels the way the robot leans
      // at positive tilt (the balance terms drive under the lean), so its
      // reaction tips the body towards NEGATIVE tilt. Hence the current is
      // -srDir x the rocking current (flipped by srInvert).
      {
        const bool tippedNow = (balanceState == BalanceState::Tipped);
        const bool srOk = motorsOk && batteryOk && !paused && !spinTesting;
        const float tiltDeg = theta[2] * kRadToDeg;
        const float fromSpDeg = (theta[2] - setPoint) * kRadToDeg;
        const float handoverDeg = fminf(selfRightHandover, deactivationAngle - 2.0f);
        const float wAvg = (wl + wr) / 2;
        const float iRock = constrain(selfRightCurrent, 0.0f, 2.0f);
        const bool towardsUpright = (fromSpDeg > 0) ? (theta_dot[2] < 0) : (theta_dot[2] > 0);

        auto stop = [&](SrPhase next) {
          srPhase = next;
          srPhaseStartMs = stepMs;
          leftTargetCurrent = 0;
          rightTargetCurrent = 0;
        };
        auto beginAttempt = [&]() {
          srAttempt++;
          srPhase = SrPhase::Rocking;
          srPhaseStartMs = stepMs;
          srDir = tiltDeg > 0 ? -1.0f : 1.0f; // first push towards upright
          DeviceLog::printf("[selfright] attempt %u: lying at %+.1f deg, rocking at %.2f A\n",
                            srAttempt, tiltDeg, iRock);
        };
        auto blockAuto = [&]() {
          srAutoBlocked = true;
          srBlockedTiltDeg = tiltDeg;
        };
        auto attemptFailed = [&](const char *why) {
          engaged = false;
          if (srTriesLeft > 1) {
            srTriesLeft--;
            DeviceLog::printf("[selfright] attempt %u failed (%s) - trying again\n", srAttempt, why);
            stop(SrPhase::Settle);
          } else {
            DeviceLog::printf("[selfright] attempt %u failed (%s) - giving up\n", srAttempt, why);
            blockAuto();
            stop(SrPhase::Idle);
          }
        };
        auto startSequence = [&](bool automatic) {
          srTriesLeft = constrain(selfRightTries, (uint8_t)1, (uint8_t)10);
          srAttempt = 0;
          DeviceLog::printf("[selfright] %s\n", automatic ? "lying still - getting up by itself" : "requested");
          if (selfRightRecord)
            srRecordSeconds = (uint8_t)constrain((int)ceilf(selfRightTimeout) + 4, 1, (int)Capture::CAPTURE_MAX_SECONDS);
          beginAttempt();
        };

        // Requests from the web page, the Xbox controller or a CAN module.
        if (srStopRequested) {
          srStopRequested = false;
          srStartRequested = false;
          if (srPhase == SrPhase::Rocking || srPhase == SrPhase::Settle || srPhase == SrPhase::Push) {
            DeviceLog::println("[selfright] stopped");
            blockAuto();
            stop(SrPhase::Idle);
          }
        }
        if (srStartRequested) {
          srStartRequested = false;
          if (srPhase == SrPhase::Idle && tippedNow && srOk && selfRightMode > 0) startSequence(false);
        }
        if (srPushRequestMs) {
          srPushMs = srPushRequestMs;
          srPushRequestMs = 0;
          if (srPhase == SrPhase::Idle && tippedNow && srOk) {
            srPhase = SrPhase::Push;
            srPhaseStartMs = stepMs;
            srDir = tiltDeg > 0 ? -1.0f : 1.0f;
          }
        }

        // Automatic start: lying still for selfRightDelay.
        if (srAutoBlocked && (balanceState == BalanceState::Waiting || balanceState == BalanceState::Balancing ||
                              fabsf(tiltDeg - srBlockedTiltDeg) > SR_MOVED_DEG))
          srAutoBlocked = false;
        const bool lyingStill = tippedNow && srOk && fabsf(theta_dot[2]) < SR_STILL_RATE
                             && fabsf(wl) < ENGAGE_MAX_WHEEL && fabsf(wr) < ENGAGE_MAX_WHEEL;
        if (srPhase != SrPhase::Idle || !lyingStill || selfRightMode < 2 || srAutoBlocked) {
          srStillSinceMs = 0;
        } else if (!srStillSinceMs) {
          srStillSinceMs = stepMs | 1;
        } else if (stepMs - srStillSinceMs >= (uint32_t)(selfRightDelay * 1000)) {
          srStillSinceMs = 0;
          startSequence(true);
        }

        // Anything that stops the motors (pause, battery, drivers) stops it.
        if (!srOk && (srPhase == SrPhase::Rocking || srPhase == SrPhase::Settle || srPhase == SrPhase::Push)) {
          DeviceLog::println("[selfright] stopped (paused, battery low or motors not ready)");
          blockAuto();
          stop(SrPhase::Idle);
        }

        switch (srPhase) {
          case SrPhase::Idle:
            break;

          case SrPhase::Rocking: {
            if (fabsf(tiltDeg) > SR_MAX_TILT_DEG) {
              attemptFailed("rolled too far over");
              break;
            }
            if (fabsf(fromSpDeg) < handoverDeg && towardsUpright) {
              // Hand over: start balancing now. previousState isn't
              // Balancing, so this step's LQR output was already worked out
              // with the position / speed / heading references reset.
              engaged = true;
              runaway = false;
              runawaySinceMs = 0;
              readySinceMs = 0;
              leftTargetCurrent = currentLeft;
              rightTargetCurrent = currentRight;
              balanceState = BalanceState::Balancing;
              DeviceLog::printf("[selfright] handed over at %+.1f deg, %+.0f deg/s, wheels %+.0f rad/s after %.2f s\n",
                                tiltDeg, theta_dot[2] * kRadToDeg, wAvg, (stepMs - srPhaseStartMs) / 1000.0f);
              srPhase = SrPhase::Watch;
              srPhaseStartMs = stepMs;
              break;
            }
            if (stepMs - srPhaseStartMs > (uint32_t)(selfRightTimeout * 1000)) {
              attemptFailed("timed out");
              break;
            }
            // Push with the swing; keep the last direction near the turning points.
            if (theta_dot[2] > SR_RATE_HYSTERESIS) srDir = 1.0f;
            else if (theta_dot[2] < -SR_RATE_HYSTERESIS) srDir = -1.0f;
            float i = (selfRightInvert ? srDir : -srDir) * iRock;
            // No more push in the direction the wheels already spin fast.
            if (fabsf(wAvg) >= selfRightWheelCap && (i > 0) == (wAvg > 0)) i = 0;
            leftTargetCurrent = i;
            rightTargetCurrent = i;
            balanceState = BalanceState::Righting;
            break;
          }

          case SrPhase::Settle: {
            leftTargetCurrent = 0;
            rightTargetCurrent = 0;
            balanceState = BalanceState::Righting;
            const uint32_t t = stepMs - srPhaseStartMs;
            if (t >= SR_SETTLE_MAX_MS || (t >= SR_SETTLE_MS && fabsf(theta_dot[2]) < SR_SETTLE_RATE)) {
              if (tippedNow || (!engaged && fabsf(fromSpDeg) >= handoverDeg)) beginAttempt();
              else stop(SrPhase::Idle);
            }
            break;
          }

          case SrPhase::Watch:
            if (paused || !motorsOk || !batteryOk) {
              srPhase = SrPhase::Idle;
            } else if (!engaged) {
              attemptFailed("fell over again");
            } else if (stepMs - srPhaseStartMs >= SR_WATCH_MS) {
              DeviceLog::printf("[selfright] up after %u attempt(s)\n", srAttempt);
              srPhase = SrPhase::Idle;
            }
            break;

          case SrPhase::Push: {
            if (stepMs - srPhaseStartMs >= srPushMs) {
              DeviceLog::println("[selfright] push test done");
              stop(SrPhase::Idle);
              break;
            }
            const float i = (selfRightInvert ? srDir : -srDir) * iRock;
            leftTargetCurrent = i;
            rightTargetCurrent = i;
            balanceState = BalanceState::Righting;
            break;
          }
        }
      }

      // ---- Wheel spin test (tuning): +I, 0, -I, 0 on both wheels ---------
      // Only while lying down (tipped), so the wheels are free; stops if the
      // robot is stood up, the drivers drop out or it's paused.
      if (spinTestRequested) {
        spinTestRequested = false;
        spinTesting = true;
        spinStartMs = stepMs;
      }
      if (spinTesting) {
        const uint32_t phase = (stepMs - spinStartMs) / spinTestPhaseMs;
        if (balanceState != BalanceState::Tipped) {
          spinTesting = false;
          DeviceLog::println("[test] wheel spin test stopped (robot no longer lying down)");
        } else if (phase >= 4) {
          spinTesting = false;
          DeviceLog::println("[test] wheel spin test done");
        } else {
          const float a = phase == 0 ? spinTestAmps : phase == 2 ? -spinTestAmps : 0.0f;
          leftTargetCurrent = a;
          rightTargetCurrent = a;
        }
      }

      // ---- Set point trim learning -----------------------------------------
      const uint32_t nowStepMs = millis();
      const bool still = (balanceState == BalanceState::Balancing) && wasBalancing
                      && axisX == 0 && axisY == 0 && speedRef == 0
                      && fabsf(posErr) < 0.9f * maxPositionError;
      if (!still) stillSinceMs = nowStepMs;
      else if (learnTrim && nowStepMs - stillSinceMs >= TRIM_SETTLE_MS) {
        setPointTrim = constrain(setPointTrim + (theta[2] - setPoint) * (dt / TRIM_TAU_S),
                                 -TRIM_LIMIT_RAD, TRIM_LIMIT_RAD);
      }

      // Send straight away (previously a separate 500 Hz task sent whatever
      // the latest values were, adding up to one extra period of delay).
      sendMotorCurrents(leftTargetCurrent, rightTargetCurrent);

      // Live data for the web page, and the recording (if one is running).
      const BalanceState st = balanceState;
      const bool running = (st == BalanceState::Balancing);
      ControlSample sample = {};
      sample.timeUs = currentTime;
      sample.tilt = theta[2] * kRadToDeg;
      sample.tiltRate = theta_dot[2] * kRadToDeg;
      sample.setpoint = setPoint * kRadToDeg;
      sample.phi = phi;
      sample.phiDot = phi_dot;
      sample.yawRate = psi_dot * kRadToDeg;
      sample.wheelL = wl;
      sample.wheelR = wr;
      sample.lifted = running ? lifted : 0;
      sample.holdScale = running ? holdScale : 1.0f;
      sample.lqr = current_LQR;
      sample.curL = leftTargetCurrent;
      sample.curR = rightTargetCurrent;
      sample.iqL = motorsOk ? leftMeasuredIq : NAN;
      sample.iqR = motorsOk ? rightMeasuredIq : NAN;
      sample.idL = motorsOk ? leftMeasuredId : NAN;
      sample.idR = motorsOk ? rightMeasuredId : NAN;
      sample.joyX = axisX;
      sample.joyY = axisY;
      sample.state = st;
      sample.source = (uint8_t)demand.source | (uint8_t)(demand.module << 4);
      sample.posErr = running ? posErr : 0.0f;
      sample.speedRef = running ? speedRef : 0.0f;
      sample.heading = psi * kRadToDeg;
      sample.headErr = running ? headErr * kRadToDeg : 0.0f;
      sample.yawRateRef = yawRateRef * kRadToDeg;
      sample.yawCur = running ? yawCurrent : 0.0f;
      sample.pitch = theta[1] * kRadToDeg;
      for (int i = 0; i < 3; i++) {
        sample.gyro[i] = imuGyro[i] * kRadToDeg;
        sample.accel[i] = imuAccel[i];
      }
      publish(sample);
      Capture::record(sample, previousState);
      previousState = st;

      stepsInWindow++;
      const uint32_t nowMs = millis();
      if (nowMs - rateWindowStartMs >= 1000) {
        portENTER_CRITICAL(&snapshotLock);
        measuredLoopHz = (uint16_t)((stepsInWindow * 1000UL) / (nowMs - rateWindowStartMs));
        portEXIT_CRITICAL(&snapshotLock);
        stepsInWindow = 0;
        rateWindowStartMs = nowMs;
      }

  }
}
