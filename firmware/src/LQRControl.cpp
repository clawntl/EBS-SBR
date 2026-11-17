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
#include "Settings.h"

float leftTargetCurrent = 0;
float rightTargetCurrent = 0;

volatile float joystickAxisX = 0;
volatile float joystickAxisY = 0;
volatile unsigned long joystickLastUpdateMs = 0;

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

constexpr unsigned long JOYSTICK_TIMEOUT_MS = 300;

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
constexpr float TRIM_TAU_S = 20.0f;
constexpr float TRIM_LIMIT_RAD = 5.0f * DEG_TO_RAD;

// If no IMU sample arrives for this long, stop driving the motors (0 A)
// rather than balancing on a stale tilt measurement. 10 ms = 5 missed
// samples at the default 500 Hz.
constexpr uint32_t IMU_SAMPLE_TIMEOUT_MS = 10;
} // namespace

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
  float phiRef = 0;
  float speedRef = 0;   // [rad/s] ramped towards the requested speed
  float psi = 0;
  float psiRef = 0;
  uint32_t stillSinceMs = millis(); // for set point trim learning

  // Start/stop logic (see ENGAGE_* and RUNAWAY_MS).
  bool engaged = false;
  bool runaway = false;
  uint32_t readySinceMs = 0, runawaySinceMs = 0, stallSinceMs = 0;

  // Loop rate measurement (steps per second).
  uint32_t rateWindowStartMs = millis();
  uint16_t stepsInWindow = 0;
  BalanceState previousState = BalanceState::MotorsNotReady;

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
      phi_dot = motorsOk ? ((leftShaftVelocity + rightShaftVelocity) / 2) : 0.0f;

      // Yaw rate [rad/s], positive in the direction a positive differential
      // current (left wheel forwards, right backwards) turns the robot.
      const float psi_dot = yawInvert ? -theta_dot[0] : theta_dot[0];

      // Pull in the latest joystick input, but treat it as stale (i.e. zero
      // it) if WebInterface hasn't refreshed it recently - protects against a
      // dropped WebSocket leaving the robot driving on a frozen command.
      float axisX = joystickAxisX;
      float axisY = joystickAxisY;
      if (millis() - joystickLastUpdateMs > JOYSTICK_TIMEOUT_MS) {
        axisX = 0;
        axisY = 0;
      }
      axisX = constrain(axisX, -1.0f, 1.0f);
      axisY = constrain(axisY, -1.0f, 1.0f);

      // ---- Set points from the drive input -------------------------------
      // Speed: ramp towards the request at maxWheelAccel. The position
      // reference moves at that speed, so the position hold (K0) follows the
      // drive input instead of fighting it, and holds the spot on release.
      // The drive input is +1 = forward, but the drivers report a NEGATIVE
      // wheel speed when the robot rolls forward (phi, phi_dot, K0 and K3
      // all use the drivers' sign), hence the minus.
      const float speedWanted = -axisY * maxWheelSpeed;
      const float maxStep = maxWheelAccel * dt;
      speedRef += constrain(speedWanted - speedRef, -maxStep, maxStep);
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
      const float current_LQR = -(
          (K[0] * posErr)
        + (K[1] * (theta[2] - setPoint))
        + (K[3] * (phi_dot - speedRef))
        + (K[4] * theta_dot[2])
      );

      // ---- Heading (differential) current ---------------------------------
      // Feed-forward for the requested turn, plus feedback on the heading and
      // yaw-rate errors. Positive = left wheel forwards.
      const float yawCurrentWanted = turnFeedForward * axisX
                                   - (K[2] * headErr + K[5] * (psi_dot - yawRateRef));

      // Each wheel is limited to +-1 A. Balance comes first: the turning
      // current only gets what's left, so a hard turn can't starve the
      // balance loop and tip the robot over.
      const float half = constrain(current_LQR / 2, -1.0f, 1.0f);
      const float yawRoom = 1.0f - fabsf(half);
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
      sample.wheelL = motorsOk ? leftShaftVelocity : 0.0f;
      sample.wheelR = motorsOk ? rightShaftVelocity : 0.0f;
      sample.lqr = current_LQR;
      sample.curL = leftTargetCurrent;
      sample.curR = rightTargetCurrent;
      sample.joyX = axisX;
      sample.joyY = axisY;
      sample.state = st;
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
