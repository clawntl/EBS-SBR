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

// If no IMU sample arrives for this long, stop driving the motors (0 A)
// rather than balancing on a stale tilt measurement. 10 ms = 5 missed
// samples at the default 500 Hz.
constexpr uint32_t IMU_SAMPLE_TIMEOUT_MS = 10;
} // namespace

const char *balanceStateName(BalanceState s) {
  switch (s) {
    case BalanceState::MotorsNotReady: return "motors-off";
    case BalanceState::ImuStalled:     return "imu-stall";
    case BalanceState::Paused:         return "paused";
    case BalanceState::Tipped:         return "tipped";
    case BalanceState::BatteryLow:     return "battery-low";
    case BalanceState::Balancing:      return "balancing";
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

  // uint16_t motorKV = 100;
  // float motorKT = 60 / (2 * PI * motorKV);

  float phi = 0;
  float phi_dot = 0;

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
        if (!imuStalled) {
          DeviceLog::printf("[lqr] no IMU sample for %lu ms - motors set to 0 A\n", (unsigned long)IMU_SAMPLE_TIMEOUT_MS);
          imuStalled = true;
        }
        leftTargetCurrent = 0;
        rightTargetCurrent = 0;
        balanceState = BalanceState::ImuStalled;
        previousState = BalanceState::ImuStalled;
        sendMotorCurrents(0, 0, BOTH_MOTOR_CAN_ADDR);
        {
          ControlSample s = getControlSnapshot();
          s.timeUs = micros();
          s.lqr = s.curL = s.curR = 0;
          s.state = BalanceState::ImuStalled;
          publish(s);
        }
        lastTime = micros(); // don't integrate phi over the gap
        continue;
      }
      if (imuStalled) {
        DeviceLog::println("[lqr] IMU samples resumed");
        imuStalled = false;
      }


      unsigned long currentTime = micros();
      float dt = (currentTime - lastTime) / 1e6f;  // Convert to seconds
      lastTime = currentTime;


      // Only trust the wheel velocities while both drivers are running FOC:
      // a stopped driver stops sending them, so the last value would go stale.
      const bool motorsOk = motorsReady();
      phi_dot = motorsOk ? ((leftShaftVelocity + rightShaftVelocity) / 2) : 0.0f;

      phi += phi_dot * dt;



      //phi = constrain(phi, -2, 2);

      // Find the total wheel torque [Nm] required using LQR
      // X = [x, th, psi_ang, dx, th_dot, psi_dot]
      // X_prime = [phi, th, psi_ang, phi_dot, th_dot, psi_dot]

        // K0: Wheel displacement gain
        // K1: Tilt angle gain
        // K2: Yaw angle gain
        // K3: Wheel velocity gain
        // K4: Tilt velocity gain
        // K5: Yaw velocity gain

    float current_LQR = -(
    (K[1] * (theta[2] - angleSetPoint))
    + (K[3] * phi_dot)
    + (K[4] * theta_dot[2])
    );

    // Pull in the latest joystick input, but treat it as stale (i.e. zero
    // it) if WebInterface hasn't refreshed it recently - protects against a
    // dropped WebSocket leaving the robot driving on a frozen command.
    float axisX = joystickAxisX;
    float axisY = joystickAxisY;
    if (millis() - joystickLastUpdateMs > JOYSTICK_TIMEOUT_MS) {
      axisX = 0;
      axisY = 0;
    }
    // Work out the indivudual wheel torques [Nm]: the joystick's turn/move
    // request is added on top of the full balancing current, not blended
    // against it - attenuating the LQR feedback by joystick magnitude let
    // large commands starve the balance loop, tip the robot past
    // deactivationAngle, and cut the motors entirely.
    float currentLeft = (current_LQR / 2);
    float currentRight = (current_LQR / 2);

    currentLeft += turnCurrentGain * axisX;
    currentLeft += moveCurrentGain * axisY;

    currentRight -= turnCurrentGain * axisX;
    currentRight += moveCurrentGain * axisY;

      //Serial.print("ang: "); Serial.print(RAD_TO_DEG * theta[2], 3); Serial.print("°");
      //Serial.print("\t"); Serial.print("sp: "); Serial.print(RAD_TO_DEG * angleSetPoint, 2); Serial.print("°");

      // Serial.print("\tL_RAW: "); Serial.print(currentLeft, 3); Serial.print("A");
      // Serial.print("\tR_RAW: "); Serial.print(currentRight, 3); Serial.print("A");

                    //Serial.print("\talpha: "); Serial.print(alpha, 4);
                    // Serial.print("\tIMU: "); Serial.print(IMUsampleRate); Serial.print(" Hz");

      currentLeft = constrain(currentLeft, -1, 1);
      currentRight = constrain(currentRight, -1, 1);

      //if (!paused && (((theta[1] - angleSetPoint) < (deactivationAngle * DEG_TO_RAD)) && ((theta[1] - angleSetPoint)  > (-deactivationAngle * DEG_TO_RAD)))) {
      const bool upright = ((theta[2]) < (deactivationAngle * DEG_TO_RAD)) && ((theta[2])  > (-deactivationAngle * DEG_TO_RAD));
      // Don't drive one wheel on its own if the other driver is down.
      const bool batteryOk = !Battery::isLow();
      if (!paused && upright && motorsOk && batteryOk) {
        leftTargetCurrent = currentLeft;
        rightTargetCurrent = currentRight;
        balanceState = BalanceState::Balancing;
      } else {
        leftTargetCurrent = 0;
        rightTargetCurrent = 0;
        balanceState = !motorsOk  ? BalanceState::MotorsNotReady
                     : paused     ? BalanceState::Paused
                     : !batteryOk ? BalanceState::BatteryLow
                                  : BalanceState::Tipped;
      }

      // Send straight away (previously a separate 500 Hz task sent whatever
      // the latest values were, adding up to one extra period of delay).
      sendMotorCurrents(leftTargetCurrent, rightTargetCurrent, BOTH_MOTOR_CAN_ADDR);

      // Live data for the web page, and the recording (if one is running).
      const BalanceState st = balanceState;
      const ControlSample sample = {
        currentTime,
        theta[2] * kRadToDeg, theta_dot[2] * kRadToDeg, angleSetPoint * kRadToDeg,
        phi, phi_dot, theta_dot[0] * kRadToDeg,
        motorsOk ? leftShaftVelocity : 0.0f, motorsOk ? rightShaftVelocity : 0.0f,
        current_LQR, leftTargetCurrent, rightTargetCurrent,
        axisX, axisY, st,
      };
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

      //Serial.print("\t"); Serial.print("L: "); Serial.print(currentLeft, 3); Serial.print("A");
      //Serial.print("\t"); Serial.print("R: "); Serial.print(currentRight, 3); Serial.print("A");

      //Serial.print("\t"); Serial.print("DA: "); Serial.print(deactivationAngle, 1); Serial.print("°");




                // Serial.print("\tphi_dot: "); Serial.print(phi_dot, 4); Serial.print(" rad/s");
                // Serial.print("\tphi : "); Serial.print(phi, 4); Serial.print(" rad");
                // // Work out the current required [A] for the requested left motor torque
                // //float Iq_Left = torqueLeft / motorKT;
                // currentLeft = constrain(currentLeft, -1, 1); // Limit the maximum current just to be safe

                // // Work out the current required [A] for the requested right motor torque
                // //float Iq_Right = torqueRight / motorKT;
                // currentRight = constrain(currentRight, -1, 1); // Limit the maximum current just to be safe






    //   Serial.print("\t\tK1: "); Serial.print(K[1], 3);
    //   Serial.print("\tK3: "); Serial.print(K[3], 3);
    //   Serial.print("\tK4: "); Serial.print(K[4], 3);
    //   Serial.println();
  }
}
