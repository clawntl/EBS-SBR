#include "LQRControl.h"

#include <Arduino.h>

#include "CanBus.h"
#include "IMUDriver.h"
#include "RobotParams.h"

float leftTargetCurrent = 0;
float rightTargetCurrent = 0;

void Task_LQR(void *pvParam) {
  unsigned long lastTime = 0;

  // uint16_t motorKV = 100;
  // float motorKT = 60 / (2 * PI * motorKV);

  float phi = 0;
  float phi_dot = 0;

  while (1) {
    //if (xSemaphoreTake(xIMU_Semaphore, pdMS_TO_TICKS(1)) == pdTRUE) {


      unsigned long currentTime = micros();
      float dt = (currentTime - lastTime) / 1e6f;  // Convert to seconds
      lastTime = currentTime;


      phi_dot = ((leftShaftVelocity + rightShaftVelocity) / 2);

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

    // Work out the indivudual wheel torques [Nm]
    float currentLeft = (current_LQR / 2);
    float currentRight = (current_LQR / 2);



      // float turnCurrent = 0.2f;
      // if (turn) {
      //   currentLeft += turnCurrent;
      //   //currentRight -= (turnCurrent);
      //   Serial.print("TURNING \t");
      // }

      //float turnCurrent = 0.2f;
      //float moveCurrent = 1.0f;

      // float axisX = gamePadAxes[0];
      // float axisY = gamePadAxes[1];

      /*
      currentLeft *= (1 - axisGain);
      currentRight *= (1 - axisGain);

      currentLeft += turnCurrent * axisX;
      currentLeft += moveCurrent * axisY;

      currentRight -= turnCurrent * axisX;
      currentRight += moveCurrent * axisY;
      */


                // if (xSemaphoreTake(xGamepad_Semaphore, portMAX_DELAY) == pdTRUE) {



                // }

      //Serial.print("ang: "); Serial.print(RAD_TO_DEG * theta[2], 3); Serial.print("°");
      //Serial.print("\t"); Serial.print("sp: "); Serial.print(RAD_TO_DEG * angleSetPoint, 2); Serial.print("°");

      // Serial.print("\tL_RAW: "); Serial.print(currentLeft, 3); Serial.print("A");
      // Serial.print("\tR_RAW: "); Serial.print(currentRight, 3); Serial.print("A");

                    //Serial.print("\talpha: "); Serial.print(alpha, 4);
                    // Serial.print("\tIMU: "); Serial.print(IMUsampleRate); Serial.print(" Hz");

      currentLeft = constrain(currentLeft, -1, 1);
      currentRight = constrain(currentRight, -1, 1);

      //if (!paused && (((theta[1] - angleSetPoint) < (deactivationAngle * DEG_TO_RAD)) && ((theta[1] - angleSetPoint)  > (-deactivationAngle * DEG_TO_RAD)))) {
      if (!paused && (((theta[2]) < (deactivationAngle * DEG_TO_RAD)) && ((theta[2])  > (-deactivationAngle * DEG_TO_RAD)))) {
        leftTargetCurrent = currentLeft;
        rightTargetCurrent = currentRight;
      } else {
        // currentLeft = 0;
        // currentLeft += turnCurrent * axisX;
        // currentLeft += moveCurrent * axisY;

        // currentRight = 0;
        // currentRight -= turnCurrent * axisX;
        // currentRight += moveCurrent * axisY;

        // currentLeft = constrain(currentLeft, -1, 1);
        // currentRight = constrain(currentRight, -1, 1);

        // motorLeft.target = currentLeft;
        // motorRight.target = currentRight;

        leftTargetCurrent = 0;
        rightTargetCurrent = 0;
      }

      //Serial.print("\t"); Serial.print("L: "); Serial.print(currentLeft, 3); Serial.print("A");
      //Serial.print("\t"); Serial.print("R: "); Serial.print(currentRight, 3); Serial.print("A");

      //Serial.print("\t"); Serial.print("DA: "); Serial.print(deactivationAngle, 1); Serial.print("°");

      // Serial.print("\t"); Serial.print("tC: "); Serial.print(turnCurrent, 2); Serial.print("");
      // Serial.print("\t"); Serial.print("mC: "); Serial.print(moveCurrent, 2); Serial.print("");




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



    //}
  }
}
