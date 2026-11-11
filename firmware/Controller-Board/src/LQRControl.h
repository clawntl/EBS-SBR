#pragma once

// Target currents [A] for the left/right motors, produced by Task_LQR and
// consumed by Task_sendMotorCurrents.
extern float leftTargetCurrent;
extern float rightTargetCurrent;

void Task_LQR(void *pvParam);
