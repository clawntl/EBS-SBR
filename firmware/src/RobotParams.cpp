#include "RobotParams.h"

//float K[6] = {0, -5.6, 0, -0.0800, -0.6000, 0};
float K[6] = {0, -10, 0, -0.05, -0.6, 0};

float angleSetPoint = 2 * DEG_TO_RAD;
float deactivationAngle = 42;

float alpha = 0.998;
uint16_t IMUsampleRate = 500;

bool paused = false;

float turnCurrentGain = 0.15;
float moveCurrentGain = 0.3;
