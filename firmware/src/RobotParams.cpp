#include "RobotParams.h"

//float K[6] = {0, -5.6, 0, -0.0800, -0.6000, 0};
float K[6] = {0, -10, 0, -0.05, -0.6, 0};

float angleSetPoint = 2 * DEG_TO_RAD;
float setPointTrim = 0;
bool learnTrim = true;
float deactivationAngle = 42;
float engageAngle = 10;
float sideCutoutAngle = 30;
float runawaySpeed = 60;
float maxWheelCurrent = 1.5;
bool balancePriority = true;
bool liftDetect = true;
float liftSpeed = 8;
float wheelRadius = 0.030f;
float trackWidth = 0.110f;

float alpha = 0.998;
uint16_t IMUsampleRate = 500;

bool paused = false;

float maxWheelSpeed = 10;
float maxWheelAccel = 20;
float maxSpeedLead = 8;
float maxTurnRate = 90 * DEG_TO_RAD;
float turnFeedForward = 0.15;
bool yawInvert = false;
float maxPositionError = 10;
float maxHeadingError = 30 * DEG_TO_RAD;

uint8_t selfRightMode = 1;
float selfRightCurrent = 1.5f;
float selfRightWheelCap = 200;
float selfRightHandover = 34;
float selfRightTimeout = 6;
uint8_t selfRightTries = 3;
float selfRightDelay = 2;
bool selfRightInvert = false;
bool selfRightRecord = true;
