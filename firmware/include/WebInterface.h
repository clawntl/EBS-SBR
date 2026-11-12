#pragma once

// Brings up the WiFi access point, LittleFS-hosted tuning page, and the
// WebSocket endpoint used to read/write RobotParams live.
void initWebInterface();

// Must be called periodically (e.g. from loop()) to let the WebSocket
// library clean up disconnected clients.
void loopWebInterface();
