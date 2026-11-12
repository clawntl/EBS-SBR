#pragma once

// Brings up the WiFi access point as a captive portal (wildcard DNS +
// OS-probe redirects), the LittleFS-hosted tuning page, and the
// WebSocket endpoint used to read/write RobotParams live.
void initWebInterface();

// Must be called often from loop(): services DNS (core 2.x), sends the
// page heartbeat and cleans up disconnected WebSocket clients.
void loopWebInterface();
