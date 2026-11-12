#pragma once

// Reads and processes any pending lines on Serial (tuning commands such as
// "K1 -10", "SP 2", "DA 42", "CA 0.998"), plus "MS" (motor driver status)
// and "MR" (retry starting a driver that gave up). Non-blocking: returns immediately
// if there is nothing to read. Call once per loop() iteration.
void processSerialCommands();
