#pragma once

#include <Arduino.h>
#include <functional>

// Status messages ("[wifi] connected ...", "[imu] gyro offset ...").
// Each line goes to Serial as before and into a small RAM ring buffer, so
// the web page's Log tab can show recent history and stream new lines -
// serial is only usable on the bench (USB lead to the controller board).
//
// Safe to call from any task, including the control task (formatting
// happens outside the lock; the lock only covers a short copy).
namespace DeviceLog {

void printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void println(const char *text);

// Sequence number of the newest line (0 = nothing logged yet).
uint32_t latest();

// Calls `fn` for each line still in the buffer with a sequence number
// greater than `after`, oldest first.
void since(uint32_t after, const std::function<void(uint32_t seq, uint32_t ms, const char *text)> &fn);

} // namespace DeviceLog
