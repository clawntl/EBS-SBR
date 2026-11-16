#pragma once

#include <Arduino.h>

#include "LQRControl.h"

// Recording of every control step for reviewing and tuning the balance
// controller: one sample per LQR run (the control rate, 500 Hz by default),
// up to CAPTURE_MAX_SECONDS, downloadable as CSV from /capture.csv. The CSV
// starts with the settings that were active, so one file is enough to see
// what happened and why.
//
// The buffer (~1 MB) is allocated in PSRAM on the first recording.
//
// record() is called by Task_LQR; everything else from core 0.
namespace Capture {

constexpr uint8_t CAPTURE_MAX_SECONDS = 60;

enum class State : uint8_t {
    Idle,       // nothing recorded (or buffer not allocated yet)
    Armed,      // waiting for the robot to start balancing
    Recording,
    Done,       // data available
};

enum class Trigger : uint8_t { Now, NextBalance };

const char *stateName(State s);

// Starts (or arms) a capture of `seconds`. Returns false if the buffer
// can't be allocated or a download is in progress.
bool start(uint8_t seconds, Trigger trigger);
void cancel();

State state();
uint32_t samples();    // recorded so far
uint32_t capacity();   // for the current capture
float rateHz();        // nominal sample rate of the current capture (control rate)

// Task_LQR: call once per control step.
void record(const ControlSample &s, BalanceState previousState);

// Pull-style CSV generator for chunked HTTP responses.
class CsvReader {
  public:
    CsvReader();
    ~CsvReader();
    // Fills up to maxLen bytes; returns 0 when finished.
    size_t read(uint8_t *buf, size_t maxLen);

  private:
    bool nextLine();
    int32_t row = -1;  // -1: header block
    String pending;
    size_t pendingPos = 0;
};

} // namespace Capture
