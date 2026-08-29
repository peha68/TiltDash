#pragma once

#include <Arduino.h>

struct ImuSample {
    bool  valid;
    float pitchDeg;  // after calibration, for display
    float rollDeg;   // after calibration, for display
    float long_g;    // fore/aft acceleration, after calibration
    float lat_g;     // lateral (left/right) acceleration, after calibration
};

// Initializes the IMU (QMI8658 + loads calibration)
void imu_init();

// One read/update cycle from the accelerometer + angle/G recalculation
void imu_update();

// Returns the last sample (note: .valid is true only after the first
// successful read)
ImuSample imu_get_sample();

// Zero calibration at the current position (mounting tilt)
// Returns false if no data could be collected from the IMU.
bool imu_calibrate_zero();
