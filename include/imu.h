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

// Like imu_get_sample(), but with a small hysteresis on pitch/rollDeg so
// a stationary vehicle's sensor noise doesn't visibly "breathe" on a
// display (long_g/lat_g are passed through unchanged). Use this wherever
// pitch/roll gets shown to a person - the MAIN screen, the MONITOR
// screen, and the phone's /live endpoint all use it; imu_get_sample()
// itself stays fully responsive for anything else.
ImuSample imu_get_display_sample();

// Zero calibration at the current position (mounting tilt)
// Returns false if no data could be collected from the IMU.
bool imu_calibrate_zero();

// QMI8658's on-die temperature sensor, degrees C. Not the same as
// ambient/cabin temperature - it's warmed by the board itself - but
// useful as a proxy while investigating whether pitch/roll drift with
// temperature (see the sensor's own datasheet bias-vs-temp curves).
// Returns NAN if the read fails.
float imu_get_temperature_c();
