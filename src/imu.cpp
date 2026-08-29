#include "imu.h"

#include <Wire.h>
#include <Preferences.h>
#include <math.h>

#include "SensorQMI8658.hpp"

// ====== IMU CONFIG / MATH ======

static constexpr float DEG2RAD = 0.017453292519943295f;
static constexpr float RAD2DEG = 57.29577951308232f;

// G-meter range - used to clamp the displayed value (should match main.cpp)
static constexpr float GMETER_FS_G = 0.40f;

// Simple smoothing filter (0..1); 1.0 = no filtering, 0.1 = heavy smoothing
static constexpr float LPF_ALPHA = 0.25f;

// Number of samples to collect during zero calibration
static constexpr int CALIB_SAMPLES = 40;

// I2C pins - duplicated here as static so they don't collide with main.cpp
static constexpr int I2C_SDA_PIN = 40;
static constexpr int I2C_SCL_PIN = 39;

// ====== Sensor object ======
static SensorQMI8658 QMI;
static IMUdata Accel;
static IMUdata Gyro;   // unused for now, kept for possible future use (yaw)

// ====== NVS (calibration) ======
static Preferences prefsImu;

// Note: NVS namespace is "att3" (calibration via a full 3D rotation instead
// of subtracting angles) - after this change a fresh calibration is needed
// once.
static constexpr const char* NVS_NAMESPACE = "att3";

// Raw gravity vector measured at calibration time (the "zero" position).
static float cal_vx = 0.0f;
static float cal_vy = 0.0f;
static float cal_vz = 1.0f;

// Rotation matrix built from (cal_vx, cal_vy, cal_vz): rotates a raw
// accelerometer reading from the sensor's own frame into the "vehicle
// frame" (Z up, matching however the device was standing during
// calibration). Because this is a full 3D rotation - not a plain
// pitch/roll angle subtraction - it correctly compensates for a COMPOUND
// mounting tilt (e.g. landscape screen + tilted-back housing). Without
// this, real vehicle roll used to leak partly into the displayed pitch
// reading and vice versa.
static float calR[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

static void buildCalRotation(float vx, float vy, float vz)
{
    const float n = sqrtf(vx * vx + vy * vy + vz * vz);
    if (n < 1e-6f) return; // degenerate input, keep the identity matrix

    vx /= n; vy /= n; vz /= n;

    // target: (0, 0, 1)
    const float ax = vy;          // v x target, target = (0,0,1)
    const float ay = -vx;
    const float az = 0.0f;
    const float s = sqrtf(ax * ax + ay * ay + az * az); // sin(theta)
    const float c = vz;                                  // cos(theta) = dot(v, target)

    if (s < 1e-6f) {
        if (c > 0.0f) {
            calR[0][0]=1; calR[0][1]=0; calR[0][2]=0;
            calR[1][0]=0; calR[1][1]=1; calR[1][2]=0;
            calR[2][0]=0; calR[2][1]=0; calR[2][2]=1;
        } else {
            // 180 deg - device was calibrated upside down (not expected in practice)
            calR[0][0]=-1; calR[0][1]=0; calR[0][2]=0;
            calR[1][0]=0;  calR[1][1]=1; calR[1][2]=0;
            calR[2][0]=0;  calR[2][1]=0; calR[2][2]=-1;
        }
        return;
    }

    // Rodrigues' rotation formula: R = I + K + K^2 * (1 - c) / s^2, K = skew(axis)
    const float K[3][3] = {
        {  0.0f, -az,   ay },
        {  az,    0.0f, -ax },
        { -ay,    ax,    0.0f }
    };

    float K2[3][3];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            K2[i][j] = K[i][0]*K[0][j] + K[i][1]*K[1][j] + K[i][2]*K[2][j];

    const float factor = (1.0f - c) / (s * s);

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            const float I_ij = (i == j) ? 1.0f : 0.0f;
            calR[i][j] = I_ij + K[i][j] + K2[i][j] * factor;
        }
}

static void applyCalRotation(float ax, float ay, float az, float& ox, float& oy, float& oz)
{
    ox = calR[0][0]*ax + calR[0][1]*ay + calR[0][2]*az;
    oy = calR[1][0]*ax + calR[1][1]*ay + calR[1][2]*az;
    oz = calR[2][0]*ax + calR[2][1]*ay + calR[2][2]*az;
}

// ====== Filter state / last sample ======
static bool       g_sample_valid = false;
static bool       g_lpf_inited   = false;
static float      lpf_pitch_deg  = 0.0f;
static float      lpf_roll_deg   = 0.0f;
static float      lpf_long_g     = 0.0f;
static float      lpf_lat_g      = 0.0f;
static ImuSample  g_lastSample   = { false, 0, 0, 0, 0 };

// ====== Math helpers ======

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Pitch / roll from the accelerometer - standard tilt formula
static void accelPitchRollRad(float ax, float ay, float az,
                              float& pitchRad, float& rollRad)
{
    // Fixed convention:
    //   roll  = atan2(ay, az);
    //   pitch = atan2(-ax, sqrt(ay^2 + az^2));
    rollRad  = atan2f(ay, az);
    pitchRad = atan2f(-ax, sqrtf(ay * ay + az * az));
}

// Gravity vector implied by a given pitch/roll (in the IMU frame)
static void gravityFromPR(float pitchRad, float rollRad,
                          float& gx, float& gy, float& gz)
{
    const float sp = sinf(pitchRad);
    const float cp = cosf(pitchRad);
    const float sr = sinf(rollRad);
    const float cr = cosf(rollRad);

    gx = -sp;
    gy =  sr * cp;
    gz =  cr * cp;
}

// ====== Calibration: NVS ======

static void loadCalibration()
{
    prefsImu.begin(NVS_NAMESPACE, true);
    cal_vx = prefsImu.getFloat("vx", 0.0f);
    cal_vy = prefsImu.getFloat("vy", 0.0f);
    cal_vz = prefsImu.getFloat("vz", 1.0f);
    prefsImu.end();

    buildCalRotation(cal_vx, cal_vy, cal_vz);

    Serial.printf("[IMU] Cal loaded: v0=(%.3f, %.3f, %.3f)\n", cal_vx, cal_vy, cal_vz);
}

static void saveCalibration(float vx, float vy, float vz)
{
    prefsImu.begin(NVS_NAMESPACE, false);
    prefsImu.putFloat("vx", vx);
    prefsImu.putFloat("vy", vy);
    prefsImu.putFloat("vz", vz);
    prefsImu.end();

    cal_vx = vx;
    cal_vy = vy;
    cal_vz = vz;
    buildCalRotation(cal_vx, cal_vy, cal_vz);

    Serial.printf("[IMU] Cal saved: v0=(%.3f, %.3f, %.3f)\n", vx, vy, vz);
}

// ====== API ======

void imu_init()
{
    Serial.println("[IMU] init()");

    loadCalibration();

    if (!QMI.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN)) {
        Serial.println("[IMU] Failed to find QMI8658!");
        // No while(1) here - we'd rather run without IMU data than brick the UI.
        return;
    }

    QMI.configAccelerometer(SensorQMI8658::ACC_RANGE_4G,
                            SensorQMI8658::ACC_ODR_1000Hz,
                            SensorQMI8658::LPF_MODE_0,
                            true);

    QMI.configGyroscope(SensorQMI8658::GYR_RANGE_64DPS,
                        SensorQMI8658::GYR_ODR_896_8Hz,
                        SensorQMI8658::LPF_MODE_3,
                        true);

    QMI.enableAccelerometer();
    QMI.enableGyroscope();

    g_sample_valid = false;
    g_lpf_inited   = false;

    Serial.println("[IMU] Ready.");
}

void imu_update()
{
    if (!QMI.getDataReady()) {
        return;
    }

    if (!QMI.getAccelerometer(Accel.x, Accel.y, Accel.z)) {
        return;
    }

    // We could also read the gyro for a future yaw estimate, but it's
    // deliberately NOT used for pitch/roll (would mix in yaw drift):
    (void)QMI.getGyroscope(Gyro.x, Gyro.y, Gyro.z);

    // 1) Rotate the raw reading into the "vehicle frame" per calibration
    //    (see buildCalRotation) - compensates for a COMPOUND mounting
    //    tilt, not just a single axis.
    float ax, ay, az;
    applyCalRotation(Accel.x, Accel.y, Accel.z, ax, ay, az);

    // 2) Pitch / roll from the rotated vector
    float pitchRad = 0.0f, rollRad = 0.0f;
    accelPitchRollRad(ax, ay, az, pitchRad, rollRad);

    float pitchDeg = pitchRad * RAD2DEG;
    float rollDeg  = rollRad  * RAD2DEG;

    // 3) Gravity vector from pitch/roll
    float gx, gy, gz;
    gravityFromPR(pitchRad, rollRad, gx, gy, gz);

    // 4) Dynamic component = measurement - gravity (in "g" units)
    const float dx = ax - gx;
    const float dy = ay - gy;
    // Convention: long_g = fore/aft (minus dx), lat_g = left/right (dy)
    float long_g = clampf(-dx, -GMETER_FS_G, GMETER_FS_G);
    float lat_g  = clampf( dy, -GMETER_FS_G, GMETER_FS_G);

    // 5) Simple smoothing filter (EMA)
    if (!g_lpf_inited) {
        lpf_pitch_deg = pitchDeg;
        lpf_roll_deg  = rollDeg;
        lpf_long_g    = long_g;
        lpf_lat_g     = lat_g;
        g_lpf_inited  = true;
    } else {
        lpf_pitch_deg += LPF_ALPHA * (pitchDeg - lpf_pitch_deg);
        lpf_roll_deg  += LPF_ALPHA * (rollDeg  - lpf_roll_deg);
        lpf_long_g    += LPF_ALPHA * (long_g   - lpf_long_g);
        lpf_lat_g     += LPF_ALPHA * (lat_g    - lpf_lat_g);
    }

#ifdef IMU_DEBUG_SERIAL
    // --- DEBUG: print orientation and accel (for calibration tests) ---
    // OFF by default: this used to print 5x/s exactly while the MAIN/CAL
    // screen was active (i.e. while swiping). On ESP32-S3 with USB-CDC
    // (ARDUINO_USB_CDC_ON_BOOT=1), writing to Serial can block when
    // nobody is reading the port (device running in the field, no PC
    // attached) - which caused periodic UI "freezes" and poor touch
    // responsiveness. Enable via `-DIMU_DEBUG_SERIAL` in build_flags only
    // for debugging at a desk.
    static uint32_t lastPrintMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= 200) {  // print ~5 times per second
        lastPrintMs = nowMs;

        Serial.printf(
            "[IMU] raw=(%.3f,%.3f,%.3f) rot=(%.3f,%.3f,%.3f) | "
            "pitch=%.1f roll=%.1f | long_g=%.3f lat_g=%.3f\n",
            Accel.x, Accel.y, Accel.z,
            ax, ay, az,
            lpf_pitch_deg, lpf_roll_deg,
            lpf_long_g, lpf_lat_g
        );
    }
#endif

    // 6) Update the last sample
    g_lastSample.valid    = true;
    g_lastSample.pitchDeg = lpf_pitch_deg;
    g_lastSample.rollDeg  = lpf_roll_deg;
    g_lastSample.long_g   = lpf_long_g;
    g_lastSample.lat_g    = lpf_lat_g;
}

ImuSample imu_get_sample()
{
    return g_lastSample;
}

bool imu_calibrate_zero()
{
    Serial.println("[IMU] Calibrate ZERO...");

    float sum_ax = 0.0f;
    float sum_ay = 0.0f;
    float sum_az = 0.0f;
    int   got    = 0;

    const uint32_t t_start = millis();

    while (got < CALIB_SAMPLES && (millis() - t_start) < 3000) {
        if (!QMI.getDataReady()) {
            delay(5);
            continue;
        }
        if (!QMI.getAccelerometer(Accel.x, Accel.y, Accel.z)) {
            delay(5);
            continue;
        }

        sum_ax += Accel.x;
        sum_ay += Accel.y;
        sum_az += Accel.z;
        got++;

        delay(10);
    }

    if (got == 0) {
        Serial.println("[IMU] Calibrate: no data!");
        return false;
    }

    const float ax = sum_ax / got;
    const float ay = sum_ay / got;
    const float az = sum_az / got;

    // Save the RAW gravity vector from the calibration pose - buildCalRotation()
    // turns it into the full 3D rotation that compensates for the mounting
    // tilt (see the comment on calR above).
    saveCalibration(ax, ay, az);

    // Reset the filter so it starts fresh in the new reference frame
    g_lpf_inited   = false;
    g_sample_valid = false;
    g_lastSample   = { false, 0, 0, 0, 0 };

    Serial.println("[IMU] Calibrate: OK.");
    return true;
}
