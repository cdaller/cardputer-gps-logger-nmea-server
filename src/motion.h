#pragma once
#include <Arduino.h>

// Accelerometer of the Cardputer ADV (BMI270; v1.x has no IMU).
//
// Stillness: spread of the acceleration over 1 s windows. Car / bike / plane
// readings: longitudinal (+ = accelerating, - = braking), lateral
// (+ = to the right) and vertical (+ = up) acceleration in g, without gravity,
// along the axes of the vehicle as calibrated. That needs the
// calibration: "up" is measured while standing still, "forward" is learned
// from GPS speed changes (acceleration that goes along with the speed change).
namespace motion {

// One 100 ms average, in milli-g.
struct Sample {
  int16_t lon, lat, vert;
};

struct Maxima {
  float accel, brake, left, right, up, down;  // g, all >= 0
};

static const int HISTORY = 240;   // samples, 10 per second
static const int SECTORS = 36;    // max envelope of the g-g diagram, 10 degrees each

bool begin();              // false without IMU
bool available();
void update();             // call often: reads the IMU

// Stillness, also without calibration
bool still();
uint32_t stillMs();        // how long still, 0 = moving
float vibration();         // spread of the last 1 s window, g

// Calibration. calibrate() takes the last 1 s window as "up" and fails if the
// device was not still. Forward then has to be learned again.
bool calibrate();
bool calibrated();
bool forwardLearned();
void setCalibration(const float up[3], const float fwd[3], bool fwdLearned);
void getCalibration(float up[3], float fwd[3]);
// Each new GPS speed, to learn forward. Returns true when forward was just learned.
bool gpsSpeed(float kmh, uint32_t ms);

// Readings, 0 if not calibrated
float lonG();
float latG();
float vertG();
const Maxima& maxima();
void resetMax();            // also the peaks for the track points
float envelope(int sector);  // max total g per direction, sector 0 = forward, clockwise
int historyCount();
Sample historyAt(int i);     // 0 = oldest
// Peaks since the last call, for the track points (g, >= 0)
void takePeaks(float& accel, float& brake, float& lat, float& up, float& down);

}  // namespace motion
