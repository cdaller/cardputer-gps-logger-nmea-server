#pragma once
#include <Arduino.h>
#include <time.h>

// Dead reckoning while the GPS has no fix (tunnel), with the accelerometer
// and gyroscope (Cardputer ADV, calibrated, forward learned).
//
// From each good fix on, the position is carried forward: heading turned by
// the gyroscope, speed from the last GPS speed plus the longitudinal
// acceleration (0 while standing still, pulled back toward the GPS speed so a
// remaining error can't add up), altitude from the slope.
namespace deadreckon {

static const uint32_t MAX_SEC = 600;  // no estimate after this long without fix

struct Estimate {
  double lat, lon;
  float altM, speedKmh, courseDeg;
  time_t utc;
};

// Difference between the estimate and the fix that ended a gap, for correcting
// the estimated points: shifted by the part of it their time is into the gap.
struct Correction {
  double dLat, dLon;
  float dAlt;
  time_t from, to;  // last fix before and first fix after the gap
};

// Each good GPS fix. Returns true if the gap before it was estimated, then
// correction() belongs to it.
bool fix(double lat, double lon, float altM, float speedKmh, float courseDeg, time_t utc);
void update(bool gpsFix);  // call often
bool active();             // no fix, estimating
uint32_t seconds();        // since the last fix while active
const Estimate& estimate();
const Correction& correction();

}  // namespace deadreckon
