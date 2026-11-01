#include "deadreckon.h"
#include "motion.h"

static const float COURSE_MIN_KMH = 5;     // GPS course is noise below this
static const float SPEED_TAU_S = 60;       // estimated speed drifts back to the GPS speed
static const float MAX_DT_S = 0.5f;
static const double EARTH_R = 6371000;

static bool haveFix = false, haveCourse = false;
static bool isActive = false;
static bool estimated = false;  // the current gap was estimated
static uint32_t fixMs = 0, lastMs = 0;
static time_t fixUtc = 0;
static float fixMps = 0, mps = 0;
static deadreckon::Estimate est;
static deadreckon::Correction corr;

namespace deadreckon {

bool fix(double lat, double lon, float altM, float speedKmh, float courseDeg, time_t utc) {
  bool wasEstimated = estimated;
  if (wasEstimated) corr = {lat - est.lat, lon - est.lon, altM - est.altM, fixUtc, utc};
  estimated = isActive = false;
  if (speedKmh >= COURSE_MIN_KMH) {
    est.courseDeg = courseDeg;
    haveCourse = true;
  }
  est.lat = lat;
  est.lon = lon;
  est.altM = altM;
  est.utc = utc;
  fixMps = mps = speedKmh / 3.6f;
  est.speedKmh = speedKmh;
  fixMs = lastMs = millis();
  fixUtc = utc;
  haveFix = true;
  return wasEstimated;
}

void update(bool gpsFix) {
  uint32_t now = millis();
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  bool usable = haveFix && haveCourse && motion::forwardLearned() && motion::gyroAvailable();
  if (!usable) {
    isActive = false;
    return;
  }
  // After MAX_SEC still carried forward, for the correction when the fix is back
  isActive = !gpsFix && now - fixMs <= MAX_SEC * 1000UL;
  estimated |= isActive;
  if (dt > MAX_DT_S) return;
  // Carried forward also between fixes: the gap starts at the last one.
  est.courseDeg = fmodf(est.courseDeg + motion::yawRate() * dt + 360, 360);
  if (motion::still()) {
    mps = 0;
  } else {
    mps += (motion::lonG() - motion::lonBias()) * 9.81f * dt;
    mps += (fixMps - mps) * min(dt / SPEED_TAU_S, 1.0f);
    mps = max(mps, 0.0f);
  }
  double d = mps * dt, c = est.courseDeg * DEG_TO_RAD;
  est.lat += d * cos(c) / EARTH_R * RAD_TO_DEG;
  est.lon += d * sin(c) / (EARTH_R * cos(est.lat * DEG_TO_RAD)) * RAD_TO_DEG;
  est.altM += d * motion::pitchSin();
  est.speedKmh = mps * 3.6f;
  est.utc = fixUtc + (now - fixMs) / 1000;
}

bool active() { return isActive; }
uint32_t seconds() { return isActive ? (millis() - fixMs) / 1000 : 0; }
const Estimate& estimate() { return est; }
const Correction& correction() { return corr; }

}  // namespace deadreckon
