#include "motion.h"
#include <M5Unified.h>

// Spread of the acceleration in a 1 s window: below STILL_G the device is
// still (BMI270 noise is ~1 mg, an idling engine adds some mg), above MOVE_G
// or with a mean shifted by more than SHIFT_G it moves. Rough values, tuned
// with the "vib" reading on the acceleration page.
static const float STILL_G = 0.02f;
static const float MOVE_G = 0.04f;
static const float SHIFT_G = 0.03f;
static const uint32_t SAMPLE_MS = 100;
static const uint32_t WINDOW_MS = 1000;
static const float SMOOTH = 0.5f;        // exponential average of the 100 ms samples
// Forward learning: speed changes of at least LEARN_MIN_G between two GPS
// fixes LEARN_MIN_DT_MS .. LEARN_MAX_DT_MS apart, above LEARN_MIN_KMH;
// forward is known after LEARN_COUNT of them.
static const float LEARN_MIN_G = 0.05f;
static const float LEARN_MIN_KMH = 5;
static const uint32_t LEARN_MIN_DT_MS = 500;
static const uint32_t LEARN_MAX_DT_MS = 2000;
static const int LEARN_COUNT = 8;

struct Vec {
  float x, y, z;
  Vec operator+(const Vec& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec operator-(const Vec& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec operator*(float f) const { return {x * f, y * f, z * f}; }
  float dot(const Vec& o) const { return x * o.x + y * o.y + z * o.z; }
  Vec cross(const Vec& o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  float len() const { return sqrtf(dot(*this)); }
  Vec norm() const { float l = len(); return l > 0 ? *this * (1 / l) : *this; }
};

static bool hasImu = false;

// 1 s window: sums for mean and spread
static Vec winSum, winSq;
static int winN = 0;
static uint32_t winStartMs = 0;
static Vec lastMean;
static float lastVib = 0;
static bool lastWindowStill = false;
static bool isStill = false;
static uint32_t stillSinceMs = 0;
static Vec stillMean;

// 100 ms sample
static Vec sampSum;
static int sampN = 0;
static uint32_t sampStartMs = 0;

// Calibration: up = reading at rest (1 g upwards), fwd = horizontal unit vector
static bool isCal = false, isLearned = false;
static Vec up, upN, fwd, right;
static Vec fixSum;            // horizontal acceleration since the last GPS fix
static int fixN = 0;
static Vec learnSum;
static int learnN = 0;
static float prevKmh = -1;
static uint32_t prevKmhMs = 0;

// Readings
static float curLon = 0, curLat = 0, curVert = 0;
static motion::Maxima maxv;
static float env[motion::SECTORS];
static motion::Sample hist[motion::HISTORY];
static int histHead = 0, histCount = 0;
static float peakAccel = 0, peakBrake = 0, peakLat = 0, peakUp = 0, peakDown = 0;

static Vec horizontal(const Vec& v) { return v - upN * v.dot(upN); }

static void setFwd(const Vec& f) {
  fwd = horizontal(f).norm();
  right = fwd.cross(upN);
}

static void clearReadings() {
  curLon = curLat = curVert = 0;
  motion::resetMax();
  histHead = histCount = 0;
}

static void addSample(const Vec& a) {
  Vec lin = a - up;
  Vec h = horizontal(lin);
  fixSum = fixSum + h;
  fixN++;
  curLon += (h.dot(fwd) - curLon) * SMOOTH;
  curLat += (h.dot(right) - curLat) * SMOOTH;
  curVert += (lin.dot(upN) - curVert) * SMOOTH;

  maxv.accel = max(maxv.accel, curLon);
  maxv.brake = max(maxv.brake, -curLon);
  maxv.right = max(maxv.right, curLat);
  maxv.left = max(maxv.left, -curLat);
  maxv.up = max(maxv.up, curVert);
  maxv.down = max(maxv.down, -curVert);
  peakAccel = max(peakAccel, curLon);
  peakBrake = max(peakBrake, -curLon);
  peakLat = max(peakLat, fabsf(curLat));
  peakUp = max(peakUp, curVert);
  peakDown = max(peakDown, -curVert);
  // Clockwise from forward, sectors centered on their direction
  float deg = atan2f(curLat, curLon) * RAD_TO_DEG + 360 + 180.0f / motion::SECTORS;
  int s = (int)(deg / (360.0f / motion::SECTORS)) % motion::SECTORS;
  env[s] = max(env[s], hypotf(curLon, curLat));

  auto mg = [](float g) { return (int16_t)constrain(g * 1000, -32000, 32000); };
  hist[histHead] = {mg(curLon), mg(curLat), mg(curVert)};
  histHead = (histHead + 1) % motion::HISTORY;
  histCount = min(histCount + 1, motion::HISTORY);
}

static void finishWindow(uint32_t now) {
  Vec mean = winSum * (1.0f / winN);
  Vec var = winSq * (1.0f / winN) - Vec{mean.x * mean.x, mean.y * mean.y, mean.z * mean.z};
  lastVib = sqrtf(max(0.0f, var.x + var.y + var.z));
  lastMean = mean;
  lastWindowStill = lastVib < STILL_G;
  if (isStill && (lastVib > MOVE_G || (mean - stillMean).len() > SHIFT_G)) {
    isStill = false;
  } else if (!isStill && lastWindowStill) {
    isStill = true;
    stillSinceMs = now;
    stillMean = mean;
  }
}

namespace motion {

bool begin() {
  hasImu = M5.Imu.isEnabled();
  winStartMs = sampStartMs = millis();
  return hasImu;
}

bool available() { return hasImu; }

void update() {
  if (!hasImu || !M5.Imu.update()) return;
  Vec a;
  if (!M5.Imu.getAccel(&a.x, &a.y, &a.z)) return;
  uint32_t now = millis();
  winSum = winSum + a;
  winSq = winSq + Vec{a.x * a.x, a.y * a.y, a.z * a.z};
  winN++;
  sampSum = sampSum + a;
  sampN++;
  if (now - sampStartMs >= SAMPLE_MS) {
    if (isCal) addSample(sampSum * (1.0f / sampN));
    sampSum = {};
    sampN = 0;
    sampStartMs = now;
  }
  if (now - winStartMs >= WINDOW_MS) {
    finishWindow(now);
    winSum = winSq = {};
    winN = 0;
    winStartMs = now;
  }
}

bool still() { return isStill; }
uint32_t stillMs() { return isStill ? millis() - stillSinceMs : 0; }
float vibration() { return lastVib; }

bool calibrate() {
  if (!hasImu || !lastWindowStill) return false;
  up = lastMean;
  upN = up.norm();
  // Until it is learned, forward is the device axis closest to horizontal
  Vec axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  Vec best = axes[0];
  for (const Vec& ax : axes)
    if (fabsf(ax.dot(upN)) < fabsf(best.dot(upN))) best = ax;
  setFwd(best);
  isCal = true;
  isLearned = false;
  learnSum = fixSum = {};
  learnN = fixN = 0;
  prevKmh = -1;
  clearReadings();
  return true;
}

bool calibrated() { return isCal; }
bool forwardLearned() { return isLearned; }

void setCalibration(const float u[3], const float f[3], bool fwdLearned) {
  up = {u[0], u[1], u[2]};
  if (up.len() < 0.5f) return;  // not calibrated
  upN = up.norm();
  setFwd({f[0], f[1], f[2]});
  isCal = fwd.len() > 0.5f;
  isLearned = isCal && fwdLearned;
}

void getCalibration(float u[3], float f[3]) {
  u[0] = up.x; u[1] = up.y; u[2] = up.z;
  f[0] = fwd.x; f[1] = fwd.y; f[2] = fwd.z;
}

// The mean horizontal acceleration between two fixes points forward when the
// speed went up and backward when it went down; summed up weighted by the
// speed change, lateral parts (curves) cancel out.
bool gpsSpeed(float kmh, uint32_t ms) {
  if (!isCal) return false;
  bool learnedNow = false;
  uint32_t dt = ms - prevKmhMs;
  if (prevKmh >= 0 && fixN && dt >= LEARN_MIN_DT_MS && dt <= LEARN_MAX_DT_MS &&
      max(kmh, prevKmh) >= LEARN_MIN_KMH) {
    float dvG = (kmh - prevKmh) / 3.6f / (dt / 1000.0f) / 9.81f;
    if (fabsf(dvG) >= LEARN_MIN_G) {
      learnSum = learnSum + fixSum * (dvG / fixN);
      if (++learnN >= LEARN_COUNT && horizontal(learnSum).len() > 0) {
        setFwd(learnSum);
        if (!isLearned) {
          isLearned = true;
          learnedNow = true;
          clearReadings();  // collected with the guessed direction
        }
      }
    }
  }
  prevKmh = kmh;
  prevKmhMs = ms;
  fixSum = {};
  fixN = 0;
  return learnedNow;
}

float lonG() { return curLon; }
float latG() { return curLat; }
float vertG() { return curVert; }
const Maxima& maxima() { return maxv; }

void resetMax() {
  maxv = {};
  for (float& e : env) e = 0;
  peakAccel = peakBrake = peakLat = peakUp = peakDown = 0;
}

float envelope(int sector) { return env[sector]; }
int historyCount() { return histCount; }

Sample historyAt(int i) {
  return hist[(histHead - histCount + i + HISTORY) % HISTORY];
}

void takePeaks(float& accel, float& brake, float& lat, float& up, float& down) {
  accel = peakAccel;
  brake = peakBrake;
  lat = peakLat;
  up = peakUp;
  down = peakDown;
  peakAccel = peakBrake = peakLat = peakUp = peakDown = 0;
}

}  // namespace motion
