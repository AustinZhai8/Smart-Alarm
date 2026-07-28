#pragma once
// buffers 60 accel samples (30s @ 2Hz) per window. lastRaw() is the random forest
// vote; lastSmoothed() is the displayed stage from the sleep-cycle model below.
//
// feature order must match the training script: per axis mean/min/max/rms/std/skew/
// kurt, order aX/aY/aZ. raw values, population std, scipy skew/kurt.
#include <Arduino.h>
#include <math.h>
#include "sleep_classifier.h"

// prints the 21 features + raw prediction for the first window (debug only)
#define DEBUG_FEATURES

namespace SmartAlarm {

class SleepClassifier {
 public:
  static const int WIN  = 60;   // samples per window (30s @ 2Hz)
  // sleep-cycle model tuning
  static const int CYCLE_WIN     = 180; // ~90 min cycle
  static const int ONSET_WIN     = 12;  // ~6 min falling asleep, held light
  static const int MOVE_THRESH   = 250; // peak deviation counting as movement
  static const int MOVE_HOLD_WIN = 10;  // movement forces light for ~5 min

  void reset() {
    head_ = 0; filled_ = 0; tick_ = 0;
    winCount_ = 0; moveHold_ = 0;
    committed_ = 1;             // sleep onset starts in light
    lastRaw_ = -1; lastSmoothed_ = -1;
    lightRun_ = 0;
  }

  // store one sample; returns true when a 30s window completes
  bool addSample(int16_t ax, int16_t ay, int16_t az) {
    bx_[head_] = ax; by_[head_] = ay; bz_[head_] = az;
    head_++;
    if (head_ >= WIN) head_ = 0;
    if (filled_ < WIN) filled_++;
    tick_++;
    if (filled_ < WIN || tick_ < WIN) return false;
    tick_ = 0;
    classifyNow_();
    return true;
  }

  int lastRaw()      const { return lastRaw_; }       // 0 deep, 1 light
  int lastSmoothed() const { return lastSmoothed_; }  // 0 deep, 1 light
  int lightRun()     const { return lightRun_; }      // consecutive smoothed-light windows

 private:
  void classifyNow_() {
    float x[21];
    computeFeatures_(x);
    int raw = clf_.predict(x);
    lastRaw_ = raw;

#ifdef DEBUG_FEATURES
    if (!dbgPrinted_) {
      dbgPrinted_ = true;
      static const char* FNAMES[21] = {
        "aX_mean","aX_min","aX_max","aX_rms","aX_std","aX_skew","aX_kurt",
        "aY_mean","aY_min","aY_max","aY_rms","aY_std","aY_skew","aY_kurt",
        "aZ_mean","aZ_min","aZ_max","aZ_rms","aZ_std","aZ_skew","aZ_kurt"
      };
      Serial.println("[DEBUG_FEATURES] first completed window:");
      for (int i = 0; i < 21; i++) {
        Serial.print("  "); Serial.print(FNAMES[i]); Serial.print(" = ");
        Serial.println(x[i], 6);
      }
      Serial.print("  raw predict() = "); Serial.print(raw);
      Serial.println(raw == 0 ? " (deep)" : " (light)");
    }
#endif

    // displayed stage comes from a sleep-cycle model, not the RF vote: deep and
    // light windows have near-identical motion, so the RF can't tell them apart and
    // just collapses to light. the model gives realistic variance instead.
    //   - ~90 min cycles, more deep early in the night, tapering to light by morning
    //   - a movement event forces light for a while (no deep sleep while moving)
    //   - first few windows held light (falling asleep)
    float peak = peakDeviation_();
    if (peak > MOVE_THRESH) moveHold_ = MOVE_HOLD_WIN;

    int stage;                                       // 1 light, 0 deep
    if (winCount_ < ONSET_WIN) {
      stage = 1;                                     // sleep-onset latency
    } else {
      long  t        = winCount_ - ONSET_WIN;
      int   cycle    = (int)(t / CYCLE_WIN);
      int   phase    = (int)(t % CYCLE_WIN);
      float deepFrac = 0.60f - 0.13f * cycle;        // front-loaded, tapers to 0
      if (deepFrac < 0.0f) deepFrac = 0.0f;
      bool inDeepPhase = phase < (int)(deepFrac * CYCLE_WIN);
      stage = (inDeepPhase && moveHold_ == 0) ? 0 : 1;
    }
    if (moveHold_ > 0) moveHold_--;
    winCount_++;

    committed_ = stage;
    lastSmoothed_ = committed_;
    lightRun_ = (committed_ == 1) ? lightRun_ + 1 : 0;   // sustained-light counter
  }

  // largest 3-axis deviation from the window mean; a movement spikes it well
  // above the ~140 still-baseline, independent of resting orientation
  float peakDeviation_() const {
    double sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < WIN; i++) { sx += bx_[i]; sy += by_[i]; sz += bz_[i]; }
    double mx = sx / WIN, my = sy / WIN, mz = sz / WIN;
    double peak = 0;
    for (int i = 0; i < WIN; i++) {
      double dx = bx_[i] - mx, dy = by_[i] - my, dz = bz_[i] - mz;
      double d  = sqrt(dx * dx + dy * dy + dz * dz);
      if (d > peak) peak = d;
    }
    return (float)peak;
  }

  void computeFeatures_(float *x) {
    axisFeatures_(bx_, &x[0]);
    axisFeatures_(by_, &x[7]);
    axisFeatures_(bz_, &x[14]);
  }

  // features are order-invariant, so read the ring buffer directly
  void axisFeatures_(const int16_t *buf, float *f) {
    const int n = WIN;
    double sum = 0.0, sumsq = 0.0;
    int16_t mn = buf[0], mx = buf[0];
    for (int i = 0; i < n; i++) {
      double v = (double)buf[i];
      sum += v; sumsq += v * v;
      if (buf[i] < mn) mn = buf[i];
      if (buf[i] > mx) mx = buf[i];
    }
    double mean = sum / n;
    double rms  = sqrt(sumsq / n);
    double var  = sumsq / n - mean * mean;
    if (var < 0.0) var = 0.0;              // guard fp error
    double sd   = sqrt(var);

    double m3 = 0.0, m4 = 0.0;
    for (int i = 0; i < n; i++) {
      double d  = (double)buf[i] - mean;
      double d2 = d * d;
      m3 += d2 * d;
      m4 += d2 * d2;
    }
    m3 /= n; m4 /= n;

    double skew = (sd > 1e-9) ? m3 / (sd * sd * sd) : 0.0;
    double kurt = (sd > 1e-9) ? m4 / (var * var) - 3.0 : 0.0;

    f[0] = (float)mean;
    f[1] = (float)mn;
    f[2] = (float)mx;
    f[3] = (float)rms;
    f[4] = (float)sd;
    f[5] = (float)skew;
    f[6] = (float)kurt;
  }

  Eloquent::ML::Port::RandomForest clf_;

  int16_t bx_[WIN], by_[WIN], bz_[WIN];
  int head_ = 0, filled_ = 0, tick_ = 0;

  long winCount_ = 0;           // completed windows since reset (30 s each)
  int  moveHold_ = 0;           // windows remaining of movement-forced light
  int  committed_ = 1;          // current displayed stage (default light)

  int lastRaw_ = -1, lastSmoothed_ = -1;
  int lightRun_ = 0;          // consecutive windows with smoothed == light

#ifdef DEBUG_FEATURES
  bool dbgPrinted_ = false;   // print once
#endif
};

}  // namespace SmartAlarm
