#pragma once
// Buffers 60 accel samples (30 s @ 2 Hz) and runs them through the exported
// Random Forest to get a deep/light prediction, then smooths it.
//
// The feature order has to match the Python training script exactly or the
// model's predictions stop meaning anything: per axis, mean/min/max/rms/std/
// skew/kurt, axes in order aX (x[0..6]), aY (x[7..13]), aZ (x[14..20]). Raw
// values, no gravity removal. std is population (n denominator), skew/kurt
// are scipy's biased/Fisher definitions. std~0 -> skew and kurt are just 0.
#include <Arduino.h>
#include <math.h>
#include "sleep_classifier.h"

// Define DEBUG_FEATURES to Serial-print the 21 named features + the raw predict()
// result for the FIRST completed window only (then nothing). Comment out to disable.
#define DEBUG_FEATURES

namespace SmartAlarm {

class SleepClassifier {
 public:
  static const int WIN  = 60;   // samples per window (30 s @ 2 Hz)
  static const int HIST = 10;   // smoothing history length (~5 min of windows)

  void reset() {
    head_ = 0; filled_ = 0; tick_ = 0;
    histHead_ = 0; histCount_ = 0;
    committed_ = 1;             // default to "light" until enough votes arrive
    lastRaw_ = -1; lastSmoothed_ = -1;
    lightRun_ = 0;
  }

  // Store one sample. Returns true once a full 30 s window has elapsed and a new
  // prediction (lastRaw / lastSmoothed) is ready.
  bool addSample(int16_t ax, int16_t ay, int16_t az) {
    bx_[head_] = ax; by_[head_] = ay; bz_[head_] = az;
    head_ = (head_ + 1) % WIN;
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

    // Smoothing: majority vote over the last HIST raw predictions (ties keep the
    // current committed stage; defaults to light before the window fills).
    //
    // NOTE: an earlier design added a "min bout of HIST consecutive identical raw"
    // layer on top of this. With ~26% raw noise, a 10-in-a-row streak almost never
    // occurs, so the stage locked onto its first value (light) and deep was never
    // logged. Validated against the recorded nights: majority-only tracks the true
    // deep fraction closely, the two-layer version collapsed it toward 0%.
    hist_[histHead_] = raw;
    histHead_ = (histHead_ + 1) % HIST;
    if (histCount_ < HIST) histCount_++;
    int ones = 0;
    for (int i = 0; i < histCount_; i++) ones += hist_[i];
    if (ones * 2 != histCount_)                     // not a tie -> follow majority
      committed_ = (ones * 2 > histCount_) ? 1 : 0; // tie -> keep committed_

    lastSmoothed_ = committed_;
    lightRun_ = (committed_ == 1) ? lightRun_ + 1 : 0;   // sustained-light counter
  }

  void computeFeatures_(float *x) {
    axisFeatures_(bx_, &x[0]);
    axisFeatures_(by_, &x[7]);
    axisFeatures_(bz_, &x[14]);
  }

  // Window features are order-invariant, so the ring buffer can be read directly.
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

  int hist_[HIST];              // ring of last HIST raw predictions
  int histHead_ = 0, histCount_ = 0;
  int committed_ = 1;           // current smoothed stage (default light)

  int lastRaw_ = -1, lastSmoothed_ = -1;
  int lightRun_ = 0;          // consecutive windows with smoothed == light

#ifdef DEBUG_FEATURES
  bool dbgPrinted_ = false;   // one-shot for the lifetime of the program
#endif
};

}  // namespace SmartAlarm
