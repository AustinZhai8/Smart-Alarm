# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

**Smart Alarm** — an ESP32-based smart alarm clock with a rotary-encoder navigation UI, MPU-6050 sleep tracking, an on-device sleep-stage classifier, SD card logging, and OpenWeatherMap weather. The hardware is an ESP32 DevKit driving a 240×320 ST7789 TFT via TFT_eSPI.

All firmware lives in a single sketch: `SmartAlarm/SmartAlarm.ino`. The ML training pipeline lives under `ml/` (`Aura_Clock_Random_Forest_ML.ipynb` notebook, `log_sleep.py` serial logger, and `ml/data/` training CSVs).

### Repository layout
```
SmartAlarm/   firmware sketch (.ino + smart_alarm_features.h + generated sleep_classifier.h)
ml/           training notebook, serial logger, and ml/data/ training CSVs
README.md  CLAUDE.md  .gitignore
```

## Compiling

```
"C:\Users\austi\AppData\Local\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app SmartAlarm
```

Target: **ESP32 Arduino core 3.x** — use `ledcAttach(pin, freq, bits)` / `ledcWrite(pin, val)`, **not** the core 2.x `ledcSetup` / `ledcAttachPin` API.

**`PartitionScheme=huge_app` is required** — the bundled Random Forest (`sleep_classifier.h`, ~1.4 MB of generated code) overflows the default partition's ~1.3 MB app slot. Huge APP gives a 3 MB app slot (no OTA; SD logging unaffected). Correctly applied, the sketch lands at ~42%.

**In the Arduino IDE (GUI):** the FQBN flag above does **not** apply — you must set **Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"** manually, or the build fails with `text section exceeds available space` (100% of 1310720 bytes).

`secrets.h` (gitignored) must exist at `SmartAlarm/secrets.h`:
```cpp
#define WIFI_SSID     "..."
#define WIFI_PASS     "..."
#define OWM_API_KEY   "..."
```

`User_Setup.h` (TFT_eSPI config) must **never be modified**. Relevant pins: MOSI 13, SCLK 14, CS 15, DC 27, RST 26.

## Pin map

| Function | Pin |
|---|---|
| TFT backlight | 32 (PWM via ledcAttach) |
| BACK / sleep button | 33 (ezButton, INPUT_PULLUP) |
| Rotary encoder CLK | 16 (interrupt FALLING, IRAM_ATTR) |
| Rotary encoder DT | 4 |
| Rotary encoder SW | 34 (ezButton, ext pull-up — input-only pin) |
| Buzzer (passive) | 25 (tone/noTone) |
| MPU-6050 SDA/SCL | 21 / 22 |
| SD card CS | 5 |

## Architecture

### Screen state machine
`enum Screen { HOME, MENU, ALARM_MENU, EDIT_ALARM, EDIT_WAKE, SLEEP_DATA, WEATHER, SETTINGS, RESTART, BOOT }` drives all rendering. `dirty = true` triggers a redraw on the next loop tick.

Navigation is handled by three functions:
- `onRotate(int dir)` — encoder turn
- `onPress()` — encoder click (SW_PIN 34)
- `onBack()` / `handleBackButton()` — context-sensitive: sleep toggle at HOME, BACK elsewhere, stops alarm if ringing

### HOME screen
Uses an **incremental redraw** path (`updateDisplay()`): only redraws time/date when the value changes, so there's no flicker. Returning to HOME calls `enterHome()` which full-clears and resets the incremental state buffers (`prevTimeBuf`, `prevDateBuf`).

### Sleep tracking
- **Enter**: `enterSleepMode()` — wipes `/sleep_data.csv` (SD.remove + FILE_WRITE + header) **and** `/sleep_stages.csv`, opens both, resets `sleepClf`, turns backlight off, sets `sleeping = true`
- **Exit**: `exitSleepMode()` — closes **both** files, turns backlight on, returns to HOME
- These are the **only** two points that open/close the SD files. Do not add intermediate close/reopen calls (that was the root cause of SD corruption bugs in the past).
- `logSensorData()` appends one raw row to `/sleep_data.csv` every 500 ms while `sleeping == true`, and feeds the same sample to the classifier.

### Sleep-stage classifier
- `smart_alarm_features.h` (`SmartAlarm::SleepClassifier`) buffers 60 accel samples (30 s @ 2 Hz), computes 21 features (per axis: mean/min/max/rms/std/skew/kurt; **raw values, no gravity removal, population std, scipy-biased skew/kurt**), and runs the Random Forest in `sleep_classifier.h` (`Eloquent::ML::Port::RandomForest::predict`, **0 = deep, 1 = light**). This RF vote is exposed as `lastRaw()` and logged in the `raw` column only.
- The on-device feature definitions **must stay identical** to the Python training script, or the `raw` prediction stops meaning anything.
- **Displayed stage** (`lastSmoothed()` → the `smoothed` column, drives the Sleep Data screen + smart wake) does **not** use the RF vote. The recorded nights are non-separable: deep and light windows have near-identical accelerometer motion (peak-dev p50 140 vs 142, std p50 69.0 vs 69.4), so any RF trained on them collapses to the majority class (light) and the screen showed a flat line. The stage instead comes from an actigraphy-style **sleep-cycle model** in `classifyNow_()`: ~90 min cycles with deep front-loaded and tapering across the night (`CYCLE_WIN`), forced to light for `MOVE_HOLD_WIN` windows after a movement event (peak deviation > `MOVE_THRESH`), and held light for the first `ONSET_WIN` windows. Tuning constants sit at the top of `SleepClassifier`; simulated over the recorded nights it yields ~20–27% deep with realistic cycling. Fixing the ML for real would need better ground-truth labels, not a model change.
- Each completed window appends `timestamp,raw,smoothed` to `/sleep_stages.csv` (1 row / 30 s).
- The Sleep Data screen parses `/sleep_stages.csv` on entry (`sleepSummaryDirty`) into totals + a run-length-encoded two-lane hypnogram; reboot-safe since it reads from SD. When no `/sleep_stages.csv` exists it shows a hardcoded demo hypnogram (`fillDemoSummary()`) that the next real session overwrites.

### Alarm
- `checkAlarm()` runs in both sleep and wake states (no guard on `sleeping`).
- **Smart alarm** (the `wake` config, surfaced in the UI as "Smart Alarm"): while sleeping, if the current time is within `wake.windowMin` minutes before the alarm **and** the classifier has reported light sleep (`sleepClf.lightRun()`) for enough consecutive windows, it rings early. Sensitivity maps to the required run: `required = 4 - wake.sens` (sens 3→1 window, 2→2, 1→3). Fires at most once per session (`smartWakeUsed`) to avoid snooze→refire loops; falls through to the exact-time trigger if light sleep never qualifies.
- `startRinging()` sets `alarmFromSleep` to capture context, then turns backlight on.
- `stopRinging()` calls `exitSleepMode()` if `alarmFromSleep`, otherwise `enterHome()`.
- `snoozeAlarm()` advances `alarmCfg` time by `snoozeDurationMin`, keeps the SD file open (tracking continues), calls `screenSleep()` if alarm fired from sleep.
- Auto-off after `RING_TIMEOUT_MS` (3 min).

### EEPROM layout
Magic byte `0xAB` at addr 0. Alarm config at 1–3, wake config at 4–6, snooze duration at 7. `saveConfig()` is called after every settings edit.

### Weather
`fetchWeather()` — current conditions from OWM `/weather`. `fetchForecast()` — 3-hourly `/forecast` endpoint aggregated into 5-day hi/lo; noon-nearest entry chosen for the daily icon. City is hardcoded to `Calgary,CA` in `OWM_CITY`. Refresh interval: 15 minutes.

### Color palettes
Two palettes coexist:
- `C_*` (legacy): used only on the HOME screen
- `COL_*` (RGB565): used on all nav/sub-screens

### Button bounce protection
`backBtn` has two layers: `setDebounceTime(150)` in `setup()` plus a 400 ms hard lockout (`lastBackActionMs` / `BACK_LOCKOUT_MS`) inside `handleBackButton()`.

## Sleep data CSVs
`ml/data/` contains the **training set**: Nights 1–8 and No Movement, each with columns `timestamp,aX,aY,aZ,label` (labels: `awake`, `light`, `deep`). On-device, the firmware writes `/sleep_data.csv` (raw 2 Hz accel) and `/sleep_stages.csv` (`timestamp,raw,smoothed` predictions, 1/30 s) to the SD card. Night 7 ends at ~09:40 due to a suspected brownout (device reset mid-session). The `AlarmCfg` struct is named `alarmCfg` (not `alarm`) to avoid a collision with libc's `alarm()` function.