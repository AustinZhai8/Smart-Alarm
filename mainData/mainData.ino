#include <TFT_eSPI.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ezButton.h>
#include "secrets.h"

// Pins
#define BL_PIN     32
#define BTN_PIN    33    // sleep / BACK (context-sensitive)
#define SD_CS       5
#define BUZZER_PIN 25
#define CLK_PIN    16    // rotary encoder CLK (interrupt)
#define DT_PIN      4    // rotary encoder DT
#define SW_PIN     34    // rotary encoder switch (click)

// Colors (legacy HOME palette)
#define C_OFFWHITE  0xDEDB
#define C_DIM       0x7BEF
#define C_DIMMER    0x4208
uint16_t C_CYAN;

// Colors (RGB565 palette for nav UI)
#define COL_BG        0x0000  // pure black
#define COL_TEXT      0xF77C  // off-white #F0ECE6
#define COL_ACCENT    0x8E9F  // cyan #8FD0FF
#define COL_DEEP      0x9C5E  // purple #9D8BF0
#define COL_KNOBTEXT  0x08A3  // dark #04121C (text on accent fills)
#define COL_TEXT_75   0xB596
#define COL_TEXT_55   0x8410
#define COL_TEXT_40   0x630C
#define COL_TEXT_30   0x4A49
#define COL_HAIRLINE  0x18E3  // dim divider
#define COL_DIMSLOT   0x10A2  // very dim slot backgrounds

// Config
const char* OWM_CITY = "Calgary,CA";

// Objects
TFT_eSPI tft = TFT_eSPI();
MPU6050  mpu;
ezButton encBtn(SW_PIN);
ezButton backBtn(BTN_PIN);

// ── navigation ────────────────────────────────────────────
enum Screen {
  HOME, MENU, ALARM_MENU, EDIT_ALARM, EDIT_WAKE,
  SLEEP_DATA, WEATHER, SETTINGS, RESTART, BOOT
};
Screen screen = HOME;
bool   dirty  = true;   // redraw requested

struct AlarmCfg { bool on; uint8_t hour, minute; };       // 24h
struct WakeCfg  { bool on; uint8_t windowMin, sens; };     // 5..60 / 1..3
AlarmCfg alarmCfg = { true, 6, 30 };
WakeCfg  wake  = { true, 30, 2 };
uint8_t  snoozeDurationMin = 10;   // Settings: snooze length (1..30)

const char* MENU_ITEMS[6] = {
  "Enter Sleep Mode", "Alarm", "Sleep Data", "Weather", "Settings", "Restart"
};
const char* ALARM_ITEMS[2] = { "Edit Alarm", "Edit Wake Up" };

uint8_t menuSel    = 1;   // start on "Alarm"
uint8_t alarmSel   = 0;
uint8_t alarmField = 0;   // 0=enabled 1=h 2=m
uint8_t wakeField  = 1;   // 0=enabled 1=minutes 2=sens
bool    rChoiceYes = false;

// alarm ring state
bool          ringing       = false;
unsigned long lastBeepMs    = 0;
bool          alarmFlashOn  = false;
int           lastCheckMin  = -1;
bool          alarmFromSleep = false;   // did the alarm fire while in sleep mode?
unsigned long ringStartMs   = 0;        // when the current ring began
#define RING_TIMEOUT_MS (3UL * 60UL * 1000UL)   // auto-off after 3 min unanswered

// ── encoder ISR ───────────────────────────────────────────
volatile int  counter   = 0;
volatile int  direction = 0;
volatile unsigned long lastTime = 0;

void IRAM_ATTR ISR_encoder() {
  if ((millis() - lastTime) < 50) return;
  lastTime = millis();
  if (digitalRead(DT_PIN) == HIGH) { counter++; direction = 1; }
  else                             { counter--; direction = -1; }
}

// State
bool     sleeping     = false;
String   weatherStr   = "CLEAR · --°";
bool     weatherDirty = true;
char     prevTimeBuf[6]  = "";
char     prevDateBuf[12] = "";
File     dataFile;
int      writeCount   = 0;

// current weather detail (parsed from fetchWeather)
int   wxTemp = 0, wxHi = 0, wxLo = 0, wxFeels = 0, wxHumidity = 0, wxWind = 0;
char  wxCond[16] = "Clear";

// 5-day forecast
struct DayWx { char wd[4]; uint8_t icon; int hi, lo; };  // icon: 0 sun 1 partly 2 cloud 3 rain
DayWx forecast[5];
bool  forecastReady = false;

unsigned long lastSensorMs  = 0;
unsigned long lastWeatherMs = 0;
const unsigned long WEATHER_MS = 15UL * 60UL * 1000UL;

// ── EEPROM ────────────────────────────────────────────────
#define EE_MAGIC_ADDR  0
#define EE_MAGIC_VAL   0xAB
#define EE_ALARM_ADDR  1   // on, hour, minute
#define EE_WAKE_ADDR   4   // on, windowMin, sens
#define EE_SNOOZE_ADDR 7   // snoozeDurationMin

void saveConfig() {
  EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VAL);
  EEPROM.write(EE_ALARM_ADDR + 0, alarmCfg.on);
  EEPROM.write(EE_ALARM_ADDR + 1, alarmCfg.hour);
  EEPROM.write(EE_ALARM_ADDR + 2, alarmCfg.minute);
  EEPROM.write(EE_WAKE_ADDR + 0, wake.on);
  EEPROM.write(EE_WAKE_ADDR + 1, wake.windowMin);
  EEPROM.write(EE_WAKE_ADDR + 2, wake.sens);
  EEPROM.write(EE_SNOOZE_ADDR, snoozeDurationMin);
  EEPROM.commit();
}

void loadConfig() {
  EEPROM.begin(32);
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VAL) {
    saveConfig();   // first boot: write defaults
    return;
  }
  alarmCfg.on     = EEPROM.read(EE_ALARM_ADDR + 0);
  alarmCfg.hour   = EEPROM.read(EE_ALARM_ADDR + 1);
  alarmCfg.minute = EEPROM.read(EE_ALARM_ADDR + 2);
  wake.on        = EEPROM.read(EE_WAKE_ADDR + 0);
  wake.windowMin = EEPROM.read(EE_WAKE_ADDR + 1);
  wake.sens      = EEPROM.read(EE_WAKE_ADDR + 2);
  snoozeDurationMin = constrain((int)EEPROM.read(EE_SNOOZE_ADDR), 1, 30);
}

// ── weather ───────────────────────────────────────────────
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.setTimeout(5000);
  String url = "http://api.openweathermap.org/data/2.5/weather?q="
               + String(OWM_CITY) + "&appid=" + String(OWM_API_KEY) + "&units=metric";
  http.begin(url);
  int code = http.GET();
  if (code == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    if (!deserializeJson(doc, payload)) {
      int    temp = (int)round(doc["main"]["temp"].as<float>());
      String desc = doc["weather"][0]["main"].as<String>();
      wxTemp     = temp;
      wxHi       = (int)round(doc["main"]["temp_max"].as<float>());
      wxLo       = (int)round(doc["main"]["temp_min"].as<float>());
      wxFeels    = (int)round(doc["main"]["feels_like"].as<float>());
      wxHumidity = doc["main"]["humidity"].as<int>();
      wxWind     = (int)round(doc["wind"]["speed"].as<float>() * 3.6);  // m/s -> km/h
      strncpy(wxCond, desc.c_str(), sizeof(wxCond) - 1);
      wxCond[sizeof(wxCond) - 1] = '\0';
      desc.toUpperCase();
      weatherStr   = desc + " · " + String(temp) + "°";
      weatherDirty = true;
    }
  }
  http.end();
}

// Map an OWM "main" condition string to our icon index.
uint8_t condToIcon(const char* main) {
  if (strstr(main, "Rain") || strstr(main, "Drizzle") || strstr(main, "Thunder")) return 3;
  if (strstr(main, "Cloud")) {
    if (strstr(main, "few") || strstr(main, "scattered")) return 1;
    return 2;
  }
  if (strstr(main, "Clear")) return 0;
  return 1;  // mist/haze/snow fallback -> partly
}

// Fetch /forecast (3-hourly) and aggregate into 5 daily hi/lo + noon condition.
void fetchForecast() {
  if (WiFi.status() != WL_CONNECTED) return;
  HTTPClient http;
  http.setTimeout(6000);
  String url = "http://api.openweathermap.org/data/2.5/forecast?q="
               + String(OWM_CITY) + "&appid=" + String(OWM_API_KEY) + "&units=metric";
  http.begin(url);
  int code = http.GET();
  if (code != 200) { http.end(); return; }

  // Keep only the fields we need so the doc stays small.
  StaticJsonDocument<200> filter;
  filter["list"][0]["dt"] = true;
  filter["list"][0]["main"]["temp_max"] = true;
  filter["list"][0]["main"]["temp_min"] = true;
  filter["list"][0]["weather"][0]["main"] = true;

  DynamicJsonDocument doc(8192);
  DeserializationError err = deserializeJson(doc, http.getStream(),
                                             DeserializationOption::Filter(filter));
  http.end();
  if (err) return;

  JsonArray list = doc["list"].as<JsonArray>();

  // Aggregate by calendar day. Track up to 5 distinct upcoming days.
  long    dayKey[5];
  int     dHi[5], dLo[5];
  long    dNoonDelta[5];   // |seconds from local noon| of the chosen icon entry
  uint8_t dIcon[5];
  int     nDays = 0;

  for (JsonObject item : list) {
    long dt = item["dt"].as<long>();
    time_t t = (time_t)dt;
    struct tm lt;
    localtime_r(&t, &lt);   // applies the timezone set by configTime()

    // local-day key + seconds-from-noon, straight from the local fields
    long key = (long)(lt.tm_year) * 1000 + lt.tm_yday;
    long secOfDay = (long)lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec;
    long noonDelta = labs(secOfDay - 43200L);

    int hi = (int)round(item["main"]["temp_max"].as<float>());
    int lo = (int)round(item["main"]["temp_min"].as<float>());
    const char* mn = item["weather"][0]["main"] | "Clear";

    int idx = -1;
    for (int i = 0; i < nDays; i++) if (dayKey[i] == key) { idx = i; break; }
    if (idx < 0) {
      if (nDays >= 5) continue;
      idx = nDays++;
      dayKey[idx] = key;
      dHi[idx] = hi; dLo[idx] = lo;
      dNoonDelta[idx] = noonDelta; dIcon[idx] = condToIcon(mn);
      strftime(forecast[idx].wd, sizeof(forecast[idx].wd), "%a", &lt);
      for (int c = 0; forecast[idx].wd[c]; c++) forecast[idx].wd[c] = toupper(forecast[idx].wd[c]);
    } else {
      if (hi > dHi[idx]) dHi[idx] = hi;
      if (lo < dLo[idx]) dLo[idx] = lo;
      if (noonDelta < dNoonDelta[idx]) { dNoonDelta[idx] = noonDelta; dIcon[idx] = condToIcon(mn); }
    }
  }

  for (int i = 0; i < nDays; i++) {
    forecast[i].hi = dHi[i];
    forecast[i].lo = dLo[i];
    forecast[i].icon = dIcon[i];
  }
  for (int i = nDays; i < 5; i++) { strcpy(forecast[i].wd, "--"); forecast[i].hi = forecast[i].lo = 0; forecast[i].icon = 1; }
  forecastReady = (nDays > 0);
}

// ── HOME display (existing incremental path) ──────────────
void drawStaticElements() {
  tft.drawFastHLine(14, 282, 212, 0x2104);
}

void redrawBottomBar() {
  tft.fillRect(0, 287, 240, 33, TFT_BLACK);
  tft.setTextColor(C_DIMMER, TFT_BLACK);
  tft.drawString(weatherStr, 14, 302, 2);
  char almStr[20];
  if (alarmCfg.on) {
    int h = alarmCfg.hour % 12; if (h == 0) h = 12;
    sprintf(almStr, "ALARM: %d:%02d %s", h, alarmCfg.minute, alarmCfg.hour >= 12 ? "PM" : "AM");
  } else {
    strcpy(almStr, "ALARM: OFF");
  }
  tft.drawString(almStr, 226 - tft.textWidth(almStr, 2), 302, 2);
  weatherDirty = false;
}

void updateDisplay() {
  struct tm ti;
  if (!getLocalTime(&ti)) return;

  char dateBuf[12];
  strftime(dateBuf, sizeof(dateBuf), "%d %b", &ti);
  for (int i = 0; dateBuf[i]; i++) dateBuf[i] = toupper(dateBuf[i]);

  if (strcmp(dateBuf, prevDateBuf) != 0) {
    strcpy(prevDateBuf, dateBuf);
    char dayBuf[12];
    strftime(dayBuf, sizeof(dayBuf), "%A", &ti);
    for (int i = 0; dayBuf[i]; i++) dayBuf[i] = toupper(dayBuf[i]);
    tft.fillRect(0, 18, 200, 32, TFT_BLACK);
    tft.setTextColor(C_DIM, TFT_BLACK);
    tft.drawString(dayBuf, 16, 20, 2);
    tft.setTextColor(C_CYAN, TFT_BLACK);
    tft.drawString(dateBuf, 16, 34, 2);
  }

  int h = ti.tm_hour % 12;
  if (h == 0) h = 12;
  bool isPM = ti.tm_hour >= 12;
  char timeBuf[6];
  sprintf(timeBuf, "%d:%02d", h, ti.tm_min);

  if (strcmp(timeBuf, prevTimeBuf) != 0) {
    strcpy(prevTimeBuf, timeBuf);
    tft.fillRect(0, 95, 240, 80, TFT_BLACK);
    tft.setTextColor(C_OFFWHITE, TFT_BLACK);
    tft.drawString(timeBuf, (240 - tft.textWidth(timeBuf, 7)) / 2, 100, 7);
    tft.fillRect(0, 156, 240, 16, TFT_BLACK);
    tft.setTextColor(C_DIMMER, TFT_BLACK);
    const char* ampm = isPM ? "PM" : "AM";
    tft.drawString(ampm, (240 - tft.textWidth(ampm, 2)) / 2, 158, 2);
  }

  if (weatherDirty) redrawBottomBar();
}

// Reset HOME and draw it fresh (used when returning from a menu).
void enterHome() {
  screen = HOME;
  tft.fillScreen(TFT_BLACK);
  drawStaticElements();
  prevTimeBuf[0] = '\0';
  prevDateBuf[0] = '\0';
  weatherDirty   = true;
  updateDisplay();
}

// ── small drawing helpers ─────────────────────────────────
// Degree symbol (built-in fonts lack it) as a small ring.
void drawDegree(int x, int y, uint16_t color) {
  tft.drawCircle(x, y, 2, color);
}

void drawSubHeader(const char* title) {
  tft.fillRect(0, 0, 240, 44, TFT_BLACK);
  char up[24];
  strncpy(up, title, sizeof(up) - 1); up[sizeof(up) - 1] = '\0';
  for (int i = 0; up[i]; i++) up[i] = toupper(up[i]);
  tft.setTextColor(COL_TEXT_30, TFT_BLACK);
  tft.drawString(up, 16, 18, 2);
  tft.drawString("< BACK", 226 - tft.textWidth("< BACK", 2), 20, 2);
  tft.drawFastHLine(16, 44, 208, COL_HAIRLINE);
}

void drawAccentBar(int y, int h) {
  int inset = h / 5;
  tft.fillRect(0, y + inset, 2, h - 2 * inset, COL_ACCENT);
}

void drawToggle(int x, int y, bool on, bool focused) {
  // 38x20 capsule
  uint16_t track = on ? COL_ACCENT : COL_DIMSLOT;
  tft.fillRoundRect(x, y, 38, 20, 10, track);
  uint16_t knob = on ? COL_KNOBTEXT : COL_TEXT_55;
  int kx = on ? x + 28 : x + 10;
  tft.fillCircle(kx, y + 10, 8, knob);
  if (focused) tft.drawRoundRect(x - 2, y - 2, 42, 24, 12, COL_ACCENT);
}

// ── screens ───────────────────────────────────────────────
void clearContent() { tft.fillRect(0, 45, 240, 275, TFT_BLACK); }

void drawMenu() {
  drawSubHeader("Menu");
  clearContent();
  int rowH = 36, y0 = 52;
  for (int i = 0; i < 6; i++) {
    int y = y0 + i * rowH;
    bool sel = (i == menuSel);
    if (sel) drawAccentBar(y, rowH);
    tft.setTextColor(sel ? COL_ACCENT : COL_TEXT_55, TFT_BLACK);
    tft.drawString(MENU_ITEMS[i], 22, y + (rowH - 16) / 2, 2);
  }
  // scrollbar
  tft.fillRect(234, 50, 2, 264, COL_DIMSLOT);
  tft.fillRect(234, 50, 2, 264, COL_TEXT_30);
}

void drawAlarmMenu() {
  drawSubHeader("Alarm");
  clearContent();
  int rowH = 44, y0 = 58;
  bool status[2] = { alarmCfg.on, wake.on };
  for (int i = 0; i < 2; i++) {
    int y = y0 + i * rowH;
    bool sel = (i == alarmSel);
    if (sel) drawAccentBar(y, rowH);
    tft.setTextColor(sel ? COL_ACCENT : COL_TEXT_55, TFT_BLACK);
    tft.drawString(ALARM_ITEMS[i], 22, y + (rowH - 16) / 2, 2);
    const char* st = status[i] ? "ON" : "OFF";
    tft.setTextColor(status[i] ? COL_ACCENT : COL_TEXT_30, TFT_BLACK);
    tft.drawString(st, 220 - tft.textWidth(st, 2), y + (rowH - 16) / 2, 2);
  }
}

void drawEditAlarm() {
  drawSubHeader("Edit Alarm");
  clearContent();
  // Alarm row + toggle
  if (alarmField == 0) drawAccentBar(52, 28);
  tft.setTextColor(alarmField == 0 ? COL_ACCENT : COL_TEXT_75, TFT_BLACK);
  tft.drawString("Alarm", 22, 58, 2);
  drawToggle(180, 56, alarmCfg.on, alarmField == 0);
  tft.drawFastHLine(18, 92, 204, COL_HAIRLINE);

  // Big HH:MM (24h), Font 7
  char hh[3], mm[3];
  sprintf(hh, "%02d", alarmCfg.hour);
  sprintf(mm, "%02d", alarmCfg.minute);
  uint16_t base = alarmCfg.on ? COL_TEXT : COL_TEXT_40;
  int wH = tft.textWidth(hh, 7), wC = tft.textWidth(":", 7), wM = tft.textWidth(mm, 7);
  int total = wH + wC + wM;
  int x = (240 - total) / 2, y = 120;
  tft.setTextColor(alarmField == 1 ? COL_ACCENT : base, TFT_BLACK);
  tft.drawString(hh, x, y, 7);
  if (alarmField == 1 && alarmCfg.on) tft.fillRect(x, y + 56, wH, 2, COL_ACCENT);
  tft.setTextColor(base, TFT_BLACK);
  tft.drawString(":", x + wH, y, 7);
  tft.setTextColor(alarmField == 2 ? COL_ACCENT : base, TFT_BLACK);
  tft.drawString(mm, x + wH + wC, y, 7);
  if (alarmField == 2 && alarmCfg.on) tft.fillRect(x + wH + wC, y + 56, wM, 2, COL_ACCENT);

  tft.setTextColor(COL_TEXT_30, TFT_BLACK);
  tft.drawCentreString("TURN: ADJUST  PRESS: NEXT", 120, 300, 2);
}

void drawEditWake() {
  drawSubHeader("Edit Wake Up");
  clearContent();
  if (wakeField == 0) drawAccentBar(52, 28);
  tft.setTextColor(wakeField == 0 ? COL_ACCENT : COL_TEXT_75, TFT_BLACK);
  tft.drawString("Wake Up", 22, 58, 2);
  drawToggle(180, 56, wake.on, wakeField == 0);
  tft.drawFastHLine(18, 92, 204, COL_HAIRLINE);

  // minutes number (Font 7) + MIN
  char mb[4]; sprintf(mb, "%d", wake.windowMin);
  uint16_t base = wake.on ? COL_TEXT : COL_TEXT_40;
  int wN = tft.textWidth(mb, 7);
  int x = (240 - wN - 30) / 2, y = 110;
  tft.setTextColor(wakeField == 1 ? COL_ACCENT : base, TFT_BLACK);
  tft.drawString(mb, x, y, 7);
  if (wakeField == 1 && wake.on) tft.fillRect(x, y + 56, wN, 2, COL_ACCENT);
  tft.setTextColor(COL_TEXT_40, TFT_BLACK);
  tft.drawString("MIN", x + wN + 6, y + 36, 2);
  tft.setTextColor(wake.on ? COL_TEXT_40 : COL_TEXT_30, TFT_BLACK);
  tft.drawCentreString("Wake window before alarm", 120, 178, 2);

  // sensitivity chips
  tft.setTextColor(wakeField == 2 ? COL_ACCENT : COL_TEXT_75, TFT_BLACK);
  tft.drawString("Sensitivity", 22, 214, 2);
  int cx = 150;
  for (int n = 1; n <= 3; n++) {
    int bx = cx + (n - 1) * 24;
    bool active = (n == wake.sens);
    tft.fillRoundRect(bx, 210, 18, 18, 5, active ? COL_ACCENT : COL_DIMSLOT);
    if (active && wakeField == 2) tft.drawRoundRect(bx - 1, 209, 20, 20, 6, COL_ACCENT);
    char d[2] = { (char)('0' + n), '\0' };
    tft.setTextColor(active ? COL_KNOBTEXT : COL_TEXT_40);  // transparent bg over chip fill
    tft.drawCentreString(d, bx + 9, 212, 2);
  }

  tft.setTextColor(COL_TEXT_30, TFT_BLACK);
  tft.drawCentreString("TURN: ADJUST  PRESS: NEXT", 120, 300, 2);
}

void drawSleepData() {
  drawSubHeader("Sleep Data");
  clearContent();
  tft.setTextColor(COL_TEXT, TFT_BLACK);
  tft.drawString("7h 24m", 18, 58, 4);
  tft.setTextColor(COL_TEXT_40, TFT_BLACK);
  tft.drawString("LAST NIGHT", 120, 64, 2);

  // segments: state(0=light,1=deep) + weight
  const uint8_t seg_s[9] = { 0, 1, 0, 1, 0, 1, 0, 1, 0 };
  const float   seg_w[9] = { 2, 3, 1.5, 2.5, 2, 1.5, 1.5, 2, 2.5 };
  float totalW = 0; for (int i = 0; i < 9; i++) totalW += seg_w[i];
  int x0 = 18, w = 204, gap = 2;
  float avail = w - gap * 8;

  // light lane (top), deep lane (bottom)
  int yL = 110, yD = 142, hh = 26;
  float fx = x0;
  for (int i = 0; i < 9; i++) {
    int bw = (int)(avail * (seg_w[i] / totalW));
    tft.fillRoundRect((int)fx, yL, bw, hh, 3, seg_s[i] == 0 ? COL_ACCENT : COL_DIMSLOT);
    tft.fillRoundRect((int)fx, yD, bw, hh, 3, seg_s[i] == 1 ? COL_DEEP : COL_DIMSLOT);
    fx += bw + gap;
  }

  tft.setTextColor(COL_TEXT_40, TFT_BLACK);
  tft.drawString("23:42", x0, yD + hh + 6, 2);
  tft.drawString("06:30", x0 + w - tft.textWidth("06:30", 2), yD + hh + 6, 2);

  // legend
  tft.fillRoundRect(40, 210, 8, 8, 2, COL_ACCENT);
  tft.setTextColor(COL_TEXT_75, TFT_BLACK);
  tft.drawString("Light 5h 14m", 54, 208, 2);
  tft.fillRoundRect(40, 232, 8, 8, 2, COL_DEEP);
  tft.drawString("Deep 2h 10m", 54, 230, 2);
}

// minimal line weather icon
void drawWxIcon(int cx, int cy, uint8_t type, uint16_t color) {
  switch (type) {
    case 0: // sun
      tft.drawCircle(cx, cy, 5, color);
      for (int a = 0; a < 360; a += 45) {
        float r = a * 0.01745;
        tft.drawLine(cx + cos(r) * 7, cy + sin(r) * 7, cx + cos(r) * 9, cy + sin(r) * 9, color);
      }
      break;
    case 1: // partly
      tft.drawCircle(cx - 3, cy - 2, 4, color);
      tft.fillRoundRect(cx - 4, cy + 1, 14, 7, 3, color);
      break;
    case 2: // cloud
      tft.fillRoundRect(cx - 7, cy - 1, 16, 8, 4, color);
      tft.fillCircle(cx - 2, cy - 3, 4, color);
      break;
    case 3: // rain
      tft.fillRoundRect(cx - 7, cy - 3, 16, 7, 3, color);
      tft.drawLine(cx - 4, cy + 6, cx - 5, cy + 9, color);
      tft.drawLine(cx,     cy + 6, cx - 1, cy + 9, color);
      tft.drawLine(cx + 4, cy + 6, cx + 3, cy + 9, color);
      break;
  }
}

void drawWeather() {
  drawSubHeader("Weather");
  clearContent();
  // today big temp
  char tb[6]; sprintf(tb, "%d", wxTemp);
  tft.setTextColor(COL_TEXT, TFT_BLACK);
  tft.drawString(tb, 18, 58, 7);
  drawDegree(18 + tft.textWidth(tb, 7) + 6, 64, COL_TEXT);
  tft.setTextColor(COL_ACCENT, TFT_BLACK);
  tft.drawString(wxCond, 20, 120, 2);
  drawWxIcon(200, 80, condToIcon(wxCond), COL_TEXT_75);

  // detail row
  char det[40];
  tft.setTextColor(COL_TEXT_55, TFT_BLACK);
  sprintf(det, "H %d  L %d", wxHi, wxLo);
  tft.drawString(det, 18, 150, 2);
  sprintf(det, "Feels %d", wxFeels);
  tft.drawString(det, 100, 150, 2);
  sprintf(det, "%d%%  %dkm/h", wxHumidity, wxWind);
  tft.drawString(det, 160, 150, 2);

  tft.drawFastHLine(16, 174, 208, COL_HAIRLINE);

  // 5-day columns
  int colW = 240 / 5;
  for (int i = 0; i < 5; i++) {
    int cx = colW * i + colW / 2;
    tft.setTextColor(COL_TEXT_30, TFT_BLACK);
    tft.drawCentreString(forecast[i].wd, cx, 188, 2);
    drawWxIcon(cx, 218, forecast[i].icon, COL_TEXT_55);
    char hl[5];
    tft.setTextColor(COL_TEXT, TFT_BLACK);
    sprintf(hl, "%d", forecast[i].hi);
    tft.drawCentreString(hl, cx, 238, 2);
    tft.setTextColor(COL_TEXT_40, TFT_BLACK);
    sprintf(hl, "%d", forecast[i].lo);
    tft.drawCentreString(hl, cx, 258, 2);
  }
}

void drawSettings() {
  drawSubHeader("Settings");
  clearContent();
  drawAccentBar(58, 28);
  tft.setTextColor(COL_ACCENT, TFT_BLACK);
  tft.drawString("Snooze Duration", 22, 64, 2);

  char mb[4]; sprintf(mb, "%d", snoozeDurationMin);
  int wN = tft.textWidth(mb, 7);
  int x = (240 - wN - 30) / 2, y = 130;
  tft.setTextColor(COL_ACCENT, TFT_BLACK);
  tft.drawString(mb, x, y, 7);
  tft.fillRect(x, y + 56, wN, 2, COL_ACCENT);
  tft.setTextColor(COL_TEXT_40, TFT_BLACK);
  tft.drawString("MIN", x + wN + 6, y + 36, 2);
  tft.drawCentreString("Snooze length", 120, y + 70, 2);

  tft.setTextColor(COL_TEXT_30, TFT_BLACK);
  tft.drawCentreString("TURN: ADJUST  PRESS: BACK", 120, 300, 2);
}

void drawRestart() {
  drawSubHeader("Restart");
  clearContent();
  tft.setTextColor(COL_TEXT, TFT_BLACK);
  tft.drawCentreString("Restart device?", 120, 110, 4);
  tft.setTextColor(COL_TEXT_40, TFT_BLACK);
  tft.drawCentreString("The clock will be unavailable", 120, 150, 2);
  tft.drawCentreString("for a few seconds.", 120, 168, 2);

  // No | Yes buttons
  int by = 210, bh = 40, bw = 90;
  int nx = 24, yx = 240 - 24 - bw;
  // No
  if (!rChoiceYes) { tft.fillRoundRect(nx, by, bw, bh, 6, COL_ACCENT); tft.setTextColor(COL_KNOBTEXT, COL_ACCENT); }
  else             { tft.drawRoundRect(nx, by, bw, bh, 6, COL_TEXT_30); tft.setTextColor(COL_TEXT_55, COL_BG); }
  tft.drawCentreString("No", nx + bw / 2, by + 12, 2);
  // Yes
  if (rChoiceYes)  { tft.fillRoundRect(yx, by, bw, bh, 6, COL_ACCENT); tft.setTextColor(COL_KNOBTEXT, COL_ACCENT); }
  else             { tft.drawRoundRect(yx, by, bw, bh, 6, COL_TEXT_30); tft.setTextColor(COL_TEXT_55, COL_BG); }
  tft.drawCentreString("Yes", yx + bw / 2, by + 12, 2);
}

void drawBoot() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(COL_TEXT_55, TFT_BLACK);
  tft.drawCentreString("Aura Clock", 120, 150, 4);
  tft.drawFastHLine(70, 180, 100, COL_HAIRLINE);
}

void render() {
  switch (screen) {
    case HOME:       updateDisplay();   break;  // incremental, no full clear
    case MENU:       drawMenu();        break;
    case ALARM_MENU: drawAlarmMenu();   break;
    case EDIT_ALARM: drawEditAlarm();   break;
    case EDIT_WAKE:  drawEditWake();    break;
    case SLEEP_DATA: drawSleepData();   break;
    case WEATHER:    drawWeather();     break;
    case SETTINGS:   drawSettings();    break;
    case RESTART:    drawRestart();     break;
    case BOOT:       drawBoot();        break;
  }
}

// ── navigation logic ──────────────────────────────────────
const uint8_t A_FIELDS[3] = { 0, 1, 2 };  // enabled, h, m
const uint8_t W_FIELDS[3] = { 0, 1, 2 };  // enabled, minutes, sens

void onRotate(int dir) {
  switch (screen) {
    case MENU:
      menuSel = (menuSel + dir + 6) % 6;
      dirty = true; break;
    case ALARM_MENU:
      alarmSel = (alarmSel + dir + 2) % 2;
      dirty = true; break;
    case EDIT_ALARM:
      if (alarmField == 0)      alarmCfg.on = !alarmCfg.on;
      else if (alarmField == 1) alarmCfg.hour = (alarmCfg.hour + dir + 24) % 24;
      else                      alarmCfg.minute = (alarmCfg.minute + dir + 60) % 60;
      saveConfig(); dirty = true; break;
    case EDIT_WAKE:
      if (wakeField == 0)       wake.on = !wake.on;
      else if (wakeField == 1)  wake.windowMin = constrain(wake.windowMin + dir * 5, 5, 60);
      else                      wake.sens = ((wake.sens - 1 + dir + 3) % 3) + 1;
      saveConfig(); dirty = true; break;
    case SETTINGS:
      snoozeDurationMin = constrain(snoozeDurationMin + dir, 1, 30);
      saveConfig(); dirty = true; break;
    case RESTART:
      rChoiceYes = !rChoiceYes; dirty = true; break;
    default: break;
  }
}

void onPress() {
  switch (screen) {
    case HOME:
      screen = MENU; dirty = true; break;
    case MENU:
      switch (menuSel) {
        case 0: enterSleepMode(); break;          // stays in sleep mode
        case 1: screen = ALARM_MENU; dirty = true; break;
        case 2: screen = SLEEP_DATA; dirty = true; break;
        case 3: screen = WEATHER;    dirty = true; break;
        case 4: screen = SETTINGS;   dirty = true; break;
        case 5: screen = RESTART; rChoiceYes = false; dirty = true; break;
      }
      break;
    case ALARM_MENU:
      screen = (alarmSel == 0) ? EDIT_ALARM : EDIT_WAKE;
      if (screen == EDIT_ALARM) alarmField = 0; else wakeField = 0;
      dirty = true; break;
    case EDIT_ALARM:
      alarmField = (alarmField + 1) % 3; dirty = true; break;
    case EDIT_WAKE:
      wakeField = (wakeField + 1) % 3; dirty = true; break;
    case SETTINGS:
      screen = MENU; dirty = true; break;        // single field → press returns to Menu
    case RESTART:
      if (rChoiceYes) ESP.restart();
      else { screen = MENU; dirty = true; }
      break;
    default: break;
  }
}

void onBack() {
  switch (screen) {
    case MENU:        enterHome(); break;
    case EDIT_ALARM:
    case EDIT_WAKE:   screen = ALARM_MENU; dirty = true; break;
    case ALARM_MENU:
    case SLEEP_DATA:
    case WEATHER:
    case SETTINGS:
    case RESTART:     screen = MENU; dirty = true; break;
    default: break;  // HOME no-op
  }
}

// ── logging ───────────────────────────────────────────────
void logSensorData() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  struct tm ti;
  char ts[25];
  if (getLocalTime(&ti)) strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &ti);
  else strcpy(ts, "time_error");

  detachInterrupt(digitalPinToInterrupt(CLK_PIN));
  if (dataFile) {
    dataFile.print(ts);  dataFile.print(",");
    dataFile.print(ax);  dataFile.print(",");
    dataFile.print(ay);  dataFile.print(",");
    dataFile.println(az);
    if (++writeCount >= 20) { dataFile.flush(); writeCount = 0; }
  }
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoder, FALLING);
}

// ── sleep mode ────────────────────────────────────────────
void enterSleepMode() {
  sleeping = true;
  screen   = HOME;
  if (dataFile) dataFile.close();
  SD.remove("/sleep_data.csv");
  dataFile = SD.open("/sleep_data.csv", FILE_WRITE);
  dataFile.println("timestamp,aX,aY,aZ");
  writeCount = 0;
  ledcWrite(BL_PIN, 0);
  tft.fillScreen(TFT_BLACK);
  Serial.println("Sleep logging started");
}

void exitSleepMode() {
  sleeping = false;
  if (dataFile) dataFile.close();
  ledcWrite(BL_PIN, 255);
  enterHome();
  Serial.println("Sleep logging stopped");
}

// Put the display back to sleep (backlight off) WITHOUT touching the open log
// file — logging continues on the same handle, exactly like the original.
void screenSleep() {
  screen = HOME;
  ledcWrite(BL_PIN, 0);
  tft.fillScreen(TFT_BLACK);
}

// ── alarm ring ────────────────────────────────────────────
// The alarm NEVER opens/closes/reopens the SD file. The file is opened once in
// enterSleepMode() and closed once in exitSleepMode(); ringing only changes the
// backlight + screen, so a buzzer-time brownout can't corrupt a mid-write file.
void startRinging() {
  alarmFromSleep = sleeping;        // remember whether to return to sleep on snooze
  ledcWrite(BL_PIN, 255);           // backlight on for the ring screen
  enterHome();                      // draw the clock backdrop (file handle untouched)
  ringing      = true;
  lastBeepMs   = 0;
  ringStartMs  = millis();
  alarmFlashOn = false;
}

// Full stop: silence; if we came from sleep, close the log + wake to HOME.
void stopRinging() {
  ringing = false;
  noTone(BUZZER_PIN);
  if (alarmFromSleep) exitSleepMode();   // the one place the file closes
  else                enterHome();
  alarmFromSleep = false;
}

void snoozeAlarm() {
  ringing = false;
  noTone(BUZZER_PIN);
  int total = alarmCfg.hour * 60 + alarmCfg.minute + snoozeDurationMin;
  total %= (24 * 60);
  alarmCfg.hour = total / 60; alarmCfg.minute = total % 60;  // temporary re-arm (not persisted)
  lastCheckMin = -1;                                         // allow re-fire at snoozed time
  if (alarmFromSleep) screenSleep();   // back to sleep; file stays open, logging continues
  else                enterHome();
}

void serviceRinging() {
  unsigned long now = millis();
  if (now - ringStartMs >= RING_TIMEOUT_MS) {   // unanswered too long → auto-off, wake to HOME
    stopRinging();
    return;
  }
  updateDisplay();   // keep the clock (time/date/weather) drawn and current
  if (now - lastBeepMs >= 500) {
    lastBeepMs = now;
    tone(BUZZER_PIN, 1000, 200);
    alarmFlashOn = !alarmFlashOn;
    // Flash the banner in the empty band below AM/PM — does NOT overlap the time.
    tft.fillRect(0, 205, 240, 42, TFT_BLACK);
    if (alarmFlashOn) {
      tft.setTextColor(COL_ACCENT, TFT_BLACK);
      tft.drawCentreString("ALARM", 120, 210, 4);
    }
  }
}

void checkAlarm() {
  // Runs every loop in BOTH sleep and wake states.
  if (ringing || !alarmCfg.on) return;
  struct tm ti;
  if (!getLocalTime(&ti)) return;
  if (ti.tm_sec == 0 && ti.tm_min != lastCheckMin) {
    lastCheckMin = ti.tm_min;
    if (ti.tm_hour == alarmCfg.hour && ti.tm_min == alarmCfg.minute) startRinging();
  }
}

// ── input ─────────────────────────────────────────────────
unsigned long lastBackActionMs = 0;
#define BACK_LOCKOUT_MS 400

void handleBackButton() {
  unsigned long now = millis();
  if (now - lastBackActionMs < BACK_LOCKOUT_MS) return;  // hard lockout vs. bounce
  lastBackActionMs = now;

  if (ringing)         { stopRinging(); return; }     // BACK during alarm = exit now (1 press)
  if (sleeping)        { exitSleepMode(); return; }   // GPIO 33 leaves sleep mode, saves file
  if (screen == HOME)  { enterSleepMode(); return; }  // and starts a fresh sleep session
  onBack();
}

// ── setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  loadConfig();

  ledcAttach(BL_PIN, 5000, 8);
  ledcWrite(BL_PIN, 255);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(CLK_PIN, INPUT);
  pinMode(DT_PIN,  INPUT);
  attachInterrupt(digitalPinToInterrupt(CLK_PIN), ISR_encoder, FALLING);
  encBtn.setDebounceTime(50);
  backBtn.setDebounceTime(150);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  C_CYAN = tft.color565(143, 208, 255);

  drawBoot();

  tft.setTextColor(C_DIM, TFT_BLACK);
  tft.drawCentreString("connecting...", 120, 200, 2);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTime(-6 * 3600, 0, "pool.ntp.org", "time.google.com");
  tft.drawCentreString("syncing time...  ", 120, 200, 2);
  struct tm ti;
  int attempts = 0;
  while (!getLocalTime(&ti) && attempts++ < 20) delay(1000);

  Wire.begin();
  mpu.initialize();

  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed");
    tft.drawCentreString("SD card failed!", 120, 220, 2);
    delay(2000);
  } else {
    Serial.println("SD card ready");
  }

  fetchWeather();
  fetchForecast();
  lastWeatherMs = millis();

  enterHome();
  Serial.println("Setup complete");
}

// ── loop ──────────────────────────────────────────────────
void loop() {
  encBtn.loop();
  backBtn.loop();
  Serial.println(digitalRead(BTN_PIN));

  // encoder rotation (one tick = one action)
  static int lastCounter = 0;
  noInterrupts();
  int c = counter;
  interrupts();
  if (c != lastCounter) {
    int step = (c > lastCounter) ? 1 : -1;
    int ticks = abs(c - lastCounter);
    lastCounter = c;
    if (ringing) {
      snoozeAlarm();                         // any rotation while ringing = snooze
    } else {
      for (int i = 0; i < ticks; i++) onRotate(step);
    }
  }

  // encoder click
  if (encBtn.isPressed()) {
    if (ringing) stopRinging();              // click while ringing = stop
    else         onPress();
  }

  // back / sleep / snooze button
  if (backBtn.isPressed()) handleBackButton();

  unsigned long now = millis();

  if (ringing) {
    serviceRinging();
    return;
  }

  if (now - lastSensorMs >= 500) {
    lastSensorMs = now;
    if (sleeping)            logSensorData();
    else if (screen == HOME) updateDisplay();
  }

  if (dirty && !sleeping) {
    render();
    dirty = false;
  }

  checkAlarm();

  if (!sleeping && now - lastWeatherMs >= WEATHER_MS) {
    lastWeatherMs = now;
    fetchWeather();
    fetchForecast();
    if (screen == HOME) weatherDirty = true;
  }
}
