#include <TFT_eSPI.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPIFFS.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <FS.h> 

// ── config ────────────────────────────────────────────────
const char* ssid     = "dlink-AZ";
const char* password = "6060606060";
const char* OWM_KEY  = "08717755251ba489e0e875d873a13fa1";
const char* OWM_CITY = "Calgary,CA";

// ── colors (RGB565) ───────────────────────────────────────
#define C_OFFWHITE  0xDEDB   // warm near-white
#define C_DIM       0x7BEF   // ~50% grey  (day name)
#define C_DIMMER    0x4208   // ~25% grey  (AM/PM, bottom bar)
#define C_CYAN      0x8EFF   // #8fd0ff    (date)

// ── globals ───────────────────────────────────────────────
TFT_eSPI tft = TFT_eSPI();
MPU6050  mpu;

String  weatherStr   = "CLEAR · --°";
bool    weatherDirty = true;

char prevTimeBuf[6]  = "";
char prevDateBuf[12] = "";

fs::File dataFile;
int  writeCount  = 0;

unsigned long lastSensorMs  = 0;
unsigned long lastWeatherMs = 0;
const unsigned long WEATHER_MS = 15UL * 60UL * 1000UL;

// ── weather ───────────────────────────────────────────────
void fetchWeather() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  String url = "http://api.openweathermap.org/data/2.5/weather?q="
               + String(OWM_CITY) + "&appid=" + OWM_KEY + "&units=metric";
  http.begin(url);
  int code = http.GET();

  if (code == 200) {
    String payload = http.getString();
    DynamicJsonDocument doc(1024);
    DeserializationError err = deserializeJson(doc, payload);
    if (!err) {
      int    temp = (int)round(doc["main"]["temp"].as<float>());
      String desc = doc["weather"][0]["main"].as<String>();
      desc.toUpperCase();
      weatherStr   = desc + " · " + String(temp) + char(167); // 167 = °
      weatherDirty = true;
    }
  }
  http.end();
}

// ── static elements (drawn once) ─────────────────────────
void drawStaticElements() {
  // thin divider above bottom bar
  tft.drawFastHLine(14, 282, 212, 0x2104);
}

// ── bottom bar ───────────────────────────────────────────
void redrawBottomBar() {
  tft.fillRect(0, 287, 240, 33, TFT_BLACK);
  tft.setTextColor(C_DIMMER, TFT_BLACK);

  // left: weather
  tft.drawString(weatherStr, 14, 302, 2);

  // right: alarm (static placeholder until buttons wired)
  String almStr = "ALARM: 6:30 AM";
  int aw = tft.textWidth(almStr, 2);
  tft.drawString(almStr, 226 - aw, 302, 2);

  weatherDirty = false;
}

// ── display update (called every second) ─────────────────
void updateDisplay() {
  struct tm ti;
  if (!getLocalTime(&ti)) return;

  // date stack — only redraws when date changes
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

  // hero time — only redraws when minute changes
  int h = ti.tm_hour % 12;
  if (h == 0) h = 12;
  bool isPM = ti.tm_hour >= 12;

  char timeBuf[6];
  sprintf(timeBuf, "%d:%02d", h, ti.tm_min);

  if (strcmp(timeBuf, prevTimeBuf) != 0) {
    strcpy(prevTimeBuf, timeBuf);

    tft.fillRect(0, 95, 240, 80, TFT_BLACK);

    tft.setTextColor(C_OFFWHITE, TFT_BLACK);
    int tw = tft.textWidth(timeBuf, 7);
    tft.drawString(timeBuf, (240 - tw) / 2, 100, 7);

    // AM/PM label
    tft.fillRect(0, 156, 240, 16, TFT_BLACK);
    tft.setTextColor(C_DIMMER, TFT_BLACK);
    const char* ampm = isPM ? "PM" : "AM";
    int aw = tft.textWidth(ampm, 2);
    tft.drawString(ampm, (240 - aw) / 2, 158, 2);
  }

  if (weatherDirty) redrawBottomBar();
}

// ── sensor logging ────────────────────────────────────────
void logSensorData() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  struct tm ti;
  char ts[25];
  if (getLocalTime(&ti)) strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &ti);
  else strcpy(ts, "time_error");

  if (dataFile) {
    dataFile.print(ts);  dataFile.print(",");
    dataFile.print(ax);  dataFile.print(",");
    dataFile.print(ay);  dataFile.print(",");
    dataFile.println(az);
    if (++writeCount >= 10) { dataFile.flush(); writeCount = 0; }
  }

  Serial.print(ts);
  Serial.print(" aX:"); Serial.print(ax);
  Serial.print(" aY:"); Serial.print(ay);
  Serial.print(" aZ:"); Serial.println(az);
}

// ── setup ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(C_DIM, TFT_BLACK);
  tft.drawCentreString("connecting...", 120, 150, 2);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTime(-6 * 3600, 0, "pool.ntp.org", "time.google.com");
  tft.drawCentreString("syncing time...  ", 120, 150, 2);
  struct tm ti;
  int attempts = 0;
  while (!getLocalTime(&ti) && attempts++ < 20) delay(1000);

  Wire.begin();
  mpu.initialize();

  if (SPIFFS.begin(true)) {
    SPIFFS.remove("/sleep_data.csv");
    dataFile = SPIFFS.open("/sleep_data.csv", FILE_APPEND);
  }

  tft.fillScreen(TFT_BLACK);
  drawStaticElements();
  fetchWeather();
  lastWeatherMs = millis();
}

// ── loop ──────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  if (now - lastSensorMs >= 1000) {
    lastSensorMs = now;
    updateDisplay();
    logSensorData();
  }

  if (now - lastWeatherMs >= WEATHER_MS) {
    lastWeatherMs = now;
    fetchWeather();
  }
}