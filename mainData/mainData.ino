#include <TFT_eSPI.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

// Pins
#define BL_PIN     32
#define BL_CHANNEL 0
#define BTN_PIN    33
#define SD_CS       5
#define BUZZER_PIN 25

// Colors
#define C_OFFWHITE  0xDEDB
#define C_DIM       0x7BEF
#define C_DIMMER    0x4208
uint16_t C_CYAN;

// Config
const char* OWM_CITY = "Calgary,CA";

// Objects
TFT_eSPI tft = TFT_eSPI();
MPU6050  mpu;

// State
bool     sleeping     = false;
String   weatherStr   = "CLEAR · --°";
bool     weatherDirty = true;
char     prevTimeBuf[6]  = "";
char     prevDateBuf[12] = "";
File     dataFile;
int      writeCount   = 0;

unsigned long lastSensorMs  = 0;
unsigned long lastWeatherMs = 0;
const unsigned long WEATHER_MS = 15UL * 60UL * 1000UL;

// Weather
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
      desc.toUpperCase();
      weatherStr   = desc + " · " + String(temp) + "°";
      weatherDirty = true;
    }
  }
  http.end();
}

// Display
void drawStaticElements() {
  tft.drawFastHLine(14, 282, 212, 0x2104);
}

void redrawBottomBar() {
  tft.fillRect(0, 287, 240, 33, TFT_BLACK);
  tft.setTextColor(C_DIMMER, TFT_BLACK);
  tft.drawString(weatherStr, 14, 302, 2);
  String almStr = "ALARM: 6:30 AM";
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

// Logging
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
    if (++writeCount >= 20) { dataFile.flush(); writeCount = 0; }
  }

  Serial.print(ts);
  Serial.print(" aX:"); Serial.print(ax);
  Serial.print(" aY:"); Serial.print(ay);
  Serial.print(" aZ:"); Serial.println(az);
}

// Sleep mode
void enterSleepMode() {
  sleeping = true;
  if (dataFile) dataFile.close();
  SD.remove("/sleep_data.csv");
  dataFile   = SD.open("/sleep_data.csv", FILE_WRITE);
  writeCount = 0;
  ledcWrite(BL_CHANNEL, 0);
  tft.fillScreen(TFT_BLACK);
  Serial.println("Sleep logging started");
}

void exitSleepMode() {
  sleeping = false;
  if (dataFile) dataFile.close();
  ledcWrite(BL_CHANNEL, 255);
  tft.fillScreen(TFT_BLACK);
  drawStaticElements();
  prevTimeBuf[0] = '\0';
  prevDateBuf[0] = '\0';
  weatherDirty   = true;
  Serial.println("Sleep logging stopped");
}

void checkButton() {
  if (digitalRead(BTN_PIN) == LOW) {
    delay(50);
    if (digitalRead(BTN_PIN) == LOW) {
      while (digitalRead(BTN_PIN) == LOW) delay(10);
      sleeping ? exitSleepMode() : enterSleepMode();
    }
  }
}

// Setup
void setup() {
  Serial.begin(115200);

  ledcSetup(BL_CHANNEL, 5000, 8);
  ledcAttachPin(BL_PIN, BL_CHANNEL);
  ledcWrite(BL_CHANNEL, 255);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  C_CYAN = tft.color565(143, 208, 255);

  tft.setTextColor(C_DIM, TFT_BLACK);
  tft.drawCentreString("connecting...", 120, 150, 2);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTime(-6 * 3600, 0, "pool.ntp.org", "time.google.com");
  tft.drawCentreString("syncing time...  ", 120, 150, 2);
  struct tm ti;
  int attempts = 0;
  while (!getLocalTime(&ti) && attempts++ < 20) delay(1000);

  Wire.begin();
  mpu.initialize();

  if (!SD.begin(SD_CS)) {
    Serial.println("SD card failed");
    tft.drawCentreString("SD card failed!", 120, 150, 2);
    delay(2000);
  } else {
    Serial.println("SD card ready");
  }

  tft.fillScreen(TFT_BLACK);
  drawStaticElements();
  fetchWeather();
  lastWeatherMs = millis();

  Serial.println("Setup complete");
}

// Loop
void loop() {
  checkButton();

  unsigned long now = millis();

  if (now - lastSensorMs >= 500) {
    lastSensorMs = now;
    sleeping ? logSensorData() : updateDisplay();
  }

  if (!sleeping && now - lastWeatherMs >= WEATHER_MS) {
    lastWeatherMs = now;
    fetchWeather();
  }
}