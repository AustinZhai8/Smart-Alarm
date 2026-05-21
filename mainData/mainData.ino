#include <Wire.h>
#include <MPU6050.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <time.h>

MPU6050 mpu;

const char* ssid = "dlink-AZ";
const char* password = "6060606060";

File file;
int writeCount = 0;

void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");

  configTime(-6 * 3600, 0, "pool.ntp.org", "time.google.com");
  Serial.print("Syncing time");
  struct tm timeinfo;
  int attempts = 0;
  while (!getLocalTime(&timeinfo) && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  Serial.println(" synced");

  Wire.begin();
  mpu.initialize();
  delay(100);

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  }

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  }

  SPIFFS.remove("/sleep_data.csv"); 

  file = SPIFFS.open("/sleep_data.csv", FILE_APPEND);

  if (!file) {
    Serial.println("Failed to open file");
    return;
  }

  Serial.println("Logging started");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  struct tm timeinfo;
  char timestamp[25];
  if (getLocalTime(&timeinfo)) {
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", &timeinfo);
  } else {
    strcpy(timestamp, "time_error");
  }

  file.print(timestamp);
  file.print(",");
  file.print(ax);
  file.print(",");
  file.print(ay);
  file.print(",");
  file.println(az);

  writeCount++;
  if (writeCount >= 10) {
    file.flush();
    writeCount = 0;
  }

  Serial.print(timestamp);
  Serial.print(" aX:"); Serial.print(ax);
  Serial.print(" aY:"); Serial.print(ay);
  Serial.print(" aZ:"); Serial.println(az);

  delay(1000);
}