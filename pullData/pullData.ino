#include <SPIFFS.h>

void setup() {
  Serial.begin(115200);
  delay(5000);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
    return;
  }

  Serial.println("Files on SPIFFS:");
  File root = SPIFFS.open("/");
  File f = root.openNextFile();
  while (f) {
    Serial.print(f.name());
    Serial.print(" - ");
    Serial.print(f.size());
    Serial.println(" bytes");
    f = root.openNextFile();
  }

  File file = SPIFFS.open("/sleep_data.csv", FILE_READ);
  if (!file) {
    Serial.println("No file found");
    return;
  }

  while (file.available()) {
    Serial.println(file.readStringUntil('\n'));
  }
  file.close();
  Serial.println("Done");
}

void loop() {}