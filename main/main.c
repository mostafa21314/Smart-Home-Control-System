#include "DHT.h"

#define DHT_PIN   17
#define DHT_TYPE  DHT22
#define PIR_PIN   23

DHT dht(DHT_PIN, DHT_TYPE);

bool roomOccupied    = false;
bool lastOccupied    = false;
unsigned long lastDHTRead = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  dht.begin();

  Serial.println("=== Smart Home Presence System ===");
  Serial.println("Warming up sensors...");
  delay(3000);
  Serial.println("System ready.\n");
}

void loop() {
  unsigned long now = millis();

  // ── PIR: check every loop ──
  roomOccupied = (digitalRead(PIR_PIN) == HIGH);

  // ── Detect state change (entry/exit event) ──
  if (roomOccupied && !lastOccupied) {
    Serial.println(">>> SOMEONE ENTERED — turning on lights/atomizer");
    // relay control will go here later
  }
  else if (!roomOccupied && lastOccupied) {
    Serial.println("<<< ROOM EMPTY — turning off everything");
    // relay control will go here later
  }
  lastOccupied = roomOccupied;

  // ── DHT22: read every 3 seconds ──
  if (now - lastDHTRead >= 3000) {
    lastDHTRead = now;

    float humidity    = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (!isnan(humidity) && !isnan(temperature)) {
      Serial.print("Temp: ");
      Serial.print(temperature, 1);
      Serial.print(" C  |  Hum: ");
      Serial.print(humidity, 1);
      Serial.print(" %  |  Room: ");
      Serial.println(roomOccupied ? "OCCUPIED" : "EMPTY");
    }
  }

  delay(500);
}
Expected output:
