#include "DHT.h"
#include <WiFi.h>
#include <PubSubClient.h>

// ── WiFi credentials ──────────────────────────────
const char* ssid     = "";
const char* password = "12345678ok";

// ── MQTT broker ───────────────────────────────────
const char* mqttServer   = "10.130.125.189";
const int   mqttPort     = 1883;
const char* mqttUser     = "admin";
const char* mqttPassword = "01001684495";

// ── MQTT topics ───────────────────────────────────
const char* topicTemp     = "smarthome/room001/temperature";
const char* topicHum      = "smarthome/room001/humidity";
const char* topicRoom     = "smarthome/room001/room";
const char* topicCommand  = "smarthome/room001/command";

// ── Pin definitions ───────────────────────────────
#define DHT_PIN  17
#define DHT_TYPE DHT22
#define PIR_PIN  23

// ── Objects ───────────────────────────────────────
DHT dht(DHT_PIN, DHT_TYPE);
WiFiClient   espClient;
PubSubClient mqtt(espClient);

// ── State ─────────────────────────────────────────
bool roomOccupied  = false;
bool lastOccupied  = false;
unsigned long lastDHTRead    = 0;
unsigned long lastMQTTRetry  = 0;

// ── Called when a command arrives from your phone ──
void onCommandReceived(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  Serial.print("Command received: ");
  Serial.println(message);

  if (message == "LIGHTS_ON") {
    Serial.println(">>> Lights ON command received");
  }
  else if (message == "LIGHTS_OFF") {
    Serial.println(">>> Lights OFF command received");
  }
  else if (message == "STATUS") {
    mqtt.publish(topicRoom, roomOccupied ? "OCCUPIED" : "EMPTY");
  }
}

// ── Connect to WiFi ───────────────────────────────
void connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;
    if (attempts > 30) {
      Serial.println("\nWiFi failed. Check credentials.");
      return;
    }
  }

  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// ── Connect to MQTT broker ────────────────────────
void connectMQTT() {
  mqtt.setServer(mqttServer, mqttPort);
  mqtt.setCallback(onCommandReceived);

  Serial.print("Connecting to MQTT broker...");

  String clientID = "ESP32-" + String(WiFi.macAddress());

  if (mqtt.connect(clientID.c_str(), mqttUser, mqttPassword)) {
    Serial.println(" connected!");
    mqtt.subscribe(topicCommand);
    Serial.print("Subscribed to: ");
    Serial.println(topicCommand);
  } else {
    Serial.print(" failed, rc=");
    Serial.println(mqtt.state());
  }
}

// ── Setup ─────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  dht.begin();

  Serial.println("=== Smart Home System ===");

  connectWiFi();
  connectMQTT();

  delay(2000);
  Serial.println("System ready.");
}

// ── Main loop ─────────────────────────────────────
void loop() {
  unsigned long now = millis();

  if (!mqtt.connected()) {
    if (now - lastMQTTRetry > 5000) {
      lastMQTTRetry = now;
      Serial.println("MQTT disconnected. Reconnecting...");
      connectMQTT();
    }
  }
  mqtt.loop();

  // ── PIR check ──
  roomOccupied = (digitalRead(PIR_PIN) == HIGH);

  if (roomOccupied && !lastOccupied) {
    Serial.println(">>> SOMEONE ENTERED");
    mqtt.publish(topicRoom, "OCCUPIED");
  }
  else if (!roomOccupied && lastOccupied) {
    Serial.println("<<< ROOM EMPTY");
    mqtt.publish(topicRoom, "EMPTY");
  }
  lastOccupied = roomOccupied;

  // ── DHT22: read and publish every 5 seconds ──
  if (now - lastDHTRead >= 5000) {
    lastDHTRead = now;

    float humidity    = dht.readHumidity();
    float temperature = dht.readTemperature();

    if (!isnan(humidity) && !isnan(temperature)) {
      char tempStr[8];
      char humStr[8];
      dtostrf(temperature, 4, 1, tempStr);
      dtostrf(humidity,    4, 1, humStr);

      mqtt.publish(topicTemp, tempStr);
      mqtt.publish(topicHum,  humStr);

      Serial.print("Published → Temp: ");
      Serial.print(tempStr);
      Serial.print(" C  |  Hum: ");
      Serial.print(humStr);
      Serial.print(" %  |  Room: ");
      Serial.println(roomOccupied ? "OCCUPIED" : "EMPTY");
    }
  }

  delay(500);
}
