#define PIR_PIN 23

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  
  Serial.println("Waiting 30 seconds for PIR to warm up...");
  Serial.println("Do not move in front of it during this time.");
  
  for (int i = 30; i > 0; i--) {
    Serial.print("Ready in: ");
    Serial.print(i);
    Serial.println(" seconds");
    delay(1000);
  }
  
  Serial.println("PIR is ready!");
}

void loop() {
  int pirState = digitalRead(PIR_PIN);
  
  if (pirState == HIGH) {
    Serial.println("MOTION DETECTED — Room: OCCUPIED");
  } else {
    Serial.println("No motion       — Room: EMPTY");
  }
  
  delay(500);
}  
