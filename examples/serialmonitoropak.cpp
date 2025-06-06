#include <Arduino.h>
#include "robotka.h"

String receivedMessage = "";     // buffer pro příchozí zprávu
unsigned long lastDotTime = 0;   // čas posledního výpisu teček
const unsigned long dotInterval = 1000;  // interval 1 sekunda

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("ESP32 je připraven. Pošlete zprávu:");
  
  // Inicializace robotka (pokud potřebuješ LEDky nebo jiné funkce)
  rkConfig cfg;
  rkSetup(cfg);
}

void loop() {
  unsigned long now = millis();

  // 1) Každou sekundu vypíše "..."
  if (now - lastDotTime >= dotInterval) {
    Serial.println("...");
    lastDotTime = now;
  }

  // 2) Čtení dat z terminálu a echo zpět
  while (Serial.available()) {
    char c = Serial.read();
    Serial.println(c);
    Serial.println(c);
    Serial.println(c);
    Serial.println(c);
  }
}
