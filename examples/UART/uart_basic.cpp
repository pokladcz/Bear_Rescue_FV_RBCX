#include <Arduino.h>
#include <stdint.h>

#define RX1_PIN   16
#define TX1_PIN   17  // mimochodem nepotřebujeme, ale Arduino vyžaduje parametry

// Stejná struktura (packed implicitně)
typedef struct __attribute__((packed)) {
    uint8_t  id;
    int16_t  value;
} ServoMsg;


const uint8_t SYNC0 = 0xAA;
const uint8_t SYNC1 = 0x55;

enum RxState { WAIT_SYNC0, WAIT_SYNC1, READ_PAYLOAD };
RxState state = WAIT_SYNC0;

static const size_t PAYLOAD_SIZE = sizeof(ServoMsg);
uint8_t buf[PAYLOAD_SIZE];
size_t idx = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("UART1 Receiver with framing");

  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
}

void loop() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    switch (state) {
      case WAIT_SYNC0:
        if (c == SYNC0) state = WAIT_SYNC1;
        break;
      case WAIT_SYNC1:
        if (c == SYNC1) {
          state = READ_PAYLOAD;
          idx = 0;
        } else {
          // možná c je už SYNC0, zkusit znovu
          state = (c == SYNC0) ? WAIT_SYNC1 : WAIT_SYNC0;
        }
        break;
      case READ_PAYLOAD:
        buf[idx++] = c;
        if (idx >= PAYLOAD_SIZE) {
          // máme celý payload
          ServoMsg m;
          memcpy(&m, buf, PAYLOAD_SIZE);
          Serial.print("Received: ");
          Serial.print(m.id);
          Serial.print(", ");
          Serial.println(m.value);
          // zpracování zprávy
          state = WAIT_SYNC0;
        }
        break;
    }
  }
}