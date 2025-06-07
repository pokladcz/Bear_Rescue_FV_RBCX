#include <Arduino.h>
#include "robotka.h"
#include "motor_commands.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "smart_servo_command.h"

#define RX1_PIN   16
#define TX1_PIN   17

#define ESP32P4_SYNC0 0xAA
#define ESP32P4_SYNC1 0x55

typedef struct __attribute__((packed)) {
    uint16_t x;
    uint16_t y;
    bool camera;
    bool on;
    int16_t angle;         // Úhel ve stupních
    uint16_t distance;     // Délka v mm (nebo cm dle odesílatele)
    uint16_t max_distance; // Maximální možná délka v mm
} Esp32p4Message;

enum RxState { WAIT_SYNC0, WAIT_SYNC1, READ_PAYLOAD, READ_CHECKSUM };
RxState state = WAIT_SYNC0;

static const size_t PAYLOAD_SIZE = sizeof(Esp32p4Message);
uint8_t buf[PAYLOAD_SIZE];
size_t idx = 0;
uint8_t checksum = 0;

Esp32p4Message msg = {
    .x = 0,
    .y = 0,
    .camera = false,
    .on = false,
    .angle = 0,
    .distance = 1000,
    .max_distance = 1300
};
bool mam_ho(){
    int distance = rkUltraMeasure(4);
    if (distance < 1) {
        Serial.println("Chyba při měření vzdálenosti!");
        return false; // Vrátí false, pokud došlo k chybě
    }
    Serial.printf("ultrazvuk: %d\n", distance); // Vypíše hodnotu distance
    if(distance > 70 && distance < 800){
        return false; // pokud je medvěd blízko, vrátí true
    }
    else{
        return true;
    }
}
void klepeta_open(auto & g_bus){
    s_s_move(g_bus, 0, 80, 120); // Otevře klepeta na servo s ID 0
    s_s_move(g_bus, 1, 200,120); // Otevře klepeta na servo s ID 1
}
void klepeta_open_max(auto & g_bus){
    s_s_move(g_bus, 0, 40, 120); // Otevře klepeta na servo s ID 0
    s_s_move(g_bus, 1, 240,120); // Otevře klepeta na servo s ID 1
}
void klepeta_close(auto & g_bus){
    s_s_soft_move(g_bus, 0, 160, 90); // Zavře klepeta na servo s ID 0
    s_s_soft_move(g_bus, 1, 120, 90); // Zavře klepeta na servo s ID 1
}
// Funkce pro aktualizaci struktury podle příchozích dat
void updateEsp32p4Message(Esp32p4Message* msg) {
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        switch (state) {
            case WAIT_SYNC0:
                if (c == ESP32P4_SYNC0) state = WAIT_SYNC1;
                break;
            case WAIT_SYNC1:
                if (c == ESP32P4_SYNC1) {
                    state = READ_PAYLOAD;
                    idx = 0;
                    checksum = 0;
                } else {
                    state = (c == ESP32P4_SYNC0) ? WAIT_SYNC1 : WAIT_SYNC0;
                }
                break;
            case READ_PAYLOAD:
                buf[idx++] = c;
                checksum += c;
                if (idx >= PAYLOAD_SIZE) {
                    state = READ_CHECKSUM;
                }
                break;
            case READ_CHECKSUM:
                if (c == checksum) {
                    memcpy(msg, buf, PAYLOAD_SIZE); // Přepisuje předanou strukturu
                    printf("Received: x=%u, y=%u, camera=%u, on=%u, angle=%d, distance=%u, max_distance=%u\n",
                        msg->x, msg->y, msg->camera, msg->on, msg->angle, msg->distance, msg->max_distance);
                    if (msg->on) {
                        rkLedGreen(true);
                    } else {
                        rkLedGreen(false);
                    }
                } else {
                    printf("Checksum error! (got 0x%02X, expected 0x%02X)\n", c, checksum);
                }
                state = WAIT_SYNC0;
                break;
        }
    }
}

void projeti_bludiste_tam(){
    Serial.println("projeti bludiště tam");
    // Příkaz pro projetí bludiště
    auto & g_bus = rkSmartServoBus(2);
    //small_forward(595, 95); // Předpokládáme, že tato funkce je definována v motor_commands.h
    fast_forward(600, 95); // Předpokládáme, že tato funkce je definována v motor_commands.h
    fast_radius_r(174, 130, 95);
    //small_forward(140, 95);
    fast_radius_l(170, 210, 95);
    klepeta_open(g_bus);
    fast_forward(810, 95);
    Serial.println("projeti bludiště hotova");
}
void jed_pro_medu(){
    ///////////////// hledani podle uhlu
    Serial.println("Hledání medvěda podle úhlu a vzdálenosti");
    fast_radius_l((int)msg.angle, 0, 20); // otočíme se na správný směr
    rkMotorsSetPower(0, 0); // Nastavíme motory na plný výkon
    Serial.printf("Otočení na úhel: %d\n", msg.angle);
    auto & g_bus = rkSmartServoBus(2);
    Serial.printf("Hledám medvěda na úhlu: %d\n", msg.angle);
    if((msg.angle > 80)){
        Serial.println("Medvěd je nalevo, otáčíme se doleva");
        small_forward(200, 70); // Předpokládáme, že tato funkce je definována v motor_commands.h
        Serial.println("Medvěd je .......");
    }
    Serial.printf("Hledám medvěda na vzdálenosti: %d\n", msg.distance);
    if((msg.angle > 10)){
        klepeta_open_max(g_bus);
    }
    if(msg.distance >= (msg.max_distance - 430)){
        forward(msg.distance - 400 , 80);
        klepeta_open(g_bus);
        delay(300);
        small_forward(200, 80); // Předpokládáme, že tato funkce je definována v motor_commands.h
    }
    else{
        
        small_forward(msg.distance - 100 , 80);
    }
    if(mam_ho()){
        rkBuzzerSet(true);
        delay(1);
        rkBuzzerSet(false);
        Serial.println("Medvěd byl nalezen!");
    }
    else{
        Serial.println("Medvěd nebyl nalezen!");
        //sem dat dojeti spet na hledaci npozici a zapnout kameru...
    }
    delay(100);
    klepeta_close(g_bus);
    delay(800);
    printf("ANGLE: %d\n", msg.angle);
    ///////////////// aby se dokazal dotocit!!!
    if(msg.angle < 10){
        fast_radius_l(-30, 180, 80); // otočíme se na správný směr
        delay(100);
        fast_radius_r(-30, 180, 80); // otočíme se na správný směr
    }
    else if(msg.angle > 80){
        fast_radius_r(-30, 180, 80); // otočíme se na správný směr
        delay(100);
        fast_radius_l(-30, 180, 80); // otočíme se na správný směr
    }
    turn_on_spot(-(95-msg.angle)); // otočíme se zpět 95, protoze se otoci malo
    delay(100);
    back_buttons(100); // vrátíme se zpět
    fast_forward(80, 80); // a jedeme zpět
    turn_on_spot(90);
    back_buttons(100); // vrátíme se zpět
    /////////////
}
void navrat_domu(){
    auto & g_bus = rkSmartServoBus(2);
    fast_radius_l(90, 0, 90); // otočíme se na správný směr
    back_buttons(90); // vrátíme se zpět
    fast_forward(160, 90); // a jedeme zpět
    fast_radius_r(90, 350, 90); // otočíme se zpět
    fast_forward(200, 90); // a jedeme zpět
    fast_radius_l(180, 110, 90); // otočíme se zpět
    fast_forward(550, 90); // a jedeme zpět
    klepeta_open(g_bus);
    rkMotorsSetPower(30000, 30000);
}
void send_esp_ready(bool ready) {
    uint8_t tx_buf[4];
    tx_buf[0] = ESP32P4_SYNC0;
    tx_buf[1] = ESP32P4_SYNC1;
    tx_buf[2] = ready ? 1 : 0; // 1 = ready, 0 = not ready
    tx_buf[3] = tx_buf[2];     // jednoduchý checksum = hodnota bool
    Serial1.write(tx_buf, 4);
    // Pro ladění:
    Serial.printf("Odesílám ESP_READY: %d (0x%02X)\n", tx_buf[2], tx_buf[2]);
}

void setup() {
  Serial.begin(115200);
  rkConfig cfg;
  rkSetup(cfg);
  rkLedRed(true); // Turn on red LED
  rkLedBlue(true); // Turn on blue LED
  printf("Robotka started!\n");
    // Nastavení tlačítek
  pinMode(Bbutton1, INPUT_PULLUP);
  pinMode(Bbutton2, INPUT_PULLUP);
  auto &bus = rkSmartServoBus(2);
  s_s_init(bus, 1, 120, 240);
  s_s_init(bus, 0, 40, 200);
  printf("RBCX UART RX started!\n");
  Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  Serial.println("Motor example started!");
  while(true){
    if(msg.x > 0 && msg.y > 0){
        projeti_bludiste_tam();
        jed_pro_medu();
        navrat_domu();
        msg.x = 0; // Reset x and y after processing
        msg.y = 0; // Reset x and y after processing
    }
    if(msg.camera){
        Serial.println("Camera mode activated");
        msg.camera = false; // Reset camera flag
        projeti_bludiste_tam();
        send_esp_ready(true);
        // Zde můžete přidat kód pro aktivaci kamery
        // např. spustit detekci objektů
    }
    else if(msg.on){
        Serial.println("Robot is ON");
        // Zde můžete přidat kód pro zapnutí robota
        // např. zapnout motory nebo senzory
    }
    else{
        updateEsp32p4Message(&msg);
    }
    if (digitalRead(Bbutton1) == LOW) {
        Serial.println("Button 1 pressed");
        // delay(3000);
        // projeti_bludiste_tam();
        // jed_pro_medu();
        // navrat_domu();
        // Zde můžete přidat kód pro akci při stisku tlačítka 1
    }
    if (digitalRead(Bbutton2) == LOW) {
        Serial.println("Button 2 pressed");
        klepeta_close(bus);
        // Zde můžete přidat kód pro akci při stisku tlačítka 2
    }
    if(rkButtonIsPressed(BTN_UP)){
        klepeta_open_max(bus);
    }
    if(rkButtonIsPressed(BTN_DOWN)){
        klepeta_open(bus);
    }
    delay(100);
  }
}

void loop() {

}