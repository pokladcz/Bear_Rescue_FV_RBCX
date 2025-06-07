#include "Arduino.h"
#include "robotka.h"
#include <iostream>
#include <thread>
#include "RBCX.h"
/////////////////////////////////////
static const uint8_t Bbutton1 = 34;
static const uint8_t Bbutton2 = 35;
const float roztec = 200.0; // roztec kol v mm
float  koeficient_turn = 1.030; // koeficient pro otáčení na místě

float koeficient_r_f = 1.029;
float koeficient_l_f = 1.018;
float koeficient_r_b = 1.03;
float koeficient_l_b = 1.032;

const float ML_25 = 1.00f, MP_25 = 1.00f;
const float ML_50 = 1.00f, MP_50 = 0.9895f;
const float ML_70 = 1.00f, MP_70 = 0.9721f;
const float ML_95 = 1.00f, MP_95 = 0.9675f;
//maximalni rychlost vice nez 60% z 4200 tzs, cirka 2600 je maximalka--65% na 95
// ted je potreba upravovat konstanty, podle noveho maxima 2600
const float ML_25_back = 1.00f, MP_25_back = 1.00f;
const float ML_50_back = 1.00f, MP_50_back = 0.99f;
const float ML_70_back = 1.00f, MP_70_back = 0.98f;
const float ML_95_back = 1.00f, MP_95_back = 0.97f;
////////////////////
// Před funkcí:
const float Kp = 55.0f;
const float Ki = 0.01f;
const float Kd = 0.1f;
int a = 300; // koeficient zrychlovani
//////////
void small_forward(int mm, int speed){
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka = 0;
    int M1_pos_up = 0, M4_pos_up = 0, M1_pos_start = 0, M4_pos_start = 0;
    int target = (1200);
    // std::cout << "target: " << target << std::endl;

    auto& man = rb::Manager::get(); // vytvoří referenci na man clas
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // Startovní pozice
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos_start = -info.position(); 
    });
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        M4_pos_start = info.position();
    });
    delay(50); // pro jistotu, aby se stihly načíst pozice

    // Zrychlování
    for(int i = 0; i < target-1; i += 150) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(-i * 0.98);
        man.motor(rb::MotorId::M4).power(i + odhylaka * Kp + integral * Ki + (odhylaka - last_odchylka) * Kd);
        // Serial.printf("[ACCEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylaka, integral);
        delay(10);
    }
    float MLc, MPc;
    if (mm > 0) {
        if (speed == 25)      { MLc = ML_25; MPc = MP_25; }
        else if (speed == 50) { MLc = ML_50; MPc = MP_50; }
        else if (speed == 70) { MLc = ML_70; MPc = MP_70; }
        else if (speed == 95) { MLc = ML_95; MPc = MP_95; }
        else                  { MLc = ML_50; MPc = MP_50; }
        Serial.printf("[FORWARD] FWD | speed: %.2f | MLc: %.4f | MPc: %.4f | L: %.2f | P: %.2f\n", speed, MLc, MPc, speed * MLc, speed * MPc);
        rkMotorsDrive(mm, mm, speed * MLc, speed * MPc);
    } else {
        if (speed == 25)      { MLc = ML_25_back; MPc = MP_25_back; }
        else if (speed == 50) { MLc = ML_50_back; MPc = MP_50_back; }
        else if (speed == 70) { MLc = ML_70_back; MPc = MP_70_back; }
        else if (speed == 95) { MLc = ML_95_back; MPc = MP_95_back; }
        else                  { MLc = ML_50_back; MPc = MP_50_back; }
        Serial.printf("[FORWARD] BACK | speed: %.2f | MLc: %.4f | MPc: %.4f | L: %.2f | P: %.2f\n", speed, MLc, MPc, speed * MLc, speed * MPc);
        rkMotorsDrive(mm, mm, speed * MLc, speed * MPc);
    }
    for(int i = 1500; i > 150; i -= 150) {

        man.motor(rb::MotorId::M1).power(-i * 0.92);
        man.motor(rb::MotorId::M4).power(i );
        // Serial.printf("[ACCEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylaka, integral);
        delay(10);
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
}
void forward(float mm, float speed) {
    int M1_pos = 0, M4_pos = 0, odhylaka = 0, integral = 0, last_odchylka = 0;
    int M1_pos_up = 0, M4_pos_up = 0, M1_pos_start = 0, M4_pos_start = 0;
    int target = (32000 * speed/100);
    // std::cout << "target: " << target << std::endl;

    auto& man = rb::Manager::get(); // vytvoří referenci na man clas
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // Startovní pozice
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos_start = -info.position(); 
    });
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        M4_pos_start = info.position();
    });
    delay(50); // pro jistotu, aby se stihly načíst pozice

    // Serial.printf("[FORWARD] Start M1_pos_start: %d, M4_pos_start: %d\n", M1_pos_start, M4_pos_start);

    int M1_must_go = ((mm / (127.5 * 3.141592)) * 40.4124852f * 48.f) + M1_pos_start; // přepočet mm na encodery
    int M4_must_go = ((mm / (127.5 * 3.141592)) * 40.4124852f * 48.f) + M4_pos_start;
    // Serial.printf("[FORWARD] M1_must_go: %d, M4_must_go: %d\n", M1_must_go, M4_must_go);

    // Zrychlování
    for(int i = 0; i < target-1; i += a) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(-i * 0.9);
        man.motor(rb::MotorId::M4).power(i + odhylaka * Kp + integral * Ki + (odhylaka - last_odchylka) * Kd);
        // Serial.printf("[ACCEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylaka, integral);
        delay(10);
    }

    // Získání aktuální pozice po zrychlení
    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos_up = -info.position();
    });
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        M4_pos_up = info.position();
    });
    delay(50);

    int M1_pos_diff = M1_pos_up - M1_pos_start;
    int M4_pos_diff = M4_pos_up - M4_pos_start;
    // Serial.printf("[AFTER ACCEL] M1_pos_up: %d, M4_pos_up: %d, M1_pos_diff: %d, M4_pos_diff: %d\n", M1_pos_up, M4_pos_up, M1_pos_diff, M4_pos_diff);

    integral = 0; // reset integrálu

    // Jízda na vzdálenost
    while (M1_pos < (M1_must_go - (2 * M1_pos_diff)) && M4_pos < (M4_must_go - (2 * M4_pos_diff))) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;
        integral += odhylaka;
        // omez ho v rozumných mezích, např. ±max_integral
        const int max_integral = 10000;
        if (integral >  max_integral) integral =  max_integral;
        if (integral < -max_integral) integral = -max_integral;
        man.motor(rb::MotorId::M1).power(-target*0.95);
        int rawPower = target + odhylaka * Kp + integral * Ki + (odhylaka - last_odchylka) * Kd;
        // saturace
        const int maxPower = 32000;
        if      (rawPower >  maxPower) rawPower =  maxPower;
        else if (rawPower < -maxPower) rawPower = -maxPower;
        man.motor(rb::MotorId::M4).power(rawPower);


        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });

        // Serial.printf("[CONST] M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d | powerM4: %d\n", M1_pos, M4_pos, odhylaka, integral, rawPower);

        delay(50);
        last_odchylka = odhylaka;
    }
        integral = 0; // reset integrálu

    // Zpomalení
    for(int i = target ; i > 0 ; i -= a) {
        odhylaka = M1_pos - M4_pos;
        integral += odhylaka;

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position(); 
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        // if(i < 5000){
        //   while(M1_pos < ( M1_must_go - (speed/50.0 * 15))){
        //     delay(10);
        //     man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        //       M1_pos = -info.position();
        //     });
        //     man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        //       M4_pos = info.position();
        //     });
        //     // Serial.printf("[finishing] M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", M1_pos, M4_pos, odhylaka, integral);
        //   }
        // }
        man.motor(rb::MotorId::M1).power(-i);
        man.motor(rb::MotorId::M4).power(i );

        // Serial.printf("[DECEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylaka, integral);

        delay(10);
    }
    // Zastavit oba motory na konci
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
    // Serial.println("[FORWARD] Dorovnání hotovo!");

    // Serial.println("[FORWARD] Hotovo!");
}

void fast_forward(float mm, float speed) {
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0;
    int M1_pos_start = 0, M4_pos_start = 0;
    int target = (32000 * speed / 100);

    auto& man = rb::Manager::get();
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);

    man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
        M1_pos_start = -info.position();
    });
    man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
        M4_pos_start = info.position();
    });
    delay(20);

    int M1_must_go = ((mm / (127.5 * 3.141592)) * 40.4124852f * 48.f) + M1_pos_start;
    int M4_must_go = ((mm / (127.5 * 3.141592)) * 40.4124852f * 48.f) + M4_pos_start;

    // Zrychlování + jízda bez zpomalování
    bool done = false;
    int i = 0;
    while (!done) {
        odchylka = M1_pos - M4_pos;
        integral += odchylka;
        if (i < target) i += a;
        if (i > target) i = target;

        int powerM1 = -i * 0.95;
        int powerM4 = i + odchylka * Kp + integral * Ki + (odchylka - last_odchylka) * Kd;

        // saturace
        const int maxPower = 32000;
        if (powerM4 > maxPower) powerM4 = maxPower;
        if (powerM4 < -maxPower) powerM4 = -maxPower;

        man.motor(rb::MotorId::M1).power(powerM1);
        man.motor(rb::MotorId::M4).power(powerM4);

        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });

        last_odchylka = odchylka;

        // Konec když oba motory dojedou
        if (M1_pos >= M1_must_go && M4_pos >= M4_must_go) done = true;

        delay(2);
    }
    man.motor(rb::MotorId::M1).power(0);
    man.motor(rb::MotorId::M4).power(0);
}

void encodery() {
  // Serial.printf("L: %f, R: %f\n", rkMotorsGetPositionLeft(), rkMotorsGetPositionRight());
}

void radius_r(int degrees, int polomer, int speed)
{
  float sR = speed * ((polomer) / (polomer + roztec));
  float sL = speed * 0.987;
  bool left_done =false;
  bool right_done = false;
  if(degrees > 0){
    float stupne = degrees * koeficient_r_f;
    rkMotorsDriveLeftAsync((((polomer + roztec) * PI * stupne) / 180) , sL, [&left_done]() {
        left_done = true;
    });
    rkMotorsDriveRightAsync((( polomer * PI * stupne) / 180) , sR, [&right_done]() {
        right_done = true;
    });
  }
  else{
  float stupne= degrees * koeficient_r_b;
    rkMotorsDriveLeftAsync((((polomer + roztec) * PI * stupne) / 180) , sL, [&left_done]() {
        left_done = true;
    });
    rkMotorsDriveRightAsync((( polomer * PI * stupne) / 180) , sR, [&right_done]() {
        right_done = true;
    });
  }
    while(!left_done || !right_done) {
        delay(10); // čekáme na dokončení obou motorů
    }
}
void radius_l(int degrees, int polomer, int speed){
  float sR = speed ;
  float sL = speed * ((polomer) / (polomer + roztec));
  bool left_done = false;
  bool right_done = false;
    if(degrees > 0){
        float stupne = degrees * koeficient_l_f;
        rkMotorsDriveLeftAsync((( polomer * PI * stupne) / 180) , sL, [&left_done]() {
            left_done = true;
        });
        rkMotorsDriveRightAsync((((polomer + roztec) * PI * stupne) / 180) , sR, [&right_done]() {
            right_done = true;
        });
    }
    else{
        float stupne = degrees * koeficient_l_b;
        rkMotorsDriveLeftAsync((( polomer * PI * stupne) / 180), sL, [&left_done]() {
            left_done = true;
        });
        rkMotorsDriveRightAsync((((polomer + roztec) * PI * stupne) / 180) , sR, [&right_done]() {
            right_done = true;
        });
    }
    while(!left_done || !right_done) {
        delay(10); // čekáme na dokončení obou motorů
    }
}
void fast_radius_r(int degrees, int polomer, int speed) {
    auto& man = rb::Manager::get();
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    float sR = speed * ((polomer) / (polomer + roztec));
    float sL = speed;
    int M1_pos = 0, M4_pos = 0;
    float distance_right = 0, distance_left = 0;
    if(degrees > 0){
        float stupne = degrees * koeficient_l_f;
        man.motor(rb::MotorId::M1).power(-sR * 320);
        man.motor(rb::MotorId::M4).power(sL * 320);
        distance_right = ((polomer * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        distance_left = (((polomer + roztec) * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = info.position();
            });
        while(M1_pos < distance_right || M4_pos < distance_left) {
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = -info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = info.position();
            });
            delay(2); // čekáme na dokončení obou motorů
        }
    }
    else{
        float stupne = degrees * koeficient_l_b;
        man.motor(rb::MotorId::M1).power(sR * 320);
        man.motor(rb::MotorId::M4).power(-sL * 320);
        distance_right = ((polomer * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        distance_left = (((polomer + roztec) * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        while(M1_pos < distance_right || M4_pos < distance_left) { 
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = -info.position();
            });
            delay(2); // čekáme na dokončení obou motorů
        }
    }
    

    // NEVYPÍNÁME! motory dál jedou v poměru -sL : +sR
}
void fast_radius_l(int degrees, int polomer, int speed) {
    auto& man = rb::Manager::get();
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    float sR = speed ;
    float sL = speed * ((polomer) / (polomer + roztec));
    int M1_pos = 0, M4_pos = 0;
    float distance_right = 0, distance_left = 0;
    if(degrees > 0){
        float stupne = degrees * koeficient_l_f;
        man.motor(rb::MotorId::M1).power(-sR * 320);
        man.motor(rb::MotorId::M4).power(sL * 320);
        distance_left = ((polomer * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        distance_right = (((polomer + roztec) * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = info.position();
            });
        while(M1_pos < distance_right || M4_pos < distance_left) {
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = -info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = info.position();
            });
            delay(2); // čekáme na dokončení obou motorů
        }
    }
    else{
        float stupne = degrees * koeficient_l_b;
        man.motor(rb::MotorId::M1).power(sR * 320);
        man.motor(rb::MotorId::M4).power(-sL * 320);
        distance_left = ((polomer * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        distance_right = (((polomer + roztec) * PI * stupne) / 180.0f) * 40.4124852f * 48.f / (127.5f * PI);
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = -info.position();
        });
        while(M1_pos < distance_right || M4_pos < distance_left) { 
            man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
                M1_pos = info.position();
            });
            man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
                M4_pos = -info.position();
            });
            delay(2); // čekáme na dokončení obou motorů
        }
    }
    

    // NEVYPÍNÁME! motory dál jedou v poměru -sL : +sR
}
void turn_on_spot(int degrees){
  rkMotorsDrive(((degrees/360.0) * PI * roztec) * koeficient_turn , ((degrees/360.0) * -PI * roztec)* koeficient_turn, 30 , 30 );
    // Serial.printf("Otáčení na místě o %d stupňů\n", degrees);
    delay(500);
}
void back_buttons(int speed)
{
    int M1_pos = 0, M4_pos = 0, odchylka = 0, integral = 0, last_odchylka = 0;
    const float Kp = 55.0f;
    const float Ki = 0.01f;
    const float Kd = 0.1f;
    int target = (32000 * speed/100);
    auto& man = rb::Manager::get();
    man.motor(rb::MotorId::M1).setCurrentPosition(0);
    man.motor(rb::MotorId::M4).setCurrentPosition(0);
    // Zrychlování
    for(int i = 0; i < target-1; i += a) {
        odchylka = M1_pos - M4_pos;
        integral += odchylka;
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });
        man.motor(rb::MotorId::M1).power(i * 0.94);
        man.motor(rb::MotorId::M4).power(-i + odchylka * Kp + integral * Ki + (odchylka - last_odchylka) * Kd);
        // Serial.printf("[ACCEL] i: %d | M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", i, M1_pos, M4_pos, odhylka, integral);
        delay(8);
    }
    int time = 0;
    int stop_time = 5000; // časový limit pro dorovnání
    while (time < stop_time)
    {
        // Získání aktuálních pozic
        man.motor(rb::MotorId::M1).requestInfo([&](rb::Motor& info) {
            M1_pos = -info.position();
        });
        man.motor(rb::MotorId::M4).requestInfo([&](rb::Motor& info) {
            M4_pos = info.position();
        });

        odchylka = M1_pos - M4_pos;
        integral += odchylka;

        man.motor(rb::MotorId::M1).power(speed * 320);
        man.motor(rb::MotorId::M4).power(-speed * 320 + odchylka * Kp + integral * Ki + (odchylka - last_odchylka) * Kd);

        // Serial.printf("[BACK] M1_pos: %d | M4_pos: %d | odchylka: %d | integral: %d\n", M1_pos, M4_pos, odchylka, integral);

        last_odchylka = odchylka;

        if ((digitalRead(Bbutton1) == LOW) && (digitalRead(Bbutton2) == LOW)) {
            break;
        }
        delay(50);
        time+=50;
    }
    delay(50);
    // Serial.println("Obě Tlačítka STISKNUTY!");
    rkMotorsSetPower(0, 0);
}
//timeout