#include <avr/wdt.h>

#include "devices.h"

Ultrasonic u;
Motor motor;

void setup() {
    // Init stuff
    Serial.begin(9600);
    u.init();
    motor.init();
    delay(2000);
}

void loop() {
    if (u.get() <= 20) {
        motor.stop();
        while (u.get() <= 20) {
            motor.move(LEFT, 100);
            delay(200);
        }
        motor.stop();
    } else {
        motor.move(FORWARDS, 100);
    }
}