#include <avr/wdt.h>

#include "devices.h"

Ultrasonic u;
Motor motor;

long startTime;

void setup() {
    // Init stuff
    Serial.begin(9600);
    u.init();
    motor.init();
    delay(2000);

    motor.move(FORWARDS, 50);
    startTime = millis();
}

void loop() {
    // Get the distance, stop if it is <= 20cm
    uint16_t dist = u.get();
    Serial.println(dist);
    if (dist <= 20) {
        motor.stop();
    }

    // stop the motor once 5 seconds have passed
    Serial.println(millis() - startTime);
    if (millis() - startTime >= 5000) motor.stop();
    exit(0);
}