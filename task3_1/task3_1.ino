#include <avr/wdt.h>

#include "devices.h"

IR ir;
Motor motor;

void setup() {
    Serial.begin(9600);
    ir.init();
    ir.setThreshold(450);
    motor.init();
    delay(2000);
}

void basicFollow() {
    if (ir.get(IR_MIDDLE)) {
        motor.move(FORWARDS, 100);
    } else if (ir.get(IR_LEFT)) {
        motor.move(LEFT, 50);
    } else if (ir.get(IR_RIGHT)) {
        motor.move(RIGHT, 50);
    } else {
        motor.stop();
    }
}

void loop() {
    basicFollow();
}