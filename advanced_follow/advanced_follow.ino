#include "devices.h"

// Initialize objects
Motor motor;

void setup()
{
  Serial.begin(9600);
  motor.init();
  delay(2000);
}

void loop()
{
  motor.startTracking();
}