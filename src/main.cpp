#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VL6180X.h>
#include <SPI.h>

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

Servo Sv;

void setup() {
  Sv.attach(9, 500, 2500);

  Sv.write(90);
}

void loop() {

}
