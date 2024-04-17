#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VL6180X.h>
#include <SPI.h>

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

void setup() {
  Serial.begin(9600);
  
  pinMode(2, OUTPUT);

  digitalWrite(2, LOW);

  while (!Serial) {
    delay(1);
  }

  Serial.println("Sensor 1 test!");
  if (! vl1.begin()) {
    Serial.println("Failed to find sensor 1");
    while (1);
  }
  Serial.println("Sensor 1 found!");

  vl1.setAddress(0x30);

  digitalWrite(2, HIGH);
  delay(22);

  Serial.println("Sensor 2 test!");
  if (! vl2.begin()) {
    Serial.println("Failed to find sensor2");
    while (1);
  }
  Serial.println("Sensor 2 found!");

  Serial.println("done");
}

void loop() {

}
