#include <Arduino.h>
#include <Servo.h> 
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>

#include <Wire.h>

Adafruit_VL6180X vl = Adafruit_VL6180X();

void setup() {
  Serial.begin(9600);

  // wait for serial port to open on native usb devices
  while (!Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL6180x test!");
  if (! vl.begin()) {
    Serial.println("Failed to find sensor");
    while (1);
  }
  Serial.println("Sensor found!");
}

void loop() {
  
  uint8_t range = vl.readRange();
  

  
  Serial.print("Range: "); Serial.println(range*range);
  

  // Some error occurred, print it out!
  
  
  delay(50);
}