#include <Arduino.h>
#include <Servo.h> 
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>

#include <Wire.h>


void setup() {
  Serial.begin(9600);
  pinMode(39, INPUT);
}

void loop() {
  Serial.println(digitalRead(39));
}