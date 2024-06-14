#include <Arduino.h>
#include <Servo.h> 
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>

// Pins for stepper motor
const int dirPin  = 14;
const int stepPin = 15;

// MicroStep pins for stepper motor
const int uStep0 = 18;
const int uStep1 = 19;
const int uStep2 = 20;

// Enable pins for vl2 sensor and tcs sensor

// Defines motor interface type
#define motorInterfaceType 1

// Creates a stepper instance
AccelStepper turnStepper(motorInterfaceType, stepPin, dirPin);

void setup() {


  // Sets Microsteppins as output
  pinMode(uStep0, OUTPUT);
  pinMode(uStep1, OUTPUT);
  pinMode(uStep2, OUTPUT);

  // Sets 1/4 microstepping
  digitalWrite(uStep0, HIGH);
  digitalWrite(uStep1, HIGH);
  digitalWrite(uStep2, LOW);
  
  // Sets pinmodes
  
  // Begins communication with ToF sensor vl2
  
  // // !!!COLOUR SENSOR!!!
  // // Enable colour sensor
  // digitalWrite(enableTcs, HIGH);
  // delay(50);

  // // Begins communication with colour sensor tcs
  // if (!tcs.begin()) {
  //   Serial.println("Failed to find tcs");
  //   while (1););
  // }
  // Serial.println("Tcs found!");

  

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	turnStepper.setMaxSpeed(1000);
	turnStepper.setAcceleration(1000);
	turnStepper.setSpeed(1000);

  
  
  
  
}

void loop() {
  turnStepper.move(-800);
  while(turnStepper.distanceToGo() > 0){
    turnStepper.run();
  }

  
  turnStepper.move(800);
  while(turnStepper.distanceToGo() > 0){
    turnStepper.run();
  }

  Serial.println("goed");

}
