#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>

// Define pin connections
const int dirPin = 14;
const int stepPin = 15;

// MicroStep pins
const int uStep0 = 20;
const int uStep1 = 19;
const int uStep2 = 18;


// Define motor interface type
#define motorInterfaceType 1

// Creates a stepper instance
AccelStepper turnStepper(motorInterfaceType, stepPin, dirPin);

// Creates two vl6180 instances
Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

// Creates a servo instance
Servo Sv;


void setup() {

  // Attach servo instance
  Sv.attach(2, 500, 2500);

  // // Set Microsteppins as output
  // pinMode(uStep0, OUTPUT);
  // pinMode(uStep1, OUTPUT);
  // pinMode(uStep2, OUTPUT);

  // // Set 1/4 microstepping
  // digitalWrite(uStep0, LOW);
  // digitalWrite(uStep1, HIGH);
  // digitalWrite(uStep2, LOW);

  // // set the maximum speed, acceleration factor,
	// // initial speed and the target position
	// turnStepper.setMaxSpeed(1000);
	// turnStepper.setAcceleration(500);
	// turnStepper.setSpeed(1000);
  Sv.write(0);
	

  

  

}

void loop() {
  // Change direction once the motor reaches target position


  // while(turnStepper.distanceToGo() != 0){turnStepper.run();}
  // delay(500);
  // turnStepper.moveTo(0);
  // while(turnStepper.distanceToGo() != 0){turnStepper.run();}
  // delay(500);
  // turnStepper.moveTo(200);

 
}
