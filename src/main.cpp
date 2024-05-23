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
const int uStep0 = 20;
const int uStep1 = 19;
const int uStep2 = 18;

// Enable pins for vl2 sensor and tcs sensor
const int enableVl2 = 2;
const int enableTcs = 3;

// Servo pins
const int servo1Pin  = 1;
const int servo2Pin  = 2;
const int servo3Pin  = 3;
const int servoGrPin = 4;



// Defines motor interface type
#define motorInterfaceType 1

// Creates a stepper instance
AccelStepper turnStepper(motorInterfaceType, stepPin, dirPin);

Servo svBase;
Servo svElbow;
Servo svWrist;
Servo svGripper;

// Adafruit_VL6180X vl1 = Adafruit_VL6180X();
// Adafruit_VL6180X vl2 = Adafruit_VL6180X();
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

struct Pos{
  int base;
  int elbow;
  int wrist;
  int gripper;
};



Pos startPos;

typedef enum {HORIZONTAL, VERTICAL} ORIENTATION;

void scanAndPickup(ORIENTATION dirDiabolo){};

void setup() {

  startPos.base     = 0;
  startPos.elbow    = 30;
  startPos.wrist    = 0;
  startPos.gripper  = 15;
  // Starts serial connection
  Serial.begin(9600);

  // Attaches servo instances
  svBase.attach(servo1Pin, 500, 2500);
  svBase.write(startPos.base);
  delay(100);
  svElbow.attach(servo2Pin, 500, 2500);
  svElbow.write(startPos.elbow);
  delay(100);
  svWrist.attach(servo3Pin, 500, 2500);
  svWrist.write(startPos.wrist);
  delay(100);
  svGripper.attach(servoGrPin, 500, 2500);
  svGripper.write(startPos.gripper);
  delay(100);


  // Sets Microsteppins as output
  pinMode(uStep0, OUTPUT);
  pinMode(uStep1, OUTPUT);
  pinMode(uStep2, OUTPUT);

  // Sets 1/4 microstepping
  digitalWrite(uStep0, HIGH);
  digitalWrite(uStep1, HIGH);
  digitalWrite(uStep2, LOW);
  
  // Sets pinmodes
  pinMode(enableVl2, OUTPUT);
  pinMode(enableTcs, OUTPUT);

  // Disables ToF sensor vl2 and colour sensor
  digitalWrite(enableVl2, LOW);
  digitalWrite(enableTcs, LOW);

  // Waits for serial before continuing the program
  while (!Serial) {
    delay(1);
  }


  // !!!VL1!!!
  // Begins communication with ToF sensor vl1
  // if (!vl1.begin()) {
  //   Serial.println("Failed to find vl1");
  //   while (1);
  // }
  // Serial.println("Vl1 found!");

  

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	turnStepper.setMaxSpeed(1000);
	turnStepper.setAcceleration(1000);
	turnStepper.setSpeed(1000);
}

void loop() {


}
