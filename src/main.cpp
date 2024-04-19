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
const int servo1Pin  = 4;
const int servo2Pin  = 3;
const int servo3Pin  = 2;
const int servoGrPin = 1;


// Defines motor interface type
#define motorInterfaceType 1

// Creates a stepper instance
AccelStepper turnStepper(motorInterfaceType, stepPin, dirPin);

Servo sv1;
Servo sv2;
Servo sv3;
Servo svGr;

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
// Adafruit_VL6180X vl2 = Adafruit_VL6180X();
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


void setup() {
  // Starts serial connection
  Serial.begin(9600);

  // Attaches servo instances
  sv1.attach(servo1Pin, 500, 2500);
  sv2.attach(servo2Pin, 500, 2500);
  sv3.attach(servo3Pin, 500, 2500);
  svGr.attach(servoGrPin, 500, 2500);

  // Sets Microsteppins as output
  pinMode(uStep0, OUTPUT);
  pinMode(uStep1, OUTPUT);
  pinMode(uStep2, OUTPUT);

  // Sets 1/4 microstepping
  digitalWrite(uStep0, LOW);
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
  if (!vl1.begin()) {
    Serial.println("Failed to find vl1");
    while (1);
  }
  Serial.println("Vl1 found!");

  // Changes adress of ToF sensor vl1 from 0x29 to 0x30
  vl1.setAddress(0x30);


  // // !!!VL2!!!
  // // Enables ToF sensor vl2 and wait for it to start
  // digitalWrite(2, HIGH);
  // delay(50);

  // // Begins communication with ToF sensor vl2
  // if (!vl2.begin()) {
  //   Serial.println("Failed to find vl2");
  //   while (1);
  // }
  // Serial.println("Vl2 found!");

  // // Changes adress of ToF sensor vl2 from 0x29 to 0x31
  // vl2.setAddress(0x31);


  // // !!!COLOUR SENSOR!!!
  // // Enable colour sensor
  // digitalWrite(enableTcs, HIGH);
  // delay(50);

  // // Begins communication with colour sensor tcs
  // if (!tcs.begin()) {
  //   Serial.println("Failed to find tcs");
  //   while (1);
  // }
  // Serial.println("Tcs found!");

  Serial.println("adresses:");
  Serial.print("vl1: ");
  Serial.println(vl1.getAddress());
  Serial.print("vl2: ");
  // Serial.println(vl2.getAddress());
  Serial.print("tcs: ");
  Serial.println(0x29);
  Serial.println("Init done!");

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	turnStepper.setMaxSpeed(1000);
	turnStepper.setAcceleration(500);
	turnStepper.setSpeed(1000);

  
  
}

void loop() {

  
}
