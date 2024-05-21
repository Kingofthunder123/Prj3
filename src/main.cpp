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
const int servo1Pin  = 12;
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

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
// Adafruit_VL6180X vl2 = Adafruit_VL6180X();
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);


//typedef enum {GRIPP_HOR, GRIPP_VER, RELEASE} ACTION;



void setup() {
  // Starts serial connection
  Serial.begin(9600);

  // Attaches servo instances
  // svBase.attach(servo1Pin, 500, 2500);
  // svBase.write(20);
  // delay(100);
  // svElbow.attach(servo2Pin, 500, 2500);
  // svElbow.write(180);
  // delay(100);
  // svWrist.attach(servo3Pin, 500, 2500);
  // svWrist.write(20);
  // delay(100);
  // svGripper.attach(servoGrPin, 500, 2500);
  // svGripper.write(10);
  // delay(100);

  


  // Sets Microsteppins as output
  // pinMode(uStep0, OUTPUT);
  // pinMode(uStep1, OUTPUT);
  // pinMode(uStep2, OUTPUT);

  // // Sets 1/4 microstepping
  // digitalWrite(uStep0, HIGH);
  // digitalWrite(uStep1, HIGH);
  // digitalWrite(uStep2, LOW);
  
  // Sets pinmodes
  // pinMode(enableVl2, OUTPUT);
  // pinMode(enableTcs, OUTPUT);

  // // Disables ToF sensor vl2 and colour sensor
  // digitalWrite(enableVl2, LOW);
  // digitalWrite(enableTcs, LOW);

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
  //vl1.setAddress(0x30);


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

  // Serial.println("adresses:");
  // Serial.print("vl1: ");
  // Serial.println(vl1.getAddress(), HEX);
  // Serial.print("vl2: ");
  // // Serial.println(vl2.getAddress(), HEX);
  // Serial.print("tcs: ");
  // Serial.println(0x29, HEX);
  // Serial.println("Init done!");

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	// turnStepper.setMaxSpeed(1000);
	// turnStepper.setAcceleration(1000);
	// turnStepper.setSpeed(1000);
  
  
  
}

void loop() {
  


  uint8_t range = vl1.readRange();
  uint8_t status = vl1.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    Serial.print("Range: "); Serial.println(range);
  }

  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    Serial.println("Range reading overflow");
  }
  delay(100);

}
