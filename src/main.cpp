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

// Servo pins
const int servo1Pin  = 1;
const int servo2Pin  = 2;
const int servo3Pin  = 3;
const int servoGrPin = 4;



// Defines motor interface type
#define motorInterfaceType 1

// Creates a stepper instance
AccelStepper turnStepper(motorInterfaceType, stepPin, dirPin);

// Creates servo instances for all joints
Servo svBase;
Servo svElbow;
Servo svWrist;
Servo svGripper;

// Creates vl1 instance for VL6180 distance sensor 
Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

// Custom variable type for diabolo orientation
typedef enum {HORIZONTAL, VERTICAL} ORIENTATION;

// Structure to define all servo angles in a single instance
struct Pos{
  int base;
  int elbow;
  int wrist;
  int gripper;
};

// Positions of all servo's
Pos servoPos;

void updateServoPos(){
  svBase.write(servoPos.base);
  svElbow.write(servoPos.elbow);
  svWrist.write(servoPos.wrist);
  svGripper.write(servoPos.gripper);

  Serial.print(servoPos.base);
  Serial.print(" ");
  Serial.print(servoPos.elbow);
  Serial.print(servoPos.wrist);
  Serial.println(servoPos.gripper);
}

// Custom variable to indicate the side to scan for the diabolo
typedef enum {LEFT, RIGHT} SIDE;

// Function for picking up a diabolo where "dirDiabolo" is the orientation of the diabolo and "side" is the side to scan
void scanAndPickup(ORIENTATION dirDiabolo, SIDE side){

  // Decide side to pick up the diabolo
  int dir;
  if(side == LEFT){
    dir = -1;
  }
  else{
    dir = 1;
  }

  // Rotate arm to avoid support bracket
  turnStepper.moveTo(dir*20);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  // Define arm joint positions for scanning the diabolo
  servoPos.elbow = 60;
  servoPos.wrist = 20;
  servoPos.gripper = 40;
  updateServoPos();

  // Rotate arm to scan for diabolo
  turnStepper.moveTo(dir*140);

  // Keep rotating arm till sesor detects diabolo
  while(vl1.readRange() > 80){
    turnStepper.run();
  }
  turnStepper.stop();
  
  // Set joint positions to scan for diabolo position
  servoPos.base     = 10;
  servoPos.elbow    = 50;
  servoPos.wrist    = 20;
  updateServoPos();
  
  // Read sensor to check orientation and if it matches the orientation of the diabolo that needs to be picked up execude code to do so
  if(vl1.readRange() > 80 && dirDiabolo == HORIZONTAL){

    // Position sequence to pick up the diabolo
    servoPos.base     = 50;
    servoPos.elbow    = 50;
    servoPos.wrist    = 120;
    updateServoPos();

    delay(100);
    servoPos.base     = 20;
    servoPos.elbow    = 50;
    servoPos.wrist    = 120;
    updateServoPos();

    delay(100);
    servoPos.gripper = 15;
    updateServoPos();
  }



}

void setup() {

  // Initial servo positions
  servoPos.base     = 0;
  servoPos.elbow    = 30;
  servoPos.wrist    = 0;
  servoPos.gripper  = 15;

  // Starts serial connection
  Serial.begin(9600);

  // Attaches servo instances and set them to initial positions
  svBase.attach(servo1Pin, 500, 2500);
  svBase.write(servoPos.base);
  delay(100);

  svElbow.attach(servo2Pin, 500, 2500);
  svElbow.write(servoPos.elbow);
  delay(100);

  svWrist.attach(servo3Pin, 500, 2500);
  svWrist.write(servoPos.wrist);
  delay(100);

  svGripper.attach(servoGrPin, 500, 2500);
  svGripper.write(servoPos.gripper);
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

  // Disables ToF sensor vl2 and colour sensor
  digitalWrite(enableVl2, LOW);

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

  vl1.setAddress(0x30);

  // !!!VL2!!!
  // Enables ToF sensor vl2 and wait for it to start
  digitalWrite(enableVl2, HIGH);
  delay(50);

  // Begins communication with ToF sensor vl2
  if (!vl2.begin()) {
    Serial.println("Failed to find vl2");
    while (1);
  }
  Serial.println("Vl2 found!");

  
  

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	turnStepper.setMaxSpeed(1000);
	turnStepper.setAcceleration(1000);
	turnStepper.setSpeed(1000);
}

void loop() {


}