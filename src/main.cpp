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
const int servo1Pin  = 2;
const int servo2Pin  = 3;
const int servo3Pin  = 4;
const int servoGrPin = 5;



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


//typedef enum {GRIPP_HOR, GRIPP_VER, RELEASE} ACTION;

struct Pose {
  int posSv1;
  int posSv2;
  int posSv3;
};

struct Action {
  Pose startPose;
  Pose targetPose;
  bool openGripper;
};

Pose STANDBY_HOR  = {180, 170, 30};
Pose STANDBY_VER  = {10, 180, 0};
Pose NEUTRAL      = {0, 0, 45};

Pose END_HOR = {180, 170, 15};
Pose END_VER = {35, 140, 0};

Pose CURRENT = {sv1.read(), sv2.read(), sv3.read()};

Action GRIPP_HOR = {STANDBY_HOR, END_HOR, false};
Action GRIPP_VER = {STANDBY_VER, END_VER, false};
Action RELEASE_HOR = {STANDBY_HOR, END_HOR, true};
Action RELEASE_VER = {STANDBY_VER, END_VER, true};

void setSv(Pose setPose){

  
  sv1.write(setPose.posSv1);
  delay(1);
  sv2.write(setPose.posSv2);
  delay(1);
  sv3.write(setPose.posSv3);
  delay(1);
}


void writeAction(Action ACTION) {


  

  int startS1 = ACTION.startPose.posSv1;
  int startS2 = ACTION.startPose.posSv2;
  int startS3 = ACTION.startPose.posSv3;

  int i = 0;

  while((startS1 != ACTION.targetPose.posSv1) || (startS2 != ACTION.targetPose.posSv2) || (startS3 != ACTION.targetPose.posSv3)){
    
    
    

    if((startS1 != ACTION.targetPose.posSv1) && (i > 25)){
      startS1 += (startS1 < ACTION.targetPose.posSv1 ? 1 : -1);
      sv1.write(startS1);
    }
    if(startS2 != ACTION.targetPose.posSv2){
      startS2 += (startS2 < ACTION.targetPose.posSv2 ? 1 : -1);
      sv2.write(startS2);
    }
    if(startS3 != ACTION.targetPose.posSv3){
      startS3 += (startS3 < ACTION.targetPose.posSv3 ? 1 : -1);
      sv3.write(startS3);
    }

    delay(15);
    Serial.println("hi");
    i++;

  }

  

  delay(1000);

  for(int i = (!ACTION.openGripper)*60; i != ACTION.openGripper*60; i += 1*ACTION.openGripper-1*(!ACTION.openGripper)){
    svGr.write(i);

    delay(25);
  }
  
  


}


void slowServo(Servo sv, int begin, int end){

  for(int i = begin; i != end; i += (begin < end ? 1 : -1)){
    sv.write(i);
    delay(25);
  }
}


void setup() {
  // Starts serial connection
  Serial.begin(9600);

  // Attaches servo instances
  sv1.attach(servo1Pin, 500, 2500);
  sv1.write(10);
  delay(100);
  sv2.attach(servo2Pin, 500, 2500);
  sv2.write(180);
  delay(100);
  sv3.attach(servo3Pin, 500, 2500);
  sv3.write(0);
  delay(100);
  svGr.attach(servoGrPin, 500, 2500);
  svGr.write(60);
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

  Serial.println("adresses:");
  Serial.print("vl1: ");
  Serial.println(vl1.getAddress(), HEX);
  Serial.print("vl2: ");
  // Serial.println(vl2.getAddress(), HEX);
  Serial.print("tcs: ");
  Serial.println(0x29, HEX);
  Serial.println("Init done!");

  // set the maximum speed, acceleration factor,
	// initial speed and the target position
	turnStepper.setMaxSpeed(1000);
	turnStepper.setAcceleration(1000);
	turnStepper.setSpeed(1000);

  
  
  
  
}

void loop() {

  
  //slowServo(sv2, 130, 180);

  // move 60 deg CCW
  turnStepper.moveTo(-267);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  writeAction(GRIPP_VER);

  slowServo(sv1, 35, 5);


  
  delay(500);


  // gipp
  // move 120 deg CW
  turnStepper.move(533);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }
  slowServo(sv3, 0, 50);
  slowServo(sv1, 5, 150);
  slowServo(sv2, 140, 140);

  slowServo(sv3, 50, 20);
  slowServo(svGr, 0, 50);
  slowServo(sv3, 20, 50);

  delay(500);
  // release
  // move 60 deg CW
  turnStepper.move(135);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }
  
  turnStepper.move(135);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }
  
  //writeAction(GRIPP_HOR);

  
  slowServo(sv3, 50, 20);
  slowServo(svGr, 50, 0);
  slowServo(sv3, 20, 50);

  delay(500);
  // gripp
  // move 90 deg CCW
  turnStepper.move(-400);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }
  

  slowServo(sv3, 50, 20);
  slowServo(svGr, 0, 50);
  slowServo(sv3, 20, 50);

  
  delay(500);

  while(1){

  turnStepper.move(133);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  slowServo(sv3, 50, 20);
  slowServo(svGr, 50, 0);
  slowServo(sv3, 20, 50);

  turnStepper.move(-800);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }
  
  slowServo(sv3, 50, 20);
  slowServo(svGr, 0, 50);
  slowServo(sv3, 20, 50);

  turnStepper.move(667);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  slowServo(sv3, 50, 20);
  slowServo(svGr, 50, 0);
  slowServo(sv3, 20, 50);

  turnStepper.move(-800);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  slowServo(sv3, 50, 20);
  slowServo(svGr, 0, 50);
  slowServo(sv3, 20, 50);

  








  turnStepper.move(133);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  slowServo(sv3, 50, 20);
  slowServo(svGr, 50, 0);
  slowServo(sv3, 20, 50);

  turnStepper.move(800);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }
  
  slowServo(sv3, 50, 20);
  slowServo(svGr, 0, 50);
  slowServo(sv3, 20, 50);

  turnStepper.move(-933);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  slowServo(sv3, 50, 20);
  slowServo(svGr, 50, 0);
  slowServo(sv3, 20, 50);

  turnStepper.move(800);
  while(turnStepper.distanceToGo() != 0){
    turnStepper.run();
  }

  slowServo(sv3, 50, 20);
  slowServo(svGr, 0, 50);
  slowServo(sv3, 20, 50);
  };
  while(1);
  
  
}
