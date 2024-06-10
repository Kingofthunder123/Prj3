#include <Arduino.h>
#include <Servo.h> 
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>
#include <TimerOne.h>

// Pins for stepper motor
const int dirPin  = 14;
const int stepPin = 15;

// MicroStep pins for stepper motor
const int uStep0 = 18;
const int uStep1 = 19;
const int uStep2 = 20;

// Enable pins for vl2 sensor and tcs sensor
const int enableVl2 = 2;

// Servo pins
const int servo1Pin  = 6;
const int servo2Pin  = 4;
const int servo3Pin  = 2;
const int servoGrPin = 7;

const int ledPin     = 45;

bool lastOne = false;


int steps;
long int stepsTaken;
// Defines motor interface type
#define motorInterfaceType 1

// Creates a stepper instance

// Creates servo instances for all joints
Servo svBase;
Servo svElbow;
Servo svWrist;
Servo svGripper;

// Creates vl1 instance for VL6180 distance sensor 
Adafruit_VL6180X vl1 = Adafruit_VL6180X();
// Adafruit_VL6180X vl2 = Adafruit_VL6180X();

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

Pos newPos;

void stepperStep(int stepDelay){


  if(steps > 0){

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);

    stepsTaken += -1*!digitalRead(dirPin) + digitalRead(dirPin);;
    

    

    steps--;
  }
  
}

void setStepTarget(bool withClock, int newSteps){
  digitalWrite(dirPin, withClock);

  steps = newSteps;
  Serial.println(steps);

}

void updateServoPos(){

  

  int baseStep = (newPos.base - servoPos.base)/abs(newPos.base - servoPos.base);
  int elbowStep = (newPos.elbow - servoPos.elbow)/abs(newPos.elbow - servoPos.elbow);
  int wristStep = (newPos.wrist - servoPos.wrist)/abs(newPos.wrist - servoPos.wrist);
  int gripperStep = (newPos.gripper - servoPos.gripper)/abs(newPos.gripper - servoPos.gripper);
  
  
  int stop = 0;
  while(stop == 0){
    stop = 1;

    if (servoPos.base != newPos.base){
      servoPos.base += baseStep;
      svBase.write(servoPos.base);
      stop = 0;
    }

    if (servoPos.elbow != newPos.elbow){
      servoPos.elbow += elbowStep;
      svElbow.write(servoPos.elbow);
      stop = 0;
    }

    if (servoPos.wrist != newPos.wrist){
      servoPos.wrist += wristStep;
      svWrist.write(servoPos.wrist);
      stop = 0;
    }

    if (servoPos.gripper != newPos.gripper){
      servoPos.gripper += gripperStep;
      svGripper.write(servoPos.gripper);
      stop = 0;
    }

    delay(15);

    
  }
}

// Custom variable to indicate the side to scan for the diabolo
typedef enum {LEFT, RIGHT} SIDE;

bool pickUp(ORIENTATION dirDiabolo, bool clockwise){

  

  
  newPos.wrist = 140;
  updateServoPos();

  delay(100);
  
  if(vl1.readRange() > 200 && dirDiabolo == HORIZONTAL){

    

    delay(200);

    newPos.base     = 30;
    updateServoPos();
    
    delay(200);

    newPos.elbow = 150;
    updateServoPos();
    delay(200);

    newPos.base = 11;
    updateServoPos();
    delay(200);

    newPos.gripper = 15;
    updateServoPos();

    delay(300);

    newPos.base     = 45;
    newPos.elbow    = 170;
    newPos.wrist    = 180;
    newPos.gripper  = 15;
    updateServoPos();

    setStepTarget(!clockwise, 770);
    while(steps > 0){
      stepperStep(900);
      Serial.println(steps);
    }
    
    newPos.base = 10;
    newPos.elbow = 130;
    newPos.wrist    = 180;
    updateServoPos();

    newPos.gripper = 50;
    updateServoPos();

    newPos.base = 30;
    updateServoPos();

    return(1);

    
  }

  else if(vl1.readRange() < 200 && dirDiabolo == VERTICAL){

    Serial.println("grip");

    setStepTarget(clockwise, 40);
    for(int i = 0; i < 40; i ++){
      stepperStep(900);
      Serial.println(steps);
    }
    if(!clockwise){
      setStepTarget(clockwise, 20);
      for(int i = 0; i < 40; i ++){
        stepperStep(900);
        Serial.println(steps);
      }
    }

    

    delay(200);

    // Position sequence to pick up the diabolo
    newPos.wrist = 105;
    updateServoPos();
    newPos.wrist = 34;
    newPos.base = 0;
    updateServoPos();

    delay(300);


    newPos.elbow = 110;
    newPos.wrist = 55;
    newPos.gripper = 15;
    updateServoPos();
    

   
    delay(300);

    newPos.base     = 20;
    updateServoPos();

    delay(200);

    if(lastOne){
      return(1);
    }

    setStepTarget(!clockwise, 800);
    while(steps > 0){
      stepperStep(900);
      Serial.println(steps);
    }

    delay(200);

    newPos.base = 0;
    newPos.elbow = 155;
    updateServoPos();

    delay(400);
    newPos.gripper = 50;
    newPos.wrist = 40;
    updateServoPos();

    delay(200);
    newPos.elbow = 170;
    newPos.wrist = 20;
    updateServoPos();
    

    setStepTarget(-1, 20);
    while(steps > 0){
    stepperStep(1200);
    }

    newPos.base = 20;
    updateServoPos();
    
    return(1);
  }

 

  return(0);
}


// Function for picking up a diabolo where "dirDiabolo" is the orientation of the diabolo and "side" is the side to scan
void scan(ORIENTATION dirDiabolo, SIDE side){
  //Timer1.start();
  // Decide side to pick up the diabolo
  bool dir;
  if(side == LEFT){
    dir = false;
  }
  else{
    dir = true;
  }

  // Rotate arm to avoid support bracket

  setStepTarget(dir, 100);
  while(steps > 0){
    stepperStep(1200);
    
  }


  delay(200);

  newPos.base     = 14;
  newPos.elbow    = 165;
  newPos.wrist    = 150;
  newPos.gripper  = 70;
  updateServoPos();

  delay(200);

  // Keep rotating arm till sesor detects diabolo

  Serial.println("ji");

  setStepTarget(dir, 800);
  while(vl1.readRange() > 150){
    for(int i = 15; i > 0; i --){
      stepperStep(800);
    }
    
  }
  setStepTarget(dir, 10);
  for(int i = 5; i > 0; i --){
      stepperStep(1200);
    }

  

  if(pickUp(dirDiabolo, dir) == 0){
    scan(dirDiabolo, side);
  }
  
  
  


  // Read sensor to check orientation and if it matches the orientation of the diabolo that needs to be picked up execude code to do so




  //Timer1.stop();
}

void blink(){
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void setup() {

  Serial.begin(9600);


  while (!Serial) {
    delay(1);
  }

  delay(20);
  // !!!VL1!!!
  // Begins communication with ToF sensor vl1
  if (!vl1.begin()) {
    Serial.println("Failed to find vl1");
    while (1);
  }
  Serial.println("Vl1 found!");

  pinMode(ledPin, OUTPUT);

  Timer1.initialize(50); // initialize timer1, and set a 1 second period

  Timer1.attachInterrupt(blink); // attaches Blink() as a timer interrupt function

  Timer1.start();

  // Initial servo positions
  newPos.base     = 35;
  servoPos.base   = 35;

  newPos.elbow    = 150;
  servoPos.elbow = 150;

  newPos.wrist    = 180;
  servoPos.wrist = 180;

  newPos.gripper  = 15;
  servoPos.gripper = 15;

  // servoPos.base     = 90;
  // servoPos.elbow    = 90;
  // servoPos.wrist    = 90;
  // servoPos.gripper  = 20;

  // Starts serial connection
  
  // Attaches servo instances and set them to initial positions
  svBase.attach(servo1Pin, 500, 2500);
  svBase.write(servoPos.base);
  delay(100);

  svElbow.attach(servo2Pin, 500, 2500);
  svElbow.write(newPos.elbow);
  delay(100);

  svWrist.attach(servo3Pin, 500, 2500);
  svWrist.write(newPos.wrist);
  delay(100);

  svGripper.attach(servoGrPin, 500, 2500);
  svGripper.write(newPos.gripper);
  delay(100);

  


  // Sets Microsteppins as output
  pinMode(uStep0, OUTPUT);
  pinMode(uStep1, OUTPUT);
  pinMode(uStep2, OUTPUT);

  // Sets 1/4 microstepping
  digitalWrite(uStep0, LOW);
  digitalWrite(uStep1, HIGH);
  digitalWrite(uStep2, LOW);
  
  // Sets pinmodes
  // pinMode(enableVl2, OUTPUT);

  // Disables ToF sensor vl2 and colour sensor
  // digitalWrite(enableVl2, LOW);

  // Waits for serial before continuing the program
  

  // vl1.setAddress(0x30);

  // // !!!VL2!!!
  // // Enables ToF sensor vl2 and wait for it to start
  // digitalWrite(enableVl2, HIGH);
  // delay(50);

  // // Begins communication with ToF sensor vl2
  // if (!vl2.begin()) {
  //   Serial.println("Failed to find vl2");
  //   while (1);
  // }
  // Serial.println("Vl2 found!");



  
  

  // set the maximum speed, acceleration factor,
	// initial speed and the target position


  
  Timer1.stop();


  delay(300);
  
  

}

void loop() {


  scan(VERTICAL, LEFT);

  setStepTarget(0, abs(stepsTaken) + 100);
  while(steps != 0){
    stepperStep(800);
  }
  
  delay(3000);

  scan(HORIZONTAL, RIGHT);

  setStepTarget(1, abs(stepsTaken));
  while(steps != 0){
    stepperStep(800);
  }

  lastOne = true;


  scan(VERTICAL, RIGHT);

  setStepTarget(0, abs(stepsTaken));
  while(steps != 0){
    stepperStep(800);
  }
  
  while(1);
  
  
}