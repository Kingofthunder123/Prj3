#include <Arduino.h>
#include <Servo.h> 
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>
#include <TimerOne.h>
#include <QTRSensors.h>

QTRSensors qtr;


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

const int zeroButton = 39;

bool lastOne = false;


int steps;
long int stepsTaken;
// Defines motor interface type
#define motorInterfaceType 1


#define dirPinA 12
#define dirPinB 13

#define pwmPinA  3
#define pwmPinB 11

#define brkPinA  9
#define brkPinB  8

#define curPinA A0
#define curPinB A1

#define strtBtn 41

#define SwithFast 43

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];



// PARAMETERS
// VVVVVVVVVV

// PID parameters
int defSpd  = 220; // Default speed. The speed setting for straght lines.
int RturnSpd; // Turning speed. The speed setting for corners.
int LturnSpd;
int error;
int lastError = 0;

// Speed settings
int maxSpeed = 220;
int rotationSpeed = 150;
int stopSpeed = 50;

// Delays
int allignDel = 1;
int pauzeDel = 5000;
int turnDelay = 20;
int preTurnSpeed = 90;

// Creates a stepper instance
AccelStepper turnStepper(motorInterfaceType, stepPin, dirPin);

// CUSTOM VARIABLES
// VVVVVVVVVVVVVVVV

typedef enum {MOTOR_R, MOTOR_L} SELECT_MOTOR;
typedef enum {FORWARD, REVERSE} DIRECTION;

// Custom variable to indicate the side to scan for the diabolo
typedef enum {LEFT, RIGHT} SIDE;
typedef enum {HORIZONTAL, VERTICAL} ORIENTATION;

bool stop = false;

int normArray[8];

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



// FUNCTIONS
// VVVVVVVVV
void stepperStep(int stepDelay){


  if(steps > 0){

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);

    stepsTaken += -1*!digitalRead(dirPin) + digitalRead(dirPin);
    

    

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


bool pickUp(ORIENTATION dirDiabolo, bool clockwise){

  

  
  newPos.wrist = 146;
  updateServoPos();

  delay(100);
  
  if(vl1.readRange() > 200 && dirDiabolo == HORIZONTAL){

    turnStepper.move(clockwise * 50 + !clockwise * -50);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    delay(200);

    newPos.base     = 40;
    newPos.elbow = 180;
    updateServoPos();
    
    delay(200);

    newPos.elbow = 145;
    updateServoPos();
    delay(200);

    newPos.base = 19;
    updateServoPos();
    delay(200);

    newPos.gripper = 0;
    updateServoPos();

    delay(300);

    newPos.base     = 60;
    newPos.elbow    = 170;
    newPos.wrist    = 180;
    newPos.gripper  = 0;
    updateServoPos();

    turnStepper.move(clockwise * -3200 + !clockwise * 3200);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}
    
    newPos.base = 20;
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

    turnStepper.move(clockwise * -20);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    newPos.base = 23;
    newPos.gripper = 60;
    updateServoPos();

    
    

    delay(200);

    // Position sequence to pick up the diabolo
    newPos.wrist = 110;
    updateServoPos();

    newPos.elbow = 165;
    newPos.base = 13;
    newPos.wrist = 75;
    updateServoPos();

    newPos.wrist = 20;
    updateServoPos();

   
    newPos.base = 2;
    newPos.elbow = 110;
    newPos.wrist = 35;
    newPos.gripper = 4;
    updateServoPos();
    

   
    delay(300);

    newPos.wrist = 30;
    newPos.base     = 30;
    updateServoPos();

    delay(200);

    if(lastOne){

      newPos.base = 50;
      newPos.elbow = 145;
      newPos.wrist    = 180;
      updateServoPos();

      return(1);
    }


    turnStepper.move(clockwise * -3200 + !clockwise * 3200);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    delay(200);

    newPos.base = 0;
    newPos.elbow = 145;
    updateServoPos();

    delay(400);
    newPos.gripper = 50;
    // newPos.wrist = 40;
    updateServoPos();

    delay(200);
    newPos.elbow = 170;
    newPos.wrist = 20;
    updateServoPos();
    
    turnStepper.move(-110);
      while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    newPos.base = 25;
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

  

  turnStepper.move(dir * 500 + !dir * -500);
  while(turnStepper.distanceToGo() != 0){turnStepper.run();}

  delay(200);

  newPos.base     = 23;
  newPos.elbow    = 163;
  newPos.wrist    = 158;
  newPos.gripper  = 70;
  updateServoPos();

  delay(200);

  // Keep rotating arm till sesor detects diabolo

  Serial.println("ji");

 
  turnStepper.setAcceleration(5000);
  while(vl1.readRange() > 200){
    turnStepper.move(dir * 200 + !dir * -200);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}
    
  }
  turnStepper.setAcceleration(3000);

  while(vl1.readRange() < 200){
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
  }

  turnStepper.move(dir * -90 + !dir * 90);
  while(turnStepper.distanceToGo() != 0){turnStepper.run();}

  if(pickUp(dirDiabolo, dir) == 0){
    scan(dirDiabolo, side);
  }
  
  
  


  // Read sensor to check orientation and if it matches the orientation of the diabolo that needs to be picked up execude code to do so




  //Timer1.stop();
}

// Change the speed depending on the current switch position  (fast or slow)
void speedControl() {
  if(digitalRead(SwithFast) == HIGH){
    defSpd = 255;
    rotationSpeed = 180;
    stopSpeed = 15;
    turnDelay = 175;
    preTurnSpeed = 90;
  }
  else {
    defSpd = 255;
    rotationSpeed = 140;
    stopSpeed = 15;
    turnDelay = 100;
    preTurnSpeed = 100;
  }

}

// Function to simplify the motor control
void driveMotor(SELECT_MOTOR motor, DIRECTION dirSel, int power, bool brake = false){

  int dir = 0;
  int pwm = 0;
  int brk = 0;

  if(motor == MOTOR_R){
    dir = 12;
    pwm = 3;
    brk = 9;
  }
  else{
    dir = 13;
    pwm = 11;
    brk = 8;
  }


  if(dirSel == FORWARD){
    digitalWrite(dir, HIGH);
  }
  else{
    digitalWrite(dir, LOW);
  }

  analogWrite(pwm, power);

  digitalWrite(brk, brake);

}

// MOST IMPORTANT FUNCTION to make the turn
int makeTurn(){

  SELECT_MOTOR turnFor;
  SELECT_MOTOR turnRev;

  if((normArray[0] == 1  && normArray[7] == 0)){
    Serial.println("RIGHT");
    turnFor = MOTOR_R;
    turnRev = MOTOR_L;
  }
  else if((normArray[0] == 0  && normArray[7] == 1)){
    Serial.println("LEFT");
    turnFor = MOTOR_L;
    turnRev = MOTOR_R;
  }
  else{
    return(0);
  }

  Serial.println("turning");

  driveMotor(turnFor, FORWARD, preTurnSpeed);
  driveMotor(turnRev, FORWARD, preTurnSpeed);

  //DO NOT CHANGE!!!
  delay(turnDelay);

  // unsigned int stepNrStart = nrStepsA;

  // while(nrStepsA < stepNrStart + stepGoal){
  // }
  
  driveMotor(MOTOR_R, FORWARD, 0, true);
  driveMotor(MOTOR_L, FORWARD, 0, true);


  
  driveMotor(turnFor, FORWARD, rotationSpeed);
  driveMotor(turnRev, REVERSE, rotationSpeed);
  
  

  qtr.readLineBlack(sensorValues);
  
  int stopt = 0;

  while(stopt == 0){
    
    qtr.readLineBlack(sensorValues);
    if(sensorValues[3] > 800 || sensorValues[4] > 800 ){
      stopt = 1;
    }
  }

  

  return(0);
}

// Refresh sensor
void readSensor(unsigned int threshold){
  qtr.readLineBlack(sensorValues);
  
  for(int i = 0; i < SensorCount; i++){
    
    if(sensorValues[i] >= threshold){
      normArray[i] = 1;
    }
    else{
      normArray[i] = 0;
    }
    Serial.print(normArray[i]);
    Serial.print(" ");
  }

  Serial.println();
}

// PID control function
void PIDSteer(){
  //PID control

  uint16_t position = qtr.readLineBlack(sensorValues);

  error = position - 3500;

  RturnSpd = defSpd - (2 * error + 3 * (error - lastError));
  LturnSpd = defSpd + (2 * error + 3 * (error - lastError));

  lastError = error;
    
  if(RturnSpd > maxSpeed){
    RturnSpd = maxSpeed;
  }
  if(LturnSpd > maxSpeed){
    LturnSpd = maxSpeed;
  }

  driveMotor(MOTOR_R, FORWARD, RturnSpd);
  driveMotor(MOTOR_L, FORWARD, LturnSpd);

}

void blink(){
  digitalWrite(ledPin, !digitalRead(ledPin));
}

void exePauze(){
  newPos.base     = 50;
  servoPos.base   = 50;

  newPos.elbow    = 150;
  servoPos.elbow = 150;

  newPos.wrist    = 180;
  servoPos.wrist = 180;

  newPos.gripper  = 15;
  servoPos.gripper = 15;

  // Starts serial connection
  
  // Attaches servo instances and set them to initial positions
  svBase.attach(servo1Pin, 500, 2500);
  svBase.write(servoPos.base);
  delay(200);

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
  digitalWrite(uStep0, HIGH);
  digitalWrite(uStep1, HIGH);
  digitalWrite(uStep2, LOW);

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


      scan(VERTICAL, LEFT);

      turnStepper.setMaxSpeed(800);
      turnStepper.move(-3200);
      while(turnStepper.distanceToGo() != 0 && !digitalRead(zeroButton)){turnStepper.run();}
      turnStepper.setMaxSpeed(3000);
      
      delay(3000);

      turnStepper.move(100);
      while(turnStepper.distanceToGo() != 0){
        turnStepper.run();
      }

      scan(HORIZONTAL, RIGHT);

      turnStepper.setMaxSpeed(900);
      turnStepper.move(3200);
      while(turnStepper.distanceToGo() != 0 && !digitalRead(zeroButton)){turnStepper.run();}
      turnStepper.setMaxSpeed(3000);
      lastOne = true;
     
      turnStepper.move(600);
      while(turnStepper.distanceToGo() != 0){
        turnStepper.run();
      }

     

      scan(VERTICAL, RIGHT);

      turnStepper.setMaxSpeed(500);
      turnStepper.moveTo(-3200);
      while(turnStepper.distanceToGo() != 0 && !digitalRead(zeroButton)){turnStepper.run();}
      turnStepper.setMaxSpeed(3000);
}

void pauzeStop(){
  if(normArray[0] == 1 && normArray[7] == 1){
      
    driveMotor(MOTOR_R, FORWARD, 0, true);
    driveMotor(MOTOR_L, FORWARD, 0, true);

    delay(10);

    driveMotor(MOTOR_R, FORWARD, stopSpeed);
    driveMotor(MOTOR_L, FORWARD, stopSpeed);

    int starttime = millis();

    while(normArray[0] == 1 || normArray[7] == 1){
      readSensor(700);
    }
    int endTime = millis();

    delay(allignDel);

    driveMotor(MOTOR_R, FORWARD, 0, true);
    driveMotor(MOTOR_L, FORWARD, 0, true);

      
    // millis delay for the pause
    if(endTime - starttime < 500){
      stop = false;

      exePauze();

      
  
    }
    else{
      stop = true;
    }
  }

  for(int i = 0; i < SensorCount; i++){
    Serial.print(normArray[i]);
    Serial.print("\t");
  }

}









void setup() {

  Serial.begin(9600);


  Serial.begin(9600);

  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);

  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);

  pinMode(brkPinA, OUTPUT);
  pinMode(brkPinB, OUTPUT);

  pinMode(curPinA, INPUT);
  pinMode(curPinB, INPUT);

  pinMode(SwithFast, INPUT);

  turnStepper.setMaxSpeed(3000);
  turnStepper.setAcceleration(3000);
  turnStepper.setSpeed(3000);

  
  

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);
  qtr.setEmitterPin(47);

  delay(500);

  pinMode(45, OUTPUT);
  
  digitalWrite(45, HIGH);

  // for (uint16_t i = 0; i < 150; i++){
  //   qtr.calibrate();
  //   Serial.println(i);
  // }

  
  digitalWrite(45, LOW);

}

void loop() {

   while(digitalRead(strtBtn) != HIGH) {}
  stop = false;

  

  //main while loop
  while(!stop){  

    // PIDSteer();

    // readSensor(750);

    // makeTurn();

    // pauzeStop();



    exePauze();

    
    stop = true;
    
  }


  
  
  
}