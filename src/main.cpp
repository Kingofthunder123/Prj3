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

  

  
  newPos.wrist = 140;
  updateServoPos();

  delay(100);
  
  if(vl1.readRange() > 200 && dirDiabolo == HORIZONTAL){

     turnStepper.move(clockwise * 80 + !clockwise * -80);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    delay(200);

    newPos.base     = 35;
    newPos.elbow = 180;
    updateServoPos();
    
    delay(200);

    newPos.elbow = 145;
    updateServoPos();
    delay(200);

    newPos.base = 17;
    updateServoPos();
    delay(200);

    newPos.gripper = 15;
    updateServoPos();

    delay(300);

    newPos.base     = 50;
    newPos.elbow    = 170;
    newPos.wrist    = 180;
    newPos.gripper  = 15;
    updateServoPos();

    // setStepTarget(!clockwise, 800);
    // while(steps > 0){
    //   stepperStep(900);
    //   Serial.println(steps);
    // }
    turnStepper.move(clockwise * -3200 + !clockwise * 3200);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}
    
    newPos.base = 14;
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

    turnStepper.move(clockwise * 160 + !clockwise * -160);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    if(clockwise){
      turnStepper.move(-60);
      while(turnStepper.distanceToGo() != 0){turnStepper.run();}
    }
    

    delay(200);

    // Position sequence to pick up the diabolo
    newPos.wrist = 105;
    updateServoPos();
    newPos.base = 9;
    newPos.wrist = 34;
    updateServoPos();
    newPos.base = 3;
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

      newPos.base = 40;
      newPos.elbow = 145;
      newPos.wrist    = 180;
      updateServoPos();

      return(1);
    }

    // setStepTarget(!clockwise, 830);
    // while(steps > 0){
    //   stepperStep(900);
    //   Serial.println(steps);
    // }

    turnStepper.move(clockwise * -3200 + !clockwise * 3200);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}

    delay(200);

    newPos.base = 0;
    newPos.elbow = 150;
    updateServoPos();

    delay(400);
    newPos.gripper = 50;
    newPos.wrist = 40;
    updateServoPos();

    delay(200);
    newPos.elbow = 170;
    newPos.wrist = 20;
    updateServoPos();
    

    // setStepTarget(-1, 20);
    // while(steps > 0){
    // stepperStep(1200);
    // }

    turnStepper.move(-80);
      while(turnStepper.distanceToGo() != 0){turnStepper.run();}

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

  // setStepTarget(dir, 100);
  // while(steps > 0){
  //   stepperStep(1200);
    
  // }

  turnStepper.move(dir * 400 + !dir * -400);
  while(turnStepper.distanceToGo() != 0){turnStepper.run();}

  delay(200);

  newPos.base     = 18;
  newPos.elbow    = 165;
  newPos.wrist    = 150;
  newPos.gripper  = 70;
  updateServoPos();

  delay(200);

  // Keep rotating arm till sesor detects diabolo

  Serial.println("ji");

  turnStepper.move(dir * 3200 + !dir * -3200);
  // while(turnStepper.distanceToGo() != 0){turnStepper.run();}

  // setStepTarget(dir, 800);

  turnStepper.setAcceleration(5000);
  while(vl1.readRange() > 150){
    turnStepper.move(dir * 100 + !dir * -100);
    while(turnStepper.distanceToGo() != 0){turnStepper.run();}
    
  }
  turnStepper.setAcceleration(3000);
  // setStepTarget(dir, 10);
  // for(int i = 5; i > 0; i --){
  //     stepperStep(1200);
  //   }

  // turnStepper.move(dir * 20 + !dir * -20);
  // while(turnStepper.distanceToGo() != 0){turnStepper.run();}
  

  

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
  newPos.base     = 40;
  servoPos.base   = 40;

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

      // setStepTarget(0, abs(stepsTaken) + 100);
      // while(steps != 0){
      //   stepperStep(800);
      // }

      turnStepper.moveTo(0);
      while(turnStepper.distanceToGo() != 0){turnStepper.run();}
      
      delay(3000);

      scan(HORIZONTAL, RIGHT);

      // setStepTarget(1, abs(stepsTaken)+30);
      // while(steps != 0){
      //   stepperStep(800);
      // }

      turnStepper.moveTo(0);
      while(turnStepper.distanceToGo() != 0){turnStepper.run();}

      lastOne = true;


      scan(VERTICAL, RIGHT);

      // setStepTarget(0, abs(stepsTaken) - 75);
      // while(steps != 0){
      //   stepperStep(800);
      // }

      turnStepper.moveTo(0);
      while(turnStepper.distanceToGo() != 0){turnStepper.run();}
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

  for (uint16_t i = 0; i < 150; i++){
    qtr.calibrate();
    Serial.println(i);
  }

  
  digitalWrite(45, LOW);


  
 
  // Initial servo positions
  
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


  

  delay(300);
  
  

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