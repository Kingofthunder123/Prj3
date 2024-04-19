#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();

const int enableVl2 = 2;
const int enableRGBs = 3;

void setup() {
  // Starts serial connection
  Serial.begin(9600);
  
  // Sets pinmodes
  pinMode(enableVl2, OUTPUT);
  pinMode(enableRGBs, OUTPUT);

  // Disables ToF sensor vl2 and colour sensor
  digitalWrite(enableVl2, LOW);
  digitalWrite(enableRGBs, LOW);

  // Waits for serial before continuing the program
  while (!Serial) {
    delay(1);
  }


  // !!!VL1!!!
  // Begins communication with ToF sensor vl1
  Serial.println("Sensor 1 test!");
  if (! vl1.begin()) {
    Serial.println("Failed to find sensor 1");
    while (1);
  }
  Serial.println("Sensor 1 found!");

  // Changes adress of ToF sensor vl1 from 0x29 to 0x30
  vl1.setAddress(0x30);


  // !!!VL2!!!
  // Enables ToF sensor vl2 and wait for it to start
  digitalWrite(2, HIGH);
  delay(22);

  // Begins communication with ToF sensor vl2
  Serial.println("Sensor 2 test!");
  if (! vl2.begin()) {
    Serial.println("Failed to find sensor2");
    while (1);
  }
  Serial.println("Sensor 2 found!");

  // Changes adress of ToF sensor vl2 from 0x29 to 0x31
  vl2.setAddress(0x31);


  // !!!COLOUR SENSOR!!!
  // Enable colour sensor
  digitalWrite(enableRGBs, HIGH);



  Serial.println("Init done!");
}

void loop() {

}
