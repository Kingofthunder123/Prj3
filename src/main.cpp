#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_VL6180X.h>
#include <SPI.h>
#include <AccelStepper.h>
#include <Adafruit_TCS34725.h>

Adafruit_VL6180X vl1 = Adafruit_VL6180X();
Adafruit_VL6180X vl2 = Adafruit_VL6180X();
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

const int enableVl2 = 2;
const int enableTcs = 3;

void setup() {
  // Starts serial connection
  Serial.begin(9600);
  
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


  // !!!VL2!!!
  // Enables ToF sensor vl2 and wait for it to start
  digitalWrite(2, HIGH);
  delay(50);

  // Begins communication with ToF sensor vl2
  if (!vl2.begin()) {
    Serial.println("Failed to find vl2");
    while (1);
  }
  Serial.println("Vl2 found!");

  // Changes adress of ToF sensor vl2 from 0x29 to 0x31
  vl2.setAddress(0x31);


  // !!!COLOUR SENSOR!!!
  // Enable colour sensor
  digitalWrite(enableTcs, HIGH);
  delay(50);

  // Begins communication with colour sensor tcs
  if (!tcs.begin()) {
    Serial.println("Failed to find tcs");
    while (1);
  }
  Serial.println("Tcs found!");

  Serial.println("adresses:");
  Serial.print("vl1: ");
  Serial.println(vl1.getAddress());
  Serial.print("vl2: ");
  Serial.println(vl2.getAddress());
  Serial.print("tcs: ");
  Serial.println(0x29);
  Serial.println("Init done!");
  
}

void loop() {

}
