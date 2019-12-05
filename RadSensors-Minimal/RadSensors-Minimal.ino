#include <Arduino.h>
#include "FOL-TMP117.h"
#include "FOL-ADIS16209.h"

#define SERIALOUT Serial



//Defines for reading and interpreting tilt sensor
#define _CS 31
#define _SCK 32
#define _MISO 0
#define _MOSI 1

TMP117 tempsensor; // Initalize sensor
ADIS16209 tiltsensor; // Initialize tilt sensor

uint16_t rawTemp, rawTilt;
float TempC, angle;

void setup() {
  // Initialize Serial
  Serial.begin(9600);
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(115200,SERIAL_8N1);    // Start serial communication at 115200 baud

  // Initialize TMP117 I2C Temperature Sensor
  if (tempsensor.setupSensor() == true) {
    SERIALOUT.println("TMP117 set up successfully");
  }
  else {
    SERIALOUT.println("TMP117 failed to set up");
  }

  // Initialize ADIS16209 Tilt Sensor
  if (tiltsensor.setupSensor() == true) {
    SERIALOUT.println("ADIS16209 set up successfully");
  }
  else{
    SERIALOUT.println("ADIS16209 failed to set up");
  }
  tiltsensor.transceiveSensor(PROD_ID);
  delayMicroseconds(40);
}

void loop() {
  // put your main code here, to run repeatedly:
  //temp sensor read
  rawTemp = tempsensor.readSensor();
  TempC = rawTemp * TMP117_RESOLUTION;
  SERIALOUT.print("Temperature in C: ");
  SERIALOUT.println(TempC);
  
  //tilt sensor read
  rawTilt = tiltsensor.transceiveSensor(XINCL_OUT);
  SERIALOUT.print("Product ID: ");
  //SERIALOUT.println(angle);
  SERIALOUT.println(rawTilt);
  delayMicroseconds(40);

  rawTilt = tiltsensor.transceiveSensor(YINCL_OUT);
  SERIALOUT.print("X inclination in degrees: ");
  angle = rawTilt * SCALE_ANGLE;
  //SERIALOUT.println(angle);
  SERIALOUT.println(rawTilt);
  delayMicroseconds(40);

  rawTilt = tiltsensor.transceiveSensor(ROT_OUT);
  SERIALOUT.print("Y inclination in degrees: ");
  angle = rawTilt * SCALE_ANGLE;
  //SERIALOUT.println(angle);
  SERIALOUT.println(rawTilt);
  delayMicroseconds(40);

  rawTilt = tiltsensor.transceiveSensor(PROD_ID);
  SERIALOUT.print("Rotation in degrees: ");
  angle = rawTilt * SCALE_ANGLE;
  //SERIALOUT.println(angle);
  SERIALOUT.println(rawTilt);
  delayMicroseconds(40);

  delay(1000);
}
