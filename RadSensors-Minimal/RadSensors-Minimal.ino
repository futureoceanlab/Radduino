#include <Arduino.h>
#include "FOL-TMP117.h"

#define SERIALOUT Serial

#define TMP117_RESOLUTION 0.0078125f


TMP117 tempsensor; // Initalize sensor


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
}

void loop() {
  // put your main code here, to run repeatedly:
  uint16_t rawTemp = tempsensor.readSensor();
  float TempC = rawTemp * TMP117_RESOLUTION;
  SERIALOUT.print("Temperature in C: ");
  SERIALOUT.println(TempC);
  
  delay(1000);
}
