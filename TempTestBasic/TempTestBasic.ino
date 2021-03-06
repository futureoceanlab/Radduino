/******************************************************************************
  SparkFun_TMP117_Breakout_Example.ino
  Example for the TMP117 I2C Temperature Sensor
  Madison Chodikov @ SparkFun Electronics
  May 29 2019
  ~

  This sketch configures the TMP117 temperature sensor and prints the
  temperature in degrees celsius and fahrenheit with a 500ms delay for
  easier readings. 

  Resources:
  Wire.h (included with Arduino IDE)
  SparkFunTMP117.h (included in the src folder)

  Development environment specifics:
  Arduino 1.8.9+
  Hardware Version 1.0.0

  This code is beerware; if you see me (or any other SparkFun employee) at
  the local, and you've found our code helpful, please buy us a round!

  Distributed as-is; no warranty is given.
******************************************************************************/

/*
  NOTE: For the most accurate readings:
  - Avoid heavy bypass traffic on the I2C bus
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
  - Place device horizontally and out of any airflow when storing
  For more information on reaching the most accurate readings from the sensor,
  reference the "Precise Temperature Measurements with TMP116" datasheet that is
  linked on Page 35 of the TMP117's datasheet
*/

//#include <Wire.h>            // Used to establish serial communication on the I2C bus
#include <Arduino.h>
//#include "i2c_t3.h"
#include "FOL-TMP117.h" // Used to send and recieve specific information from our sensor

// The default address of the device is 0x48 = (GND)
TMP117 sensor; // Initalize sensor
uint8_t regmap[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 0x0F};
char outline[100];

void setup()
{
  uint16_t regdata = 0;
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire.setDefaultTimeout(200000); // 200ms
  Serial.begin(9600);
  //Serial1.begin(115200,SERIAL_8N1);
  //Serial1.setTX(26);
  //Serial1.setRX(27);
  //Serial1.begin(115200,SERIAL_8N1);    // Start serial communication at 115200 baud
  //Wire.setClock(100000);   // Set clock speed to be the fastest for better communication (fast mode)

  Serial.println("TMP117 Example 1: Basic Readings");
  if (sensor.setupSensor() == true) // Function to check if the sensor will correctly self-identify with the proper Device ID/Address
  {
    Serial.println(sensor.dataReady());
    Serial.println("Begin");
    //sensor.setContinuousConversionMode();
    Serial.println("Starting register read");
    for (int i = 0; i < 10; i++)
    {
      regdata = sensor.readRegister(regmap[i]);
      sprintf(outline, "Register %X: %X",regmap[i], regdata);
      Serial.println(outline);
    }    
  }
  else
  {
    Serial.println("Device failed to setup- Freezing code.");
    while (1); // Runs forever
  }
}

void loop()
{
  // Data Ready is a flag for the conversion modes - in continous conversion the dataReady flag should always be high
  //Serial1.println("Starting loop");
  if (sensor.dataReady() == true) // Function to make sure that there is data ready to be printed, only prints temperature values when data is ready
  {
    
    float tempC = sensor.readTempC();
    float tempF = sensor.readTempF();
    // Print temperature in °C and °F
    Serial.println(); // Create a white space for easier viewing
    Serial.print("Temperature in Celsius: ");
    Serial.println(tempC);
    Serial.print("Temperature in Fahrenheit: ");
    Serial.println(tempF);
    delay(1000); // Delay added for easier readings
    
  }
}
