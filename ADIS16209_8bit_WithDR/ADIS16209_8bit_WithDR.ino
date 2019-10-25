////////////////////////////////////////////////////////////////////////////////////////////////////////
//  October 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16229.ino
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This Arduino project interfaces with an ADIS16209 using SPI and the accompanying C++ libraries, 
//  reads inclinometer data in LSBs, scales the data, and outputs measurements to a serial 
//  debug terminal (putty) via the onboard USB serial port.
//
//  This project has been tested on an Arduino Duemilanove and Uno, but should be compatible with any other
//  8-Bit Arduino embedded platform. 
//
//  This example is free software. You can redistribute it and/or modify it
//  under the terms of the GNU Lesser Public License as published by the Free Software
//  Foundation, either version 3 of the License, or any later version.
//
//  This example is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
//  FOR A PARTICULAR PURPOSE.  See the GNU Lesser Public License for more details.
//
//  You should have received a copy of the GNU Lesser Public License along with 
//  this example.  If not, see <http://www.gnu.org/licenses/>.
//
//  Pinout for Arduino Uno/Diecimila/Duemilanove
//  Gray = RST = 4
//  Purple = SCLK = 13
//  Blue = CS = 7
//  Green = DOUT(MISO) = 11
//  Yellow = DIN(MOSI) = 12
//  Brown = GND
//  Red = VCC [3.3V ONLY]
//  White = DR = 2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
//#include <spi4teensy3.h>
#include <SPI.h>
#include <ADIS16209.h>



#define DEBUG
// Initialize Variables
// Accelerometer
int AX = 0;
int AY = 0;
float AXS = 0;
float AYS = 0;
// Inclinometer
int INCX = 0;
int INCY = 0;
float INCXS = 0;
float INCYS = 0;
// Rotation
int ROT = 0;
float ROTS = 0;
// Control Registers
int MSC = 0;
int SENS = 0;
int SMPL = 0;
int PRODID = 0;
int STATUSREG = 0xFF;
// Temperature
int TEMP = 0;
float TEMPS = 0;
// Supply
int SUPPLY = 0;
float SUPPLYS = 0;

// Data Ready Flag
boolean validData = false;


//SPI.setMOSI(1)
//SPI.setMISO(0)
//SPI.setSCK(32)
// Call ADIS16209 Class
ADIS16209 INCL(31,2,4); //ChipSelect,DataReady,Reset Pin Assignments

void setup()
{
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(115200,SERIAL_8N1);
  //Serial.begin(115200); // Initialize serial output via USB
  INCL.configSPI(); // Configure SPI communication
  
  #ifdef DEBUG
    Serial1.println("**********DEBUG MODE**********");
  #endif
  
  delay(100); // Give the part time to start up
  INCL.regWrite(MSC_CTRL,0x6);  // Enable Data Ready on INCL
  delay(20); 
  INCL.regWrite(AVG_CNT,0x8); // Set Digital Filter on INCL
  delay(20);
  INCL.regWrite(SMPL_PRD,0x14), // Set Decimation on INCL
  delay(20);
  
  // Read the control registers once to print to screen
  MSC = INCL.regRead(MSC_CTRL);
  SENS = INCL.regRead(AVG_CNT);
  SMPL = INCL.regRead(SMPL_PRD);
  PRODID = INCL.regRead(PROD_ID);
  Serial1.println("Control Registers");
  Serial1.print("MSC_CTRL: ");
  Serial1.println((unsigned char)MSC,HEX);
  Serial1.print("SENS_AVG: ");
  Serial1.println((unsigned char)SENS,HEX);
  Serial1.print("SMPL_PRD: ");
  Serial1.println((unsigned char)SMPL,HEX);
  Serial1.print("PROD_ID: ");
  Serial1.println(PRODID);
  attachInterrupt(0, setDRFlag, RISING); // Attach interrupt to pin 2. Trigger on the rising edge
  
}

// Function used to read register values via SPI and load them into variables in LSBs
void grabData()
{
  // Put all the Data Registers you want to read here
  TEMP = 0;
  AX = INCL.regRead(XACCL_OUT);
  AY = INCL.regRead(YACCL_OUT);
  INCX = INCL.regRead(XINCL_OUT);
  INCY = INCL.regRead(YINCL_OUT);
  ROT = INCL.regRead(ROT_OUT);
  TEMP = INCL.regRead(TEMP_OUT);
  SUPPLY = INCL.regRead(SUPPLY_OUT);
  STATUSREG = INCL.regRead(STATUS);
}

// Function used to scale all acquired data (scaling functions are included in ADIS16209.cpp)
void scaleData()
{
  AXS = (INCL.accelScale(AX) * 0.001);
  AYS = (INCL.accelScale(AY) * 0.001);
  INCXS = INCL.inclineScale(INCX);
  INCYS = INCL.inclineScale(INCY);
  ROTS = INCL.inclineScale(ROT);
  TEMPS = INCL.tempScale(TEMP);
  SUPPLYS = INCL.supplyScale(SUPPLY);
}

// Data Ready Interrupt Routine
void setDRFlag()
{
  validData = !validData;
}

// Main loop. Scale and display registers read using the interrupt
void loop()
{
  #ifdef DEBUG
  Serial1.println("Starting Main Loop");
  #endif
  
  if (1) // If data present in the ADIS16209 registers is valid...
  {
    grabData(); // Grab data from the INCL
    
    scaleData(); // Scale data acquired from the INCL
    
    // Print header
    Serial1.println("ADIS16209 Arduino Example Program");
    Serial1.println("October 2015 - Juan J Chong");
    Serial1.println(" ");
    
    //Print control registers to the serial port
    Serial1.println("Control Registers");
    Serial1.print("MSC_CTRL: ");
    Serial1.println((unsigned char)MSC,HEX);
    Serial1.print("SENS_AVG: ");
    Serial1.println((unsigned char)SENS,HEX);
    Serial1.print("SMPL_PRD: ");
    Serial1.println((unsigned char)SMPL,HEX);
    Serial1.print("PROD_ID: ");
    Serial1.println(PRODID);
    Serial1.print("STATUS: ");
    Serial1.println(STATUSREG,HEX);
    Serial1.println(" ");
    Serial1.println("Data Registers");
    
    //Print scaled accel data
    Serial1.print("XACCL: ");
    Serial1.println(AXS);
    Serial1.print("YACCL: ");
    Serial1.println(AYS);
    Serial1.println(" ");

    //Print scaled accel data
    Serial1.print("XINCL: ");
    Serial1.println(INCXS);
    Serial1.print("YINCL: ");
    Serial1.println(INCYS);
    Serial1.print("ROT: ");
    Serial1.println(ROTS);
    Serial1.println(" ");
   
    //Print scaled temp data
    Serial1.print("TEMP: ");
    Serial1.println(TEMPS);
    Serial1.print("Supply: ");
    Serial1.println(SUPPLYS);
   
    delay(150); // Give the user time to read the data
    
    //Clear the serial terminal and reset cursor
    //Only works on supported serial terminal programs (PuTTY)
    //Serial1.print("\033[2J");
    //Serial1.print("\033[H");
  }
  delay(1000);
}
