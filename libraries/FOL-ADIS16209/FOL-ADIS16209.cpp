////////////////////////////////////////////////////////////////////////////////////////////////////////
//  October 2015
//  Author: Juan Jose Chong <juan.chong@analog.com>
////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ADIS16209.cpp
////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
//  This library provides all the functions necessary to interface the ADIS16209 Inclinometer with an 
//  8-Bit Atmel-based Arduino development board. Functions for SPI configuration, reads and writes,
//  and scaling are included. This library may be used for the entire ADIS1620x family of devices 
//  with some modification.
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
////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "FOL-ADIS16209.h"

////////////////////////////////////////////////////////////////////////////
// Constructor with configurable CS, DR, and RST
////////////////////////////////////////////////////////////////////////////
// CS - Chip select pin
// DR - DR output pin for data ready
// RST - Hardware reset pin
////////////////////////////////////////////////////////////////////////////
ADIS16209::ADIS16209(int CS,  SPIClass &spiPort) {
  _CS = CS;

  _spiPort = &spiPort;
  _spiPort->begin();
  //SPI.begin(); // Initialize SPI bus
  //configSPI(); // Configure SPI

//Set default pin states
  pinMode(_CS, OUTPUT); // Set CS pin to be an output
//  pinMode(_DR, INPUT); // Set DR pin to be an input
//  pinMode(_RST, OUTPUT); // Set RST pin to be an output
  digitalWrite(_CS, HIGH); // Initialize CS pin to be high
//  digitalWrite(_RST, HIGH); // Initialize RST pin to be high
}

////////////////////////////////////////////////////////////////////////////
// Destructor
////////////////////////////////////////////////////////////////////////////
ADIS16209::~ADIS16209() {
  // Close SPI bus
  //SPI.end();
}



////////////////////////////////////////////////////////////////////////////
// Sets SPI bit order, clock divider, and data mode. This function is useful
// when there are multiple SPI devices using different settings.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
/*int ADIS16209::configSPI() {
  SPI.setBitOrder(MSBFIRST); // Per the datasheet
  SPI.setClockDivider(SPI_CLOCK_DIV16); // Config for 1MHz
  SPI.setDataMode(SPI_MODE3); // Clock base at one, sampled on falling edge
  return(1);
}
*/

bool ADIS16209::setupSensor() {
  _spiSet = &_slowSPI;
  if (readRegister(PROD_ID) == 0x3F51) {
    writeRegister(SMPL_PRD, 0x0007); //
    writeRegister(AVG_CNT, 0x0006); //
    _spiSet = &_fastSPI;
    return true;
  }
  else {
    return false;
  }
}


uint16_t ADIS16209::transceiveSensor(uint8_t nextTransferReg){
  // Write register address to be read
  _spiPort->beginTransaction(*_spiSet);
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1);
  uint8_t _msbData = _spiPort->transfer(nextTransferReg); // Write address over SPI bus
  uint8_t _lsbData = _spiPort->transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  _spiPort->endTransaction();    
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits
  uint16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  return(_dataOut);
}

////////////////////////////////////////////////////////////////////////////////////////////
// Reads two bytes (one word) in two sequential registers over SPI
////////////////////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be read
// return - (int) signed 16 bit 2's complement number
////////////////////////////////////////////////////////////////////////////////////////////
uint16_t ADIS16209::readRegister(uint8_t regAddr) {
//Read registers using SPI
  
  // Write register address to be read
  _spiPort->beginTransaction(*_spiSet);
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1);
  _spiPort->transfer(regAddr); // Write address over SPI bus
  _spiPort->transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(25); // Delay to not violate read rate (40us)

  // Read data from requested register
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1);
  uint8_t _msbData = _spiPort->transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData = _spiPort->transfer(0x00); // Send (0x00) and place lower byte into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  _spiPort->endTransaction();
  delayMicroseconds(25); // Delay to not violate read rate (40us)
  
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits
  uint16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  
  return(_dataOut);
}

////////////////////////////////////////////////////////////////////////////
// Writes one byte of data to the specified register over SPI.
// Returns 1 when complete.
////////////////////////////////////////////////////////////////////////////
// regAddr - address of register to be written
// regData - data to be written to the register
////////////////////////////////////////////////////////////////////////////
int ADIS16209::writeRegister(uint8_t regAddr, uint16_t regData) {

  // Write register address and data
  uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  _spiPort->beginTransaction(*_spiSet);
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1); // Require setup time between CS Low and first clock edge
  _spiPort->transfer(highBytehighWord); // Write high byte from high word to SPI bus
  _spiPort->transfer(lowBytehighWord); // Write low byte from high word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (40us)

  // Write lowWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1);
  _spiPort->transfer(highBytelowWord); // Write high byte from low word to SPI bus
  _spiPort->transfer(lowBytelowWord); // Write low byte from low word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  _spiPort->endTransaction();
  return(1);
}

  