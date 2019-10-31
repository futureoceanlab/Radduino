#include <SPI.h>

SPISettings SPIset(1000000, MSBFIRST, SPI_MODE3);

#define _CS 31
#define _SCK 32
#define _MISO 0
#define _MOSI 1

#define ENDURANCE 0x00  //Flash memory write count
#define SUPPLY_OUT 0x02 //Power supply measurement output
#define XACCL_OUT 0x04  //X-axis accelerometer output
#define YACCL_OUT 0x06  //Y-axis accelerometer output
#define AUX_ADC 0x08    //Auxialiary ADC output
#define TEMP_OUT 0x0A   //Temperature output
#define XINCL_OUT 0x0C  //X-axis inclinometer ouput, horizontal
#define YINCL_OUT 0x0E  //Y-axis inclinometer ouput, horizontal
#define ROT_OUT 0x10    //Vertical orientation output
#define XACCL_NULL 0x12 //X-axis accelerometer offset correction factor
#define YACCL_NULL 0x14 //Y-axis accelerometer offset correction factor
#define XINCL_NULL 0x16 //X-axis inclinometer offset correction factor
#define YINCL_NULL 0x18 //Y-axis inclinometer offset correction factor
#define ROT_NULL 0x1A   //Vertical orientation offset correction factor
#define ALM_MAG1 0x20   //Alarm 1 amplitude threshold
#define ALM_MAG2 0x22   //Alarm 2 amplitude threshold
#define ALM_SMPL1 0x24  //Alarm 1 sample size/time
#define ALM_SMPL2 0x26  //Alarm 2 sample size/time
#define ALM_CTRL 0x28   //Alarm control
#define AUX_DAC 0x30    //Auxilary DAC data input
#define GPIO_CTRL 0x32  //GPIO control
#define MSC_CTRL 0x34   //MISC control
#define SMPL_PRD 0x36   //Sample clock/Decimation filter control
#define AVG_CNT 0x38    //Average count control (filter setting)
#define SLP_CNT 0x3A    //Sleep mode control
#define STATUS 0x3C     //System status (error flags)
#define COMMAND 0x3E    //System global commands
#define PROD_ID 0x4A

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

//#define VERBOSE

void setup() {
  
  // put your setup code here, to run once:
  SPI1.setMOSI(_MOSI);
  SPI1.setMISO(_MISO);
  SPI1.setSCK(_SCK);
  //SPI.setCS(_CS);
  pinMode (_CS, OUTPUT);
  SPI1.begin();
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(115200,SERIAL_8N1);
  Serial1.println("Booted up");
  int16_t prodID = regRead(PROD_ID);
  Serial1.print("Product ID: ");
  Serial1.println(prodID);
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  SPI1.beginTransaction(SPIset);
  digitalWrite(_CS, LOW);
  delayMicroseconds(1);
  SPI1.transfer(PROD_ID);
  SPI1.transfer(0x00);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(50);
  digitalWrite(_CS, LOW);
  delayMicroseconds(1);
  uint8_t msbData = SPI1.transfer(0x00);
  uint8_t lsbData = SPI1.transfer(0x00);
  digitalWrite(_CS, HIGH);
  SPI1.endTransaction();
  delayMicroseconds(1000);
  uint16_t dataOut = (msbData << 8) | (lsbData & 0xFF);
  Serial1.print("Product ID: ");
  Serial1.println(dataOut);
  //*/
  if (1) {
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
  }
  delay(100);

}

uint16_t regRead(uint8_t regAddr) {
  #ifdef VERBOSE  
  Serial1.println("starting regRead");
  #endif
  SPI1.beginTransaction(SPIset);
  #ifdef VERBOSE
  Serial1.print("Writing to register address ");
  Serial1.println(regAddr);
  #endif
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1);
  SPI1.transfer(regAddr); // Write address over SPI bus
  SPI1.transfer(0x00); // Write 0x00 to the SPI bus fill the 16 bit transaction requirement
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  #ifdef VERBOSE  
  Serial1.println("Wrote Register Address");
  #endif
  delayMicroseconds(25); // Delay to not violate read rate (40us)

  // Read data from requested register
  digitalWrite(_CS, LOW); // Set CS low to enable device
  delayMicroseconds(1);
  uint8_t _msbData = SPI1.transfer(0x00); // Send (0x00) and place upper byte into variable
  uint8_t _lsbData = SPI1.transfer(0x00); // Send (0x00) and place lower byte into variable
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  #ifdef VERBOSE  
  Serial1.println("Read Register Data");
  #endif
  delayMicroseconds(25); // Delay to not violate read rate (40us)
  
  // Shift MSB data left by 8 bits, mask LSB data with 0xFF, and OR both bits
  uint16_t _dataOut = (_msbData << 8) | (_lsbData & 0xFF); // Concatenate upper and lower bytes
  SPI1.endTransaction();
  return(_dataOut);
}

void regWrite(uint8_t regAddr, int16_t regData) {
  uint16_t addr = (((regAddr & 0x7F) | 0x80) << 8); // Toggle sign bit, and check that the address is 8 bits
  uint16_t lowWord = (addr | (regData & 0xFF)); // OR Register address (A) with data(D) (AADD)
  uint16_t highWord = ((addr | 0x100) | ((regData >> 8) & 0xFF)); // OR Register address with data and increment address

  // Split words into chars
  uint8_t highBytehighWord = (highWord >> 8);
  uint8_t lowBytehighWord = (highWord & 0xFF);
  uint8_t highBytelowWord = (lowWord >> 8);
  uint8_t lowBytelowWord = (lowWord & 0xFF);

  // Write highWord to SPI bus
  SPI1.beginTransaction(SPIset);
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI1.transfer(highBytehighWord); // Write high byte from high word to SPI bus
  SPI1.transfer(lowBytehighWord); // Write low byte from high word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device

  delayMicroseconds(40); // Delay to not violate read rate (40us)

  // Write lowWord to SPI bus
  digitalWrite(_CS, LOW); // Set CS low to enable device
  SPI1.transfer(highBytelowWord); // Write high byte from low word to SPI bus
  SPI1.transfer(lowBytelowWord); // Write low byte from low word to SPI bus
  digitalWrite(_CS, HIGH); // Set CS high to disable device
  SPI1.endTransaction();
}

void grabData()
{
  // Put all the Data Registers you want to read here
  TEMP = 0;
  AX = regRead(XACCL_OUT);
  AY = regRead(YACCL_OUT);
  INCX = regRead(XINCL_OUT);
  INCY = regRead(YINCL_OUT);
  ROT = regRead(ROT_OUT);
  TEMP = regRead(TEMP_OUT);
  SUPPLY = regRead(SUPPLY_OUT);
  STATUSREG = regRead(STATUS);
}

void scaleData()
{
  AXS = (accelScale(AX) * 0.001);
  AYS = (accelScale(AY) * 0.001);
  INCXS = inclineScale(INCX);
  INCYS = inclineScale(INCY);
  ROTS = inclineScale(ROT);
  TEMPS = tempScale(TEMP);
  SUPPLYS = supplyScale(SUPPLY);
}

/////////////////////////////////////////////////////////////////////////////////////////
// Converts accelerometer data output from the regRead() function and returns
// acceleration in mg's
/////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled accelerometer in mg's
/////////////////////////////////////////////////////////////////////////////////////////
float accelScale(int16_t sensorData)
{
  int signedData = 0;
  sensorData = sensorData & 0x3FFF; // Discard upper two bits
  int isNeg = sensorData & 0x2000;
  if (isNeg == 0x2000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0x3FFF;
  else
    signedData = sensorData;
  float finalData = signedData * 0.24414; // Multiply by accel sensitivity (244.14 uG/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts incline angle data output from the regRead() function and returns incline angle
// in degrees
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled incline or rotation in degrees
/////////////////////////////////////////////////////////////////////////////////////////
float inclineScale(int16_t sensorData)
{
  int signedData = 0;
  sensorData = sensorData & 0x3FFF; // Discard upper two bits
  int isNeg = sensorData & 0x2000;
  if (isNeg == 0x2000) // If the number is negative, scale and sign the output
    signedData = sensorData - 0x3FFF;
  else
    signedData = sensorData;
  float finalData = signedData * 0.025; // Multiply by (0.025 degrees/LSB)
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts temperature data output from the regRead() function and returns temperature 
// in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled temperature in degrees Celcius
/////////////////////////////////////////////////////////////////////////////////////////
float tempScale(int16_t sensorData)
{
  sensorData = sensorData & 0x0FFF; // Discard upper two bits
  float finalData = (((sensorData - 1278) * -0.47) + 25); // Multiply by temperature scale. 25C = 0x04FE
  return finalData;
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Converts voltage supply data output from the regRead() function and returns voltage 
// in volts.
/////////////////////////////////////////////////////////////////////////////////////////////
// sensorData - data output from regRead()
// return - (float) signed/scaled voltage in Volts
/////////////////////////////////////////////////////////////////////////////////////////
float supplyScale(int16_t sensorData)
{
  sensorData = sensorData & 0x3FFF; // Discard upper two bits
  float finalData = sensorData * 0.000305176; // Multiply by 0.000305176 Volts/LSB)
  return finalData;
}
