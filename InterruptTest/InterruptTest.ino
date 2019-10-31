
#define INTERRUPTPIN 38
#define READCOUNTDELAYMICROS 250
#define SENSORREADPERIODMILLIS 100
#define HEARTBEATPERIODMILLIS 1000

#define TEMP_ID_REGISTER 0X0F
#define TEMP_ID_VALUE 0x0117  
#define TILT_ID_REGISTER 0x4A
#define TILT_ID_VALUE 0x3F51

#define SERIALBAUD 115200
const int _CS=31;
const int _SCK= 32;
const int _MISO= 0;
const int _MOSI= 1;

volatile int numFPGAreads = 0;
int numI2Creads = 0;
int numSPIreads = 0;
int numI2Cfails = 0;
int numSPIfails = 0;
int nextread = 0;
int nextbeat = 0;

#include <SPI.h>
#include <i2c_t3.h>
#include <SparkFun_TMP117.h>
#include <ADIS16209.h>

SPISettings SPIset(1000000, MSBFIRST, SPI_MODE3);

TMP117 tempsensor;
ADIS16209 tiltsensor(_CS, SPI1, SPIset);

void setup() {
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(SERIALBAUD,SERIAL_8N1);
  Serial1.println("Interrupt test");  
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), SimulateRead, FALLING);
}

void loop() {
  uint32_t timenow = millis();
  char outline[100];
  if (timenow > nextread) {
    checkI2C();
    checkSPI();
    nextread += SENSORREADPERIODMILLIS;
  }
  if (timenow > nextbeat) {
    //sprintf(outline);
    Serial1.println(outline);
    nextbeat += HEARTBEATPERIODMILLIS;
  }

}

void SimulateRead() {
  numFPGAreads++;
  delayMicroseconds(READCOUNTDELAYMICROS);  
}

void checkI2C() {
  uint16_t id;
  id = tempsensor.readRegister(TEMP_ID_REGISTER);
  if (id != TEMP_ID_VALUE) {
    numI2Cfails++;
    Serial1.println("I2C Register Read Failed");
  }  
  numI2Creads++;
}

void checkSPI() {
  uint16_t id = tiltsensor.readRegister(TILT_ID_REGISTER);
  if (id != TILT_ID_VALUE) {
    numSPIfails++;
    Serial1.println("SPI Register Read Failed");
  }
  numSPIreads++;
}
