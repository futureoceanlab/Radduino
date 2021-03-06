
#define INTERRUPTPIN 38
#define READCOUNTDELAYMICROS 500
#define SENSORREADPERIODMILLIS 100
#define HEARTBEATPERIODMILLIS 1000

#define TEMP_ID_REGISTER 0X0F
#define TEMP_ID_VALUE 0x0117  
#define TILT_ID_REGISTER 0x4A
#define TILT_ID_VALUE 0x3F51

#define SERIALBAUD 115200
const int _CS=31;
const int _SCK= 32;
const int _MISO= 1;
const int _MOSI= 0;

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

TMP117 tempsensor;
ADIS16209 tiltsensor(_CS, SPI1);

void setup() {
  //Set up Serial
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(SERIALBAUD,SERIAL_8N1);
  Serial.begin(115200);
  Serial1.println("Interrupt test doot doot");  

  //Set up TMP117 Temp Sensor
  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms
  //Serial.println("Interrupt test");
  if (tempsensor.begin() == true) {
    Serial1.println("TMP117 Temperature Sensor ready to go");
  }

  //Set up ADIS16209 Tilt Sensor
  SPI1.setMOSI(_MOSI);
  SPI1.setMISO(_MISO);
  SPI1.setSCK(_SCK);
  SPI1.begin();
  if (tiltsensor.begin() == true) {
    uint16_t id = tiltsensor.sensorTransfer(TILT_ID_REGISTER);
    Serial1.println("ADIS16209 Tilt Sensor ready to go");
  }

  //Set up Interrupt
  attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN), SimulateRead, FALLING);
}

void loop() {
  uint32_t timenow = millis();
  char outline[100];
  //sprintf(outline,"Time: %d, nextread: %d, nextbeat: %d",timenow,nextread, nextbeat);
  //Serial1.println(outline);
  if (timenow > nextread) {
    //Serial1.println("Read IDs");
    checkI2C();
    checkSPI();
    nextread += SENSORREADPERIODMILLIS;
  }
  if (timenow > nextbeat) {
    sprintf(outline,"Heartbeat: %d I2C reads, %d I2C faults, %d SPI reads, %d SPI faults, %d Interrupts",numI2Creads, numI2Cfails, numSPIreads, numSPIfails, numFPGAreads);
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
  //Serial1.println("Start I2C check");
  id = tempsensor.readRegister(TEMP_ID_REGISTER);
  if (id != TEMP_ID_VALUE) {
    numI2Cfails++;
    Serial1.println("I2C Register Read Failed");
  }  
  numI2Creads++;
  //Serial1.println("I2C check finished");
}

void checkSPI() {
  //Serial1.println("Start SPI check");
  char outline[100];
  uint16_t id = tiltsensor.sensorTransfer(TILT_ID_REGISTER);
  if (id != TILT_ID_VALUE) {
    numSPIfails++;
    sprintf(outline,"SPI Register Read Failed, expected %X, got %X",TILT_ID_VALUE, id);
    Serial1.println(outline);
  }
  numSPIreads++;
  //Serial1.println("SPI check finished");
}
