#include <SPI.h>

SPISettings SPIset(1000000, MSBFIRST, SPI_MODE3);

#define _CS 31
#define _SCK 32
#define _MISO 0
#define _MOSI 1

#define PROD_ID 0x4A

void setup() {
  
  // put your setup code here, to run once:
  SPI.setMOSI(_MOSI);
  SPI.setMISO(_MISO);
  SPI.setSCK(_SCK);
  //SPI.setCS(_CS);
  pinMode (_CS, OUTPUT);
  SPI.begin();
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(115200,SERIAL_8N1);
  Serial1.println("Booted up");
}

void loop() {
  // put your main code here, to run repeatedly:
  SPI.beginTransaction(SPIset);
  digitalWrite(_CS, LOW);
  uint8_t msbData = SPI.transfer(PROD_ID);
  uint8_t lsbData = SPI.transfer(0x00);
  digitalWrite(_CS, HIGH);
  delayMicroseconds(25);
  uint16_t dataOut = (msbData << 8) | (lsbData & 0xFF);
  Serial1.print("Product ID: ");
  Serial1.println(dataOut);

}
