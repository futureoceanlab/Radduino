#include <arduino.h>
#include <SPI.h>

#define US_DELAY 1
#define CSPIN 9
#define EN_HIGH 16
#define EN_LOW 19
#define MODE 1

const int maxval = 1<<16;
const float vref=1.5;
float outarray[11];
SPISettings mySetting(5000000, MSBFIRST, SPI_MODE0);


void setup() {
    pinMode(CSPIN, OUTPUT);
    pinMode(EN_HIGH, OUTPUT);
    pinMode(EN_LOW, OUTPUT);
    digitalWrite(CSPIN, HIGH);
    digitalWrite(EN_LOW, HIGH);
    digitalWrite(EN_HIGH, LOW);    
    SPI.begin();
    for (int i=0; i<= 10; i++){
        outarray[i] = 0.1 * i;
    }
}

void loop() {
    static uint16_t i=0;
    #if MODE == 0
        for (i = 0; i < maxval; i++) {
            SPI.beginTransaction(mySetting);
            digitalWrite(CSPIN, LOW);
            SPI.transfer16(i);
            digitalWrite(CSPIN, HIGH);
            SPI.endTransaction();
            delayMicroseconds(US_DELAY);
        }
    #elif MODE == 1
        static uint16_t outval;
        for (i = 0; i<= 10; i++){
            outval = lroundf(maxval*outarray[i]/vref);
            writeDAC(outval);
            delay(1000);
        }
    #endif

}

void writeDAC(uint16_t val) {
    SPI.beginTransaction(mySetting);
    digitalWrite(CSPIN, LOW);
    SPI.transfer16(val);
    digitalWrite(CSPIN, HIGH);
    SPI.endTransaction();
}