
#define INTERRUPTPIN 38
#define READCOUNTDELAYMICROS 25
#define SENSORREADPERIODMILLIS 100
#define HEARTBEATPERIODMILLIS 1000

#define SERIALBAUD 115200

volatile int numFPGAreads = 0;
int numI2Creads = 0;
int numSPIreads = 0;
int numI2Cfails = 0;
int numSPIfails = 0;
int nextread = 0;
int nextbeat = 0;

<include SPI.h>
<include i2c_t3.h>

attachInterrupt(digitalPinToInterrupt(INTERRUPTPIN, SimulateRead, FALLING)

void setup() {
  Serial1.begin(SERIALBAUD);
  Serial1.println("Interrupt test");
}

void loop() {
  int timenow = millis;
  char outline[100];
  if (timenow > nextread) {
    checkI2C();
    checkSPI();
    nextread += SENSORREADPERIODMILLIS;
  }
  if (timenow > nextbeat) {
    sprintf(outline);
    Serial1.println(outline);
    nextbeat += HEARBEATPERIODMILLIS;
  }

}

void SimulateRead() {
  numFPGAreads++;
  delayMicroseconds(READCOUNTDELAYMICROS);  
}

void checkI2C() {
  numI2Creads++;
}

void checkSPI() {
  numSPIreads++;
}
