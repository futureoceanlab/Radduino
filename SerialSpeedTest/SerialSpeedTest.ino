/*
  SerialPassthrough sketch

  Some boards, like the Arduino 101, the MKR1000, Zero, or the Micro, have one
  hardware serial port attached to Digital pins 0-1, and a separate USB serial
  port attached to the IDE Serial Monitor. This means that the "serial
  passthrough" which is possible with the Arduino UNO (commonly used to interact
  with devices/shields that require configuration via serial AT commands) will
  not work by default.

  This sketch allows you to emulate the serial passthrough behaviour. Any text
  you type in the IDE Serial monitor will be written out to the serial port on
  Digital pins 0 and 1, and vice-versa.

  On the 101, MKR1000, Zero, and Micro, "Serial" refers to the USB Serial port
  attached to the Serial Monitor, and "Serial1" refers to the hardware serial
  port attached to pins 0 and 1. This sketch will emulate Serial passthrough
  using those two Serial ports on the boards mentioned above, but you can change
  these names to connect any two serial ports on a board that has multiple ports.

  created 23 May 2016
  by Erik Nyquist
*/
elapsedMicros speedstart;

union Packet {
  uint8_t bytes[4];
  uint16_t shorts[4];
};

uint8_t   dataheader8[12];
uint32_t* dataheader32 = (uint32_t*)dataheader8;
uint8_t   dataword8[2];
uint16_t* dataword16 = (uint16_t*) dataword8;
uint8_t   heartbeat8[24];
uint32_t* heartbeat32 = (uint32_t*)heartbeat8;


/*
void speedtest(int datarate = 38400, int uinterval = 1000, int nmsgs = 1000) {
    Serial.println("Starting serial test");
    Serial1.begin(datarate);
    int nsent = 0;
    union Packet packet = { .shorts = {0xFC00, 0, 0, 0}};
    uint32_t nextsend = uinterval;
    speedstart = 0;
    while (nsent < nmsgs) {
        if (speedstart >= nextsend) {
            Serial1.write(packet.bytes,8);
            nsent++;
            nextsend += uinterval;
            packet.shorts[1] = speedstart;
            packet.shorts[3] = nsent;
        }
    }
    Serial1.end();
    Serial.println("Serial test complete");
}
*/

void speed_data() {
  Serial.println("Sending data packet");
  dataheader32[0] = 0x00ff00ff;
  dataheader32[1] = 1;
  dataheader32[2] = 2;
  //Serial.write(dataheader8,12);
  Serial1.write(dataheader8,12);
  for (int i=0; i<50; ++i) {
    dataword16[0] = 1024+i;
    //Serial.write(dataword8,2);
    Serial1.write(dataword8,2);
  }
}

void speed_hb() {
  Serial.println("Sending heartbeat");
  heartbeat32[0] = 0x00fe00fe;
  Serial1.write(heartbeat8,24);
}

void setup() {
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial.begin(9600);
  Serial1.begin(38400);
  delay(100);
  Serial.println("Welcome to the speedtest");
  Serial.println("hit 'd' to send data packet");
  Serial.println("hit 'h' to send heartbeat");
}

void loop() {
  static char cmd='0';
  if (Serial.available()) {      // If anything comes in Serial (USB),
    cmd = Serial.read();
    switch (cmd) {
        case 'd':
            speed_data();
            break;
        case 'h':
            speed_hb();
            break;
    }
    //Serial.println("Completed command");
  }


}
