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

void setup() {
  Serial1.setTX(26);
  Serial1.setRX(27);
  Serial1.begin(115200,SERIAL_8N1);
  Serial1.println("Booted up");
//  Serial.begin(9600);
//  Serial.println("Booted up (USB)");
}

void loop() {
//  if (Serial1.available()) {      // If anything comes in Serial (USB),
//    Serial1.println("I got a message");
//    while (Serial1.available()) {
//      Serial1.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)
//    }
//  }
delay(1000);
Serial1.println("Ping");
//  if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)
//    Serial.write(Serial1.read());   // read it and send it out Serial (USB)
//  }
}
