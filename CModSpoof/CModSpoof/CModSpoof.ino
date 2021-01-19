/*
  Blink without Delay

  Turns on and off a light emitting diode (LED) connected to a digital pin,
  without using the delay() function. This means that other code can run at the
  same time without being interrupted by the LED code.

  The circuit:
  - Use the onboard LED.
  - Note: Most Arduinos have an on-board LED you can control. On the UNO, MEGA
    and ZERO it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN
    is set to the correct LED pin independent of which board is used.
    If you want to know what pin the on-board LED is connected to on your
    Arduino model, check the Technical Specs of your board at:
    https://www.arduino.cc/en/Main/Products

  created 2005
  by David A. Mellis
  modified 8 Feb 2010
  by Paul Stoffregen
  modified 11 Nov 2013
  by Scott Fitzgerald
  modified 9 Jan 2017
  by Arturo Guadalupi

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/BlinkWithoutDelay
*/

#define OUTPIN 0

const uint16_t      Ns[]={ 1000, 2000, 4000, 8000, 10000, 16000, 25000, 40000}; // Possible sampling rates, 250MHz:
uint32_t time_now;
uint8_t period = 1000; //Period in microseconds
int readPins[] = {1, 2, 3}

void setup() {
  pinMode(OUTPIN, OUTPUT);
  for (i=0; i<3; ++i) {
    pinMode(readPins[i], INPUT);
  }
  time_now = micros();  
}

void loop() {
  static uint16_t nsel = 0;

  if (micros() - time_now > period) {
    time_now = micros()
    digitalWrite(OUTPIN, HIGH);
    nsel = 0;
    for (i=0; i<3; ++i) {
      if (digitalRead(readPins[i]) == HIGH) {
        nsel |= 1 << i;
      }
    }
    period = Ns[nsel]
    delayMicroseconds(10);
    digitalWrite(OUTPIN, LOW);
  }
}
