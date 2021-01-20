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

const int      Ns[8]={ 1000, 2000, 4000, 8000, 10000, 16000, 25000, 40000}; // Possible sampling rates, 250MHz:
uint32_t time_now;
int period = 1000, oldperiod = 0; //Period in microseconds
int readPins[3] = {1, 2, 3};
int pinvals[3] = {0};
char perstring[50];

void setup() {
  pinMode(OUTPIN, OUTPUT);
  for (int i=0; i<3; ++i) {
    pinMode(readPins[i], INPUT);
  }
  time_now = micros();
  // while(!Serial){
  //   delay(1);
  // }
  delay(5000);
  Serial.begin(1200);
  Serial.println("Starting up CMod Spoofer");
}

void loop() {
  static uint16_t nsel = 0, i=0;

  if (micros() - time_now > period) {
    time_now = micros();
    digitalWrite(OUTPIN, HIGH);
    nsel = 0;
    for (i=0; i<3; ++i) {
      pinvals[i] = digitalRead(readPins[i]);
    }
    nsel = pinvals[0] + (pinvals[1] << 1) + (pinvals[2] << 2);
    period = 1000000 / Ns[nsel];
    //if (true) {
    if (period != oldperiod) {
      sprintf(perstring, "New nsel: %d, freq: %d, period: %d", nsel, Ns[nsel], period);
      Serial.println(perstring);
      oldperiod = period;
    }
    digitalWrite(OUTPIN, LOW);
  }
}
