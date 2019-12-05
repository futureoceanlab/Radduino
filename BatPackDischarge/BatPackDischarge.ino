int i = 0;
float R1 = 18050;
float R2 = 990;
float K = (R1+R2)/R2;
float refVolt = 1.25;
int resolution = 1024;
float LSB = refVolt/resolution;
int analogPin = A0;
int readval = 0;
float battVoltage;
float readrate = 1;

void setup() {
  // put your setup code here, to run once:
  analogReference(EXTERNAL);
  Serial.begin(9600);

  Serial.println("CLEARSHEET");
  Serial.println("LABEL,Counter,Timer,Voltage");
}

void loop() {
  // put your main code here, to run repeatedly:
  readval = analogRead(analogPin);
  battVoltage = readval*LSB*K;
  Serial.println( (String) "DATA," + i++ + "," + millis()/1000. + "," + battVoltage + ",AUTOSCROLL_20");
  delay(1000./readrate);
}
