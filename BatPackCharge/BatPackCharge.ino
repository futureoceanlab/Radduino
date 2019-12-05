int i = 0;
float R1 = 18050;
float R2 = 990;
float Rsense = 0.2;
float K = (R1+R2)/R2;
float refVolt = 1.25;
int resolution = 1024;
float LSB = refVolt/resolution;
int BattVoltPin = A0;
int BattCurrPin = A1;
int currval = 0;
int voltval = 0;
float battVoltage;
float battCurrent;
float readrate = 1;

void setup() {
  // put your setup code here, to run once:
  analogReference(EXTERNAL);
  Serial.begin(9600);

  Serial.println("CLEARSHEET");
  Serial.println("LABEL,Counter,Timer,Voltage,Current");
}

void loop() {
  // put your main code here, to run repeatedly:
  currval = analogRead(BattCurrPin);
  voltval = analogRead(BattVoltPin);
  battVoltage = voltval*LSB*K - currval*LSB;
  battCurrent = currval*LSB/Rsense;
  Serial.println( (String) "DATA," + i++ + "," + millis()/1000. + "," + battVoltage + "," + battCurrent + ",AUTOSCROLL_20");
  delay(1000./readrate);
}
