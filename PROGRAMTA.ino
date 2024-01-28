#include <MCP41_Simple.h>
MCP41_Simple MyPot;
#define WINDOW_SIZE 10
#include <EEPROM.h>

bool sudahDiinisialisasi = false;
bool isCharging = false; // Flag to indicate whether the battery is charging
int INDEX = 0;
float VALUE = 0;
float SUM = 0;
float READINGS[WINDOW_SIZE];
float AVERAGED = 0;
float actualCurrent = 0;
float actualVoltage;
float actualVoltage2;
const uint8_t  CS_PIN      = 53;
const uint8_t  motor = 11;
const uint8_t  rev = 10;
const uint8_t  lowBrake = 9;
const uint8_t  gearOne = 7;
const uint8_t  gearThree = 6;
const uint8_t  fan = 5;
int setgraph;

const byte PulsesPerRevolution = 2;
const unsigned long ZeroTimeout = 100000;
const byte numReadings = 20;

volatile unsigned long LastTimeWeMeasured;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned long FrequencyRaw;
unsigned long FrequencyReal;
unsigned long RPM;
unsigned int PulseCounter = 1;
unsigned long PeriodSum;
unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;
unsigned long readings[numReadings];
unsigned long readIndex;
unsigned long total;
unsigned long average;

const int pinADC = A2;
double sensitivitas = 66;
double nilaiadc = 00;
double teganganoffset = 2500;
float tegangan = 00;
float nilaiarus = 00;

const int analogPin = A4;
float Vmodul = 0.0;
float hasil = 0.0;
float R1 = 253000.0;
float R2 = 14500.0;
float value = 0;
float Vmodul2 = 0.0;
float hasil2 = 0.0;
float value2 = 0;
float soc;
float Soc;
float SOC;
float batteryCapacity;
float totalCharge = 0.0;
float prevvTime = 0.0;
int plus = 0;
float voltageOC;
float volt;
float volt2;

//Aktuator Pin
int ena = 12;
int in1 = 51;
int in2 = 13;

double proportional, derivative, integral, previous, potValue = 0;

void saveSOC() {
  // Simpan nilai SOC ke EEPROM
  int address = 0;
  EEPROM.put(address, soc);
}
void loadSOC() {
  // Baca nilai SOC dari EEPROM
  int address = 0;
  EEPROM.get(address, Soc);
}
void Current_Measurement() {
  float adc = analogRead(pinADC);
  float voltage = adc * 5 / 1023.0;
  float current = (voltage - 2.5) / 0.066;
  float mcurrent = current * 1000;

  SUM = SUM - READINGS[INDEX];
  VALUE = current;
  READINGS[INDEX] = VALUE;
  SUM = SUM + VALUE;
  INDEX = (INDEX + 1) % WINDOW_SIZE;

  AVERAGED = SUM / WINDOW_SIZE;
  actualCurrent = (0.9563 * AVERAGED) + 0.2322;
  // Ensure that current is not less than 0
  if (actualCurrent < 0) {
    actualCurrent = 0;
  }

  // Calculate time difference since the last measurement

  float currenttTime = millis() / 1000; // convert milliseconds to seconds
  float timeInSeconds = currenttTime - prevvTime;

  // Update total charge based on the current and time for discharge
  totalCharge += actualCurrent * timeInSeconds;

  // Update SOC based on the total charge and battery capacity
  if (!isCharging) {
    soc = Soc;
    soc = soc - (totalCharge / 180000) * 100.0;
    SOC = soc;
    int actualSOC = SOC * 100;
    int actualC = actualCurrent * 100;
    Serial1.print("battery.val=");
    Serial1.print(actualSOC);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("current.val=");
    Serial1.print(actualC);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
    if (SOC >= 85 && SOC <= 100) {
      Serial1.print("bat.pic=");
      Serial1.print("86");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 70 && SOC < 85) {
      Serial1.print("bat.pic=");
      Serial1.print("85");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 55 && SOC < 70) {
      Serial1.print("bat.pic=");
      Serial1.print("84");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 40 && SOC < 55) {
      Serial1.print("bat.pic=");
      Serial1.print("83");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 20 && SOC < 40) {
      Serial1.print("bat.pic=");
      Serial1.print("82");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (actualSOC >= 0 && SOC < 20) {
      Serial1.print("bat.pic=");
      Serial1.print("81");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 70 && SOC < 75) {
      Serial1.print("bat.pic=");
      Serial1.print("80");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 50 && SOC < 75) {
      Serial1.print("bat.pic=");
      Serial1.print("86");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 25 && SOC < 50) {
      Serial1.print("bat.pic=");
      Serial1.print("85");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 15 && SOC < 25) {
      Serial1.print("bat.pic=");
      Serial1.print("84");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 5 && SOC < 15) {
      Serial1.print("bat.pic=");
      Serial1.print("83");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 1 && SOC < 5) {
      Serial1.print("bat.pic=");
      Serial1.print("82");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (SOC >= 0 && SOC < 1) {
      Serial1.print("bat.pic=");
      Serial1.print("81");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    // Update previous time for the next iteration
    prevvTime = currenttTime;
  }
}

void Voltage_Awal() {
  value = analogRead(analogPin);
  Vmodul = (value * 5.0) / 1023.0;
  hasil = Vmodul / (R2 / (R1 + R2));
  actualVoltage = (1.0856 * hasil) + 0.2082;
//  actualVoltage = (1.0115 * hasil) + 1.1197;
  if (actualVoltage >= 37) {
    volt = ( 0.255 * actualVoltage ) + 36.155;
  }
  Soc = ((volt - 37) / 11) * 100;
}

void Voltage_Measurement() {
  value2 = analogRead(analogPin);
  Vmodul2 = (value2 * 5.0) / 1023.0;
  hasil2 = Vmodul2 / (R2 / (R1 + R2));
  actualVoltage2 = (1.0856 * hasil2) + 0.2082;
  if (actualVoltage2 >= 37) {
    volt2 = ( 0.255 * actualVoltage2 ) + 36.155;
  }
  int actualV = volt2 * 100;
  Serial1.print("voltage.val=");
  Serial1.print(actualV);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void Pulse_Event()
{
  PeriodBetweenPulses = micros() - LastTimeWeMeasured;
  LastTimeWeMeasured = micros();
  if (PulseCounter >= AmountOfReadings)
  {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter = 1;
    PeriodSum = PeriodBetweenPulses;
    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);
    AmountOfReadings = RemapedAmountOfReadings;
  }
  else
  {
    PulseCounter++;
    PeriodSum = PeriodSum + PeriodBetweenPulses;
  }
}
double kalrpm;
double kalman(double U) {
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 1;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P * H / (H * P * H + R);
  U_hat += + K * (U - H * U_hat);
  P = (1 - K * H) * P + Q;
  return U_hat;
}

unsigned long prevTime = millis();

void rpmCounter() {

  LastTimeCycleMeasure = LastTimeWeMeasured;
  CurrentMicros = micros();
  if (CurrentMicros < LastTimeCycleMeasure)
  {
    LastTimeCycleMeasure = CurrentMicros;
  }
  FrequencyRaw = 10000000000 / PeriodAverage;
  if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra)
  {
    FrequencyRaw = 0;
    ZeroDebouncingExtra = 2000;
  }
  else
  {
    ZeroDebouncingExtra = 0;
  }
  FrequencyReal = FrequencyRaw / 10000;
  RPM = FrequencyRaw / PulsesPerRevolution * 60;
  RPM = RPM / 10000;
  //  kalrpm = kalman(RPM);
  //  return kalrpm;
  kalrpm = kalman(RPM);
  //  return kalrpm;
  int disrpm = kalrpm;
  int kmh = 2 * 3.6 * 3.1415926536 * 0.095 * disrpm / 60; //(2 * 3.1415926536 * 0.300803 * kalrpm * 60) / 1000;
  Serial1.print("rpm.val=");
  Serial1.print(disrpm);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.print("kmh.val=");
  Serial1.print(kmh);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  int value = map(disrpm, 0, 5000, 0, 255);
  String line1 = "add ";
  line1 += 5;
  line1 += ",";
  line1 += 1;
  line1 += ",";
  line1 += value;
  Serial1.print(line1);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  String line2 = "add ";
  line2 += 5;
  line2 += ",";
  line2 += 0;
  line2 += ",";
  line2 += setgraph;
  Serial1.print(line2);
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}
void motorOn() {
  digitalWrite(motor, LOW);
  // Serial.println("MOTOR ON");
}
void motorOff() {
  digitalWrite(motor, HIGH);
  // Serial.println("MOTOR OFF");
}
void setReverse() {
  digitalWrite(rev, LOW);
  // Serial.println("Motor siap mundur");
}
void setForward() {
  digitalWrite(rev, HIGH);
  // Serial.println("Motor siap maju");
}
void lowBrakeOn() {
  digitalWrite(lowBrake, LOW);
  // Serial.println("STOP, LOW Brake On");
}
void lowBrakeOff() {
  digitalWrite(lowBrake, HIGH);
  // Serial.println("STOP, Low Brake Off");
}
void gearOneOn() {
  digitalWrite(gearOne, LOW);
  digitalWrite(gearThree, HIGH);
  // Serial.println("Gear One On");
}
void gearTwoOn() {
  digitalWrite(gearOne, HIGH);
  digitalWrite(gearThree, HIGH);
  // Serial.println("Gear Two On");
}
void gearThreeOn() {
  digitalWrite(gearThree, LOW);
  digitalWrite(gearOne, HIGH);
  // Serial.println("Gear Three On");
}

double setpoint = 0;
void relay() {
  static bool aChangeConditionReceived = false;
  static bool bChangeConditionReceived = false;
  static bool cChangeConditionReceived = false;
  static bool dChangeConditionReceived = false;
  static bool eChangeConditionReceived = false;
  static bool fChangeConditionReceived = false;
  static bool gChangeConditionReceived = false;
  static bool hChangeConditionReceived = false;
  static bool iChangeConditionReceived = false;
  static bool lChangeConditionReceived = false;
  static bool xChangeConditionReceived = false;
  static bool zChangeConditionReceived = false;
  static bool uChangeConditionReceived = false;
  static bool vChangeConditionReceived = false;
  static bool AChangeConditionReceived = false;
  static bool BChangeConditionReceived = false;
  static bool CChangeConditionReceived = false;
  static bool DChangeConditionReceived = false;
  static bool EChangeConditionReceived = false;
  static bool FChangeConditionReceived = false;
  static bool GChangeConditionReceived = false;
  static bool HChangeConditionReceived = false;
  static bool IChangeConditionReceived = false;
  static bool QChangeConditionReceived = false;
  while (Serial1.available()) {
    char ChangeCondition = Serial1.read();
    if (ChangeCondition == 'a' && !aChangeConditionReceived) {
      aChangeConditionReceived = true;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'b' && !bChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = true;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'c' && !cChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = true;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'd' && !dChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = true;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'e' && !eChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = true;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'f' && !fChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = true;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'g' && !gChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = true;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'h' && !hChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = true;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'i' && !iChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = true;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'x' && !xChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = true;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'l' && !lChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      lChangeConditionReceived = true;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'z' && !zChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      lChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = true;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'v' && !vChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      lChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      uChangeConditionReceived = false;
      vChangeConditionReceived = true;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'u' && !uChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      lChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      vChangeConditionReceived = false;
      uChangeConditionReceived = true;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'A' && !AChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = true;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'B' && !BChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = true;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'C' && !CChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = true;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'D' && !DChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = true;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'E' && !EChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = true;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'F' && !FChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = true;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'G' && !GChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = true;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'H' && !HChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = true;
      IChangeConditionReceived = false;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'I' && !IChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = true;
      QChangeConditionReceived = false;
    }
    else if (ChangeCondition == 'Q' && !QChangeConditionReceived) {
      aChangeConditionReceived = false;
      bChangeConditionReceived = false;
      cChangeConditionReceived = false;
      dChangeConditionReceived = false;
      eChangeConditionReceived = false;
      fChangeConditionReceived = false;
      gChangeConditionReceived = false;
      hChangeConditionReceived = false;
      iChangeConditionReceived = false;
      xChangeConditionReceived = false;
      zChangeConditionReceived = false;
      lChangeConditionReceived = false;
      AChangeConditionReceived = false;
      BChangeConditionReceived = false;
      CChangeConditionReceived = false;
      DChangeConditionReceived = false;
      EChangeConditionReceived = false;
      FChangeConditionReceived = false;
      GChangeConditionReceived = false;
      HChangeConditionReceived = false;
      IChangeConditionReceived = false;
      QChangeConditionReceived = true;
    }
  }

  if (aChangeConditionReceived) {
    setForward();
  }
  else if (bChangeConditionReceived) {
    setReverse();
  }
  //  //Stop motor dikondisi maju/mundur
  else if (cChangeConditionReceived) {
    lowBrakeOn();
  }
  else if (dChangeConditionReceived) {
    lowBrakeOff();
  }
  else if (fChangeConditionReceived) {
    motorOff();
  }
  else if (eChangeConditionReceived) {
    motorOn();
  }
  else if (gChangeConditionReceived) {
    gearOneOn();
  }
  else if (hChangeConditionReceived) {
    gearTwoOn();
  }
  else if (iChangeConditionReceived) {
    gearThreeOn();
  }
  else if (lChangeConditionReceived) {
    Voltage_Awal();
  }
  else if (zChangeConditionReceived) {
    saveSOC();
  }
  else if (xChangeConditionReceived) {
    loadSOC();
  }
  else if (uChangeConditionReceived) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, 255);
    delay(225);
    analogWrite(ena, 0);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
  else if (vChangeConditionReceived) {
    analogWrite(ena, 255);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    delay(225);
    analogWrite(ena, 0);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
  else if (QChangeConditionReceived) {
    setpoint = 500;
    Serial1.print("in1.val=");
    Serial1.print("500");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (AChangeConditionReceived) {
    setpoint = 1000;
    Serial1.print("in1.val=");
    Serial1.print("1000");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (BChangeConditionReceived) {
    setpoint = 1500;
    Serial1.print("in1.val=");
    Serial1.print("1500");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  } else if (CChangeConditionReceived) {
    setpoint = 2000;
    Serial1.print("in1.val=");
    Serial1.print("2000");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (DChangeConditionReceived) {
    setpoint = 2500;
    // Serial.println("2500");
    Serial1.print("in1.val=");
    Serial1.print("2500");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (EChangeConditionReceived) {
    setpoint = 3000;
    Serial1.print("3000");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (FChangeConditionReceived) {
    setpoint = 3500;
    Serial1.print("in1.val=");
    Serial1.print("3500");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (GChangeConditionReceived) {
    setpoint = 4000;
    Serial1.print("in1.val=");
    Serial1.print("4000");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (HChangeConditionReceived) {
    setpoint = 4200;
    Serial1.print("in1.val=");
    Serial1.print("4200");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (IChangeConditionReceived) {
    setpoint = 0;
    Serial1.print("in1.val=");
    Serial1.print("0000");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
}


void setup() {
  Serial1.begin(9600);
  //while (!Serial) delay(100);
  MyPot.begin(CS_PIN);

  pinMode(rev, OUTPUT);
  pinMode(lowBrake, OUTPUT);
  pinMode(motor, OUTPUT);
  pinMode(gearOne, OUTPUT);
  pinMode(gearThree, OUTPUT);
  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(3), Pulse_Event, RISING);
}

void loop() {
  setgraph = map(setpoint, 0, 5000, 0, 255);
  unsigned long currentTime = millis();
  relay();
  rpmCounter();
  if (currentTime - prevTime >= 500) {
    double input = kalrpm;
    double output = pid(input);
    MyPot.setWiper(output);
    MyPot.setWiper(output);
    Voltage_Measurement();
    Current_Measurement();
    prevTime = currentTime;
  }
}

double pid(double input)
{
  double error = setpoint - input;
  proportional = error;
  derivative = (error - previous) ;

  double kp = 0.016857; //* k1; //0.00024923
  double ki = 0.0082878; //* k2; //0.0033105
  double kd = 0.0085712; //* k3; //4.6908e-06

  double p_value = proportional * kp;
  double i_value = integral * ki ;
  double d_value = derivative * kd ;
  double potValue = p_value + i_value + d_value;
  double newPot;
  double IAE;
  int pid = potValue;
  newPot = map(pid, -255, 255, 0, 255);

  if (setpoint == 0 && newPot <= 160) {
    error = 0;
    integral = 0;
  }
  else {
    integral += error;
    previous = error;
  }

  double ka = 0.98333;
  if (newPot > 255) {
    integral = integral - (ka * newPot);
    newPot = 255;
  }
  else if (newPot < 0) {
    integral = integral - (ka * newPot);
    newPot = 0;
  }
  Serial.print(kalrpm);
  Serial.print("\t");
  Serial.println(newPot);


  return newPot;
}
