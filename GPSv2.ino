#define I2C_ADDRESS 0x3C
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <MechaQMC5883.h>


#define ARDUINO_GPS_RX 3
#define ARDUINO_GPS_TX 4
#define GPS_BAUD 9600
#define gpsPort ssGPS
#define SerialMonitor Serial
#define I2C_ADDRESS 0x3C
#define Pin 6

TinyGPSPlus tinyGPS;
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX);
OneWire ourWire(Pin);
DallasTemperature sensors(&ourWire);
MechaQMC5883 qmc;


int mX0, mX1, mX_out;
int mY0, mY1, mY_out;
int mZ0, mZ1, mZ_out;
float heading, headingDegrees, headingFiltered, declination;
float Xm, Ym, Zm;
int led1 = 9;
int led2 = 10;
int led3 = 11;
int led4 = 12;
int led1s = 0;
int led2s = 0;
int led3s = 0;
int led4s = 0;
int TankValue0;
int TankValue1;
int calrpm = 0;
int analogInput = 2;
float vout = 0.0;
float vin = 0.0;
float R1 = 100000.0; // resistance of R1 (100K) -see text!
float R2 = 10000.0; // resistance of R2 (10K) - see text!
int value = 0;
int RelayState0 = 0;
int RelayState1 = 0;
int RelayState2 = 0;
int RelayState3 = 0;
int RPM = 0;
int input = 13;
int high_time;
int low_time;
float time_period;
float frequency;
String Direccion = "";
float tempC;
int reading;
int tempPin = 3;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int minVal = 265;
int maxVal = 402;
double xg;
double yg;
double zg;

SSD1306AsciiAvrI2c oled;

void setup()
{
  SerialMonitor.begin(9600);
  gpsPort.begin(GPS_BAUD);
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(System5x7);
  oled.clear();
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  digitalWrite(led3, HIGH);
  digitalWrite(led4, HIGH);
  analogReference(INTERNAL);
  pinMode(input, INPUT);
  sensors.begin();
  qmc.init();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

}

void loop()
{
  //***************************Lectura Gyro*********************************

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  xg = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  yg = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  zg = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);


  //***************************fin Lectura Gyro*********************************

  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  smartDelay(1000);
  oled.setCursor(0, 0);

  //***************************Sensores de temperatura*********************************
  reading = analogRead(tempPin);
  tempC = reading / 9.31;

  sensors.requestTemperatures();

  //***************************Fin Sensores de temperatura*********************************
  //***************************Calculos de Heading*********************************
  int x, y, z;
  qmc.read(&x, &y, &z);
  //---- X-Axis
  Xm = x * 0.00092;
  //---- Y-Axis
  Ym = y * 0.00092;
  //---- Z-Axis
  Zm = z * 0.00092;

  heading = atan2(Ym, Xm);
  declination = 0.155;
  heading += declination;
  if (heading < 0) heading += 2 * PI;
  if (heading > 2 * PI)heading -= 2 * PI;
  headingDegrees = heading * 180 / PI;
  headingFiltered = headingFiltered * 0.85 + headingDegrees * 0.15;

  if (headingFiltered < 45 ) {
    Direccion = "N";
  }
  if (headingFiltered > 45 && headingFiltered < 90) {
    Direccion = "NE";
  }
  if (headingFiltered > 90 && headingFiltered < 135) {
    Direccion = "E";
  }
  if (headingFiltered > 135 && headingFiltered < 180) {
    Direccion = "SE";
  }
  if (headingFiltered > 180 && headingFiltered < 225) {
    Direccion = "S";
  }
  if (headingFiltered > 225 && headingFiltered < 270) {
    Direccion = "SW";
  }
  if (headingFiltered > 270 && headingFiltered < 315) {
    Direccion = "W";
  }
  if (headingFiltered > 315 && headingFiltered < 360) {
    Direccion = "NW";
  }

  //***************************fin de Calculos de Heading*********************************
  //***************************Rutina de Relay*********************************

  RelayState0 = digitalRead(led1);
  RelayState1 = digitalRead(led2);
  RelayState2 = digitalRead(led3);
  RelayState3 = digitalRead(led4);

Relay:
  if (Serial.available() > 0)
  { //cierre del segundo if
    char datos_serial = Serial.read();

    if (datos_serial == 'A')
    {
      if (RelayState0 == HIGH) {
        Serial.println("enciende 1");
        digitalWrite(led1, LOW);
        delay(2000);
        goto Relay;
      }
      if (RelayState0 == LOW) {
        digitalWrite(led1, HIGH);
        Serial.println("Apaga 1");
        delay(2000);
        goto Relay;
      }
    }
    if (datos_serial == 'B')
    {
      if (RelayState1 == HIGH) {
        Serial.println("enciende 2");
        digitalWrite(led2, LOW);
        delay(2000);
        goto Relay;
      }
      if (RelayState1 == LOW) {
        digitalWrite(led2, HIGH);
        Serial.println("Apaga 2");
        delay(2000);
        goto Relay;
      }
    }
    if (datos_serial == 'C')
    {
      if (RelayState2 == HIGH) {
        Serial.println("enciende 3");
        digitalWrite(led3, LOW);
        delay(2000);
        goto Relay;
      }
      if (RelayState2 == LOW) {
        digitalWrite(led3, HIGH);
        Serial.println("Apaga 3");
        delay(2000);
        goto Relay;
      }
    }
    if (datos_serial == 'D')
    {
      if (RelayState3 == HIGH) {
        Serial.println("enciende 4");
        digitalWrite(led4, LOW);
        delay(2000);
        goto Relay;
      }
      if (RelayState3 == LOW) {
        digitalWrite(led4, HIGH);
        Serial.println("Apaga 4");
        delay(2000);
        goto Relay;
      }
    }
    if (datos_serial == 'E')
    {
      digitalWrite(led1, HIGH);
      digitalWrite(led2, HIGH);
      digitalWrite(led3, HIGH);
      digitalWrite(led4, HIGH);
      Serial.println("Apaga todos");
    }
  }

  led1s = digitalRead(led1);
  led2s = digitalRead(led2);
  led3s = digitalRead(led3);
  led4s = digitalRead(led4);

  //***************************Fin de Rutina de Relay*********************************
  //***************************Rutina de tanque*********************************

  int sensorTankValue0 = analogRead(A0);
  int sensorTankValue1 = analogRead(A1);
  TankValue0 = map(sensorTankValue0, 295, 785, 0, 100);
  TankValue1 = map(sensorTankValue1, 295, 785, 0, 100);
  if (TankValue0 < 0) {
    TankValue0 = 0;
  }
  if (TankValue1 < 0) {
    TankValue1 = 0;
  }
  if (TankValue0 > 100) {
    TankValue0 = 100;
  }
  if (TankValue1 > 100) {
    TankValue1 = 100;
  }
  value = analogRead(analogInput);
  vout = (value * 5.0) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));
  float Vscalado;
  Vscalado = map(vin, 0, 500, 0, 1200);
  if (vin < 0.09) {
    Vscalado = 0.0; //statement to quash undesired reading !
  }

  //***************************Fin de Rutina de tanque*********************************
  //***************************Rutina de Display*********************************

  oled.print("Lat: ");
  oled.println(tinyGPS.location.lat(), 6);
  oled.print("Long: ");
  oled.println(tinyGPS.location.lng(), 6);
  oled.print("Date: ");
  printDate();
  oled.print("Time: ");
  printTime();
  oled.print(sensors.getTempCByIndex(0));
  oled.println(" C");
  oled.print("Direccion:");
  oled.print(Direccion);
  oled.println(" ");
  oled.print("T0=");
  oled.print(TankValue0);
  oled.print(" ");
  oled.print("T1=");
  oled.println(TankValue1);
  oled.print("V=");
  oled.print(Vscalado / 10);
  oled.println(" ");

  //***************************Fin de Rutina de Display*********************************
  //***************************Inicio de Transmision************************************

  printGPSInfo();
  SerialMonitor.print(Direccion);
  SerialMonitor.print(",");
  SerialMonitor.print(headingFiltered);
  SerialMonitor.print(",");
  SerialMonitor.print(sensors.getTempCByIndex(0));
  SerialMonitor.print(",");
  SerialMonitor.print(tempC);
  SerialMonitor.print(",");
  SerialMonitor.print(analogRead(A0));
  SerialMonitor.print(",");
  SerialMonitor.print(sensorTankValue1);
  SerialMonitor.print(",");
  SerialMonitor.print(Vscalado);
  SerialMonitor.print(",");
  SerialMonitor.print(!led1s);
  SerialMonitor.print(",");
  SerialMonitor.print(!led2s);
  SerialMonitor.print(",");
  SerialMonitor.print(!led3s);
  SerialMonitor.print(",");
  SerialMonitor.print(!led4s);
  SerialMonitor.print(",");
  SerialMonitor.print(frequency);
  SerialMonitor.print(",");
  SerialMonitor.println(100 - xg);

  //***************************Fin de Transmision*********************************
  //***************************Lectura rpm*********************************

  high_time = pulseIn(13, HIGH);
  low_time = pulseIn(13, LOW);
  time_period = low_time;
  time_period = time_period / 1000;
  frequency = 1000 / time_period;
  if (frequency < 0) {
    frequency = frequency * -1;
  }
  calrpm = calrpm + 1;
  //delay(500);

  //***************************Fin Lectura rpm*********************************

}

void printGPSInfo()
{
  SerialMonitor.print(tinyGPS.location.lat(), 6);
  SerialMonitor.print(",");
  SerialMonitor.print(tinyGPS.location.lng(), 6);
  SerialMonitor.print(",");
}


static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {

    while (gpsPort.available())
      tinyGPS.encode(gpsPort.read());
  } while (millis() - start < ms);
}


void printDate()
{
  oled.print(tinyGPS.date.day());
  oled.print("/");
  oled.print(tinyGPS.date.month());
  oled.print("/");
  oled.println(tinyGPS.date.year());
}

void printTime()
{
  oled.print(tinyGPS.time.hour());
  oled.print(":");
  if (tinyGPS.time.minute() < 10) oled.print('0');
  oled.print(tinyGPS.time.minute());
  oled.print(":");
  if (tinyGPS.time.second() < 10) oled.print('0');
  oled.println(tinyGPS.time.second());
}
