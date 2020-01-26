
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#include "SIM900.h"
#include "sms.h"
SMSGSM sms;


LiquidCrystal_I2C lcd(0x3F,16,2); 

static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 9600;

SoftwareSerial ss(RXPin, TXPin); //RX=pin 10, TX=pin 11
TinyGPSPlus gps;

#define smokePin A0
#define alchoPin A4
#define flamePin A2
#define CrushPin1 A3
#define CrushPin2 A5
#define tempPin A1

//smoke,temp,crush



#define LMF 8
#define LMB 9
#define RMF 12
#define RMB 13


boolean started = false;
char sms_position;
char phone_number[20];
char sms_text[100];
int i;



char phone_no[] = "01533575674";    //put your won Phone No
long gyroX, gyroY, gyroZ;
int rotX, rotY, rotZ;
int cheack = 0, flameRead;
int xfixd = -1, yfixd = -1;

char Z;

int crushData, tempData;




void setup() {

  ss.begin(GPSBaud);
  lcd.begin(16, 2);
  setupMPU();
  digitalWrite(CrushPin1, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Safe Driving &");
  lcd.setCursor(0, 1);
  lcd.print("Vehicle Tracker");
  delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Supervised by :");
  lcd.setCursor(0, 1);
  lcd.print("Abedul Hadi");
  delay(4000);
  lcd.clear();

  pinMode(smokePin, INPUT);
  pinMode(alchoPin, INPUT);
  pinMode(flamePin, INPUT);
  pinMode(CrushPin2, INPUT);
  pinMode(tempPin, INPUT);

  //  serial_connection.begin(9600);

  if (gsm.begin(4800)) {

    Serial.print("ok");
    started = true;
  }
  if (started) {


    lcd.setCursor(0, 0);
    lcd.print("System Ready");
    delay(2000);
    lcd.clear();

  }
  Serial.begin(9600);
  //Wire.begin();
  //  lcd.print();

}



void setupMPU() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
  processGyroData();


}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void printData() {
  //    Serial.print("Gyro (deg)");
  //    Serial.print(" X=");
  //    Serial.print(rotX);
  //    Serial.print(" Y=");
  //    Serial.print(rotY);
  //    Serial.print(" Z=");
  //    Serial.print(rotZ);

  int xdata = xfixd - rotX;
  int ydata = yfixd - rotY;


  if (xdata >= 12   ||  xdata <= -12) {
    Serial.begin(9600);
    delay(1000);
    Serial.println("AT+CMGF=1");
    delay(2000);
    Serial.print("AT+CMGS=\"");
    Serial.print(phone_no);
    Serial.write(0x22);
    Serial.write(0x0D);  // hex equivalent of Carraige return
    Serial.write(0x0A);  // hex equivalent of newline
    delay(2000);
    Serial.print("There is an accdient !!!!! GSM&Gyro Test!!!");
    delay(500);
    Serial.println (char(26));//the ASCII code of the ctrl+z is 26


  }
  if (ydata >= 12  || ydata <= -12) {

    Serial.begin(9600);
    delay(1000);
    Serial.println("AT+CMGF=1");
    delay(2000);
    Serial.print("AT+CMGS=\"");
    Serial.print(phone_no);
    Serial.write(0x22);
    Serial.write(0x0D);  // hex equivalent of Carraige return
    Serial.write(0x0A);  // hex equivalent of newline
    delay(2000);
    Serial.print("There is an accdient !!!!! GSM&Gyro Test!!!");
    delay(500);
    Serial.println (char(26));//the ASCII code of the ctrl+z is 26

  }

  Serial.println();

}

void go() {

  digitalWrite(LMF, 1);
  digitalWrite(LMB, 0);
  digitalWrite(RMF, 1);
  digitalWrite(RMB, 0);

}

void back() {

  digitalWrite(LMF, 0);
  digitalWrite(LMB, 1);

  digitalWrite(RMF, 0);
  digitalWrite(RMB, 1);
}

void left() {

  digitalWrite(LMF, 1);
  digitalWrite(LMB, 0);

  digitalWrite(RMF, 0);
  digitalWrite(RMB, 0);

}

void right() {

  digitalWrite(LMF, 0);
  digitalWrite(LMB, 0);

  digitalWrite(RMF, 1);
  digitalWrite(RMB, 0);

}

void st() {

  digitalWrite(LMF, 0);
  digitalWrite(LMB, 0);

  digitalWrite(RMF, 0);
  digitalWrite(RMB, 0);
}

void carRun() {

  if (Serial.available() > 0) {
    Z = Serial.read();
  }

  switch (Z) {

    case 'F':
      go();
      break;

    case 'B':
      back();
      break;

    case 'L':
      left();
      break;

    case 'R':
      right();
      break;

    case 'S':
      st();
      break;


    default:
      st();
      break;

  }
}

void temp() {
  int temp;
  tempData = analogRead(tempPin);
  temp = tempData * 0.48828125;
  if (temp > 60) {
    lcd.setCursor(0, 0);
    lcd.print("Hitted!!");
    Serial.print("Hitted");
    sms.SendSMS(phone_no, "Engine Over Hited!!!  Location:: 23.7945196,90.4244944");


    // gpsValu();
  } else {
    lcd.setCursor(0, 0);
    lcd.print("        ");
  }

}

void  crush() {
  crushData = digitalRead(CrushPin2);

  if (crushData == 0) {
    lcd.setCursor(8, 0);
    lcd.print("Crushed!");
    sms.SendSMS(phone_no, "Your Car Crushed!!  Location:: 23.7945196,90.4244944");

    //gpsValu();
  } else {
    lcd.setCursor(8, 0);
    lcd.print("        ");
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}



void gpsValu() {

  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;



  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);


  sms.SendSMS(phone_no, "23.7945196,90.4244944");



  unsigned long distanceKmToLondon = (unsigned long)TinyGPSPlus::distanceBetween(
                                       gps.location.lat(),
                                       gps.location.lng(),
                                       LONDON_LAT,
                                       LONDON_LON) / 1000;






  Serial.println();

  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

}

void Flam() {
  flameRead = analogRead(flamePin);
  if (flameRead < 800) {
    lcd.setCursor(0, 1);
    lcd.print("Fire!!");
    sms.SendSMS(phone_no, "Fire On Your Car  Location:: 23.7945196,90.4244944");
    //gpsValu();
  } else {
    lcd.setCursor(0, 1);
    lcd.print("      ");
  }
}

void smokeSensor() {

  int smokeRead = analogRead(smokePin);
  if (smokeRead > 180) {
    lcd.setCursor(8, 1);
    lcd.print("Smoke!!");
    sms.SendSMS(phone_no, "Smoke On Your Car!!  Location:: 23.7945196,90.4244944");

    //gpsValu();
    Serial.println("Smoke!!!!!");
  } else {
    lcd.setCursor(8, 1);
    lcd.print("       ");
  }
}


void alcholRead() {
  int alchoRead = analogRead(alchoPin);
  if (alchoPin > 180) {
    Serial.println("alchole!!!!!");
    sms.SendSMS(phone_no, "Your Driver Drinks Alchole!! Location:: 23.7945196,90.4244944");
    sms.SendSMS(phone_no, "23.7945196");
    sms.SendSMS(phone_no, "90.4244944");
    //gpsValu();

  }

}




void loop() {

  carRun();
  recordGyroRegisters();
  printData();
  smokeSensor();
  alcholRead();
  Flam();
  crush();
  temp();
  delay(100);
}
