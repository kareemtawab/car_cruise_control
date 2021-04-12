#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <TimeLib.h>
#include "Timer.h"
#include "Button2.h";
#include "ESPRotary.h";
#include <FastPID.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ElegantOTA.h>

#define AP_SSID "NissanTiida'07 CC"
#define GPSRXPin 0
#define GPSTXPin 2
#define GPSBaud 38400
#define TSmin 75
#define TSmax 130
#define TESTSPEED 100
#define ButtonCCholdtime 2000
#define BUZZER_PIN D8
#define RELAY_PIN D5
#define BRAKE_PIN A0
#define ANALOG_THRESHOLD 750
#define CLICKS_PER_STEP   4   // this number depends on your rotary encoder 
#define ROTARY_PIN1 D7 //D6
#define ROTARY_PIN2 D6 //D7
#define BUTTON_PIN  D0 //GPIO16

IPAddress local_IP(192, 168, 1, 2);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
byte systemstatus = 0;
uint8_t fix[8] = {0x0, 0x1, 0x3, 0x16, 0x1c, 0x8, 0x0};
uint8_t nofix[8] = {0x0, 0x1b, 0xe, 0x4, 0xe, 0x1b, 0x0};
uint8_t ccon[8]  = {0x00, 0x04, 0x0E, 0x1F, 0x0E, 0x04, 0x00, 0x00};
uint8_t ccoff[8]  = {0x00, 0x04, 0x0A, 0x11, 0x0A, 0x04, 0x00, 0x00};
bool enableCCflag;
bool systemtest = false;
long buzzerblib;
long buttonpressstarttime;
int currentspeed;
int targetspeed = 100;
int DACoutputpercent;
int dir;
float Kp = 2, Ki = 0.03, Kd = 1, Hz = 10;
int output_bits = 8;
bool output_signed = true;
String MAIN_page() {
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<title>NISSAN TIIDA '07 CRUISE CONTROL</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: table; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr += "p {font-size: 20px;color: #888;margin-bottom: 14px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";

  ptr += "<h1>NISSAN TIIDA '07 CRUISE CONTROL</h1>\n";
  ptr += "<h2>created 02/2021</h2>\n";
  ptr += "<h3>Go to 'http://192.168.1.2/update' to perform OTA update</h3>\n";

  ptr += "</body>\n";
  ptr += "</html>\n";
  return ptr;
}

ESP8266WebServer server(80);
TinyGPSPlus gps;
SoftwareSerial ss(GPSRXPin, GPSTXPin);
LiquidCrystal_I2C lcd(0x3F, 16, 2);  // I2c address, columns lcd module, rows lcd module
Adafruit_MCP4725 dac1;
Adafruit_MCP4725 dac2;
Timer t;
ESPRotary encoder = ESPRotary(ROTARY_PIN1, ROTARY_PIN2, CLICKS_PER_STEP);
Button2 button = Button2(BUTTON_PIN);
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

void encoderblib(ESPRotary& r) {
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerblib = millis();
}

void increaseTS(ESPRotary& r) {
  if (targetspeed > TSmax) {
    targetspeed = TSmax;
  }
  else if (systemstatus != 2) {
    targetspeed++;
  }
}

void decreaseTS(ESPRotary& r) {
  if (targetspeed < TSmin) {
    targetspeed = TSmin;
  }
  else if (systemstatus != 2) {
    targetspeed--;
  }
}

void enableCC(Button2& btn) {
  enableCCflag = 1;
}

void buttonblib(Button2& btn) {
  if (systemstatus == 2) {
    enableCCflag = 0;
  }
  digitalWrite(BUZZER_PIN, HIGH);
  buzzerblib = millis();
}


void checksystemstatus() {
  if (gps.time.age() < 1000 && gps.date.age() < 1000 && gps.speed.age() < 1000 && gps.sentencesWithFix() > 0 && enableCCflag == 0 && systemtest == false) {
    systemstatus = 1;
  }
  if (enableCCflag == 1 && button.wasPressedFor() > ButtonCCholdtime && systemstatus == 1 && systemtest == false) {
    systemstatus = 2;
  }
  if (systemtest) {
    systemstatus = 3;
  }
  if (systemstatus != 1 && systemstatus != 2 && systemstatus != 3) {
    systemstatus = 0;
  }
  if (analogRead(BRAKE_PIN) < ANALOG_THRESHOLD &&  systemstatus == 2) {
    enableCCflag = 0;
  }
}

void updatelcd() {
  switch (systemstatus) {
    case 0:
      lcd.setCursor(2, 0);
      lcd.write(3);
      lcd.setCursor(2, 1);
      lcd.write(3);
      lcd.setCursor(6, 0);
      lcd.write(3);
      lcd.setCursor(6, 1);
      lcd.write(3);
      lcd.setCursor(15, 1);
      lcd.write(1);

      lcd.setCursor(0, 0);
      lcd.print("CS");
      lcd.setCursor(3, 0);
      lcd.print("---");

      lcd.setCursor(0, 1);
      lcd.print("TS");
      lcd.setCursor(3, 1);
      if (targetspeed < 100 && targetspeed >= 10) {
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else if (targetspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else {
        lcd.print(int(targetspeed));
      }

      lcd.setCursor(8, 0);
      lcd.print("--");

      lcd.setCursor(8, 1);
      lcd.print("---");
      lcd.print(char(B11011111));

      if (timeStatus() == timeSet) {
        lcd.setCursor(11, 0);
        if (hourFormat12() < 10) {
          lcd.print('0');
          lcd.print(hourFormat12());
        }
        else {
          lcd.print(hourFormat12());
        }
        lcd.print(':');
        if (minute() < 10) {
          lcd.print('0');
          lcd.print(minute());
        }
        else {
          lcd.print(minute());
        }
      }
      else {
        lcd.setCursor(11, 0);
        lcd.print("--:--");
      }
      break;
    case 1:
      lcd.setCursor(2, 0);
      lcd.write(3);
      lcd.setCursor(2, 1);
      lcd.write(3);
      lcd.setCursor(6, 0);
      lcd.write(3);
      lcd.setCursor(6, 1);
      lcd.write(3);
      lcd.setCursor(15, 1);
      lcd.write(0);

      lcd.setCursor(0, 0);
      lcd.print("CS");
      lcd.setCursor(3, 0);
      if (currentspeed < 100 && currentspeed >= 10) {
        lcd.print('0');
        lcd.print(int(currentspeed));
      }
      else if (currentspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(currentspeed));
      }
      else {
        lcd.print(int(currentspeed));
      }

      lcd.setCursor(0, 1);
      lcd.print("TS");
      lcd.setCursor(3, 1);
      if (targetspeed < 100 && targetspeed >= 10) {
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else if (targetspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else {
        lcd.print(int(targetspeed));
      }

      lcd.setCursor(8, 0);
      lcd.print("--");

      lcd.setCursor(8, 1);
      if (dir < 100 && dir >= 10) {
        lcd.print('0');
        lcd.print(int(dir));
      }
      else if (dir < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(dir));
      }
      else {
        lcd.print(int(dir));
      }
      lcd.print(char(B11011111));

      if (timeStatus() == timeSet) {
        lcd.setCursor(11, 0);
        if (hourFormat12() < 10) {
          lcd.print('0');
          lcd.print(hourFormat12());
        }
        else {
          lcd.print(hourFormat12());
        }
        lcd.print(':');
        if (minute() < 10) {
          lcd.print('0');
          lcd.print(minute());
        }
        else {
          lcd.print(minute());
        }
      }
      break;

    case 2:
      if (second() % 2) {
        lcd.setCursor(2, 0);
        lcd.write(3);
        lcd.setCursor(6, 0);
        lcd.write(3);
        lcd.setCursor(2, 1);
        lcd.write(2);
        lcd.setCursor(6, 1);
        lcd.write(2);
      }
      else {
        lcd.setCursor(2, 0);
        lcd.write(2);
        lcd.setCursor(6, 0);
        lcd.write(2);
        lcd.setCursor(2, 1);
        lcd.write(3);
        lcd.setCursor(6, 1);
        lcd.write(3);
      }
      lcd.setCursor(15, 1);
      lcd.write(0);

      lcd.setCursor(0, 0);
      lcd.print("CS");
      lcd.setCursor(3, 0);
      if (currentspeed < 100 && currentspeed >= 10) {
        lcd.print('0');
        lcd.print(int(currentspeed));
      }
      else if (currentspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(currentspeed));
      }
      else {
        lcd.print(int(currentspeed));
      }

      lcd.setCursor(0, 1);
      lcd.print("TS");
      lcd.setCursor(3, 1);
      if (targetspeed < 100 && targetspeed >= 10) {
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else if (targetspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else {
        lcd.print(int(targetspeed));
      }

      lcd.setCursor(8, 0);
      if (DACoutputpercent < 10) {
        lcd.print('0');
        lcd.print(DACoutputpercent);
      }
      else {
        lcd.print(DACoutputpercent);
      }

      lcd.setCursor(8, 1);
      if (dir < 100 && dir >= 10) {
        lcd.print('0');
        lcd.print(int(dir));
      }
      else if (dir < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(dir));
      }
      else {
        lcd.print(int(dir));
      }
      lcd.print(char(B11011111));

      if (timeStatus() == timeSet) {
        lcd.setCursor(11, 0);
        if (hourFormat12() < 10) {
          lcd.print('0');
          lcd.print(hourFormat12());
        }
        else {
          lcd.print(hourFormat12());
        }
        lcd.print(':');
        if (minute() < 10) {
          lcd.print('0');
          lcd.print(minute());
        }
        else {
          lcd.print(minute());
        }
      }
      break;

    case 3:
      lcd.setCursor(2, 0);
      lcd.write(3);
      lcd.setCursor(2, 1);
      lcd.write(3);
      lcd.setCursor(6, 0);
      lcd.write(3);
      lcd.setCursor(6, 1);
      lcd.write(3);
      lcd.setCursor(15, 1);
      lcd.print("-");

      lcd.setCursor(0, 0);
      lcd.print("CS");
      lcd.setCursor(3, 0);
      if (currentspeed < 100 && currentspeed >= 10) {
        lcd.print('0');
        lcd.print(int(currentspeed));
      }
      else if (currentspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(currentspeed));
      }
      else {
        lcd.print(int(currentspeed));
      }

      lcd.setCursor(0, 1);
      lcd.print("TS");
      lcd.setCursor(3, 1);
      if (targetspeed < 100 && targetspeed >= 10) {
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else if (targetspeed < 10) {
        lcd.print('0');
        lcd.print('0');
        lcd.print(int(targetspeed));
      }
      else {
        lcd.print(int(targetspeed));
      }
      lcd.setCursor(12, 0);
      lcd.print("TEST");
      lcd.setCursor(8, 0);
      if (DACoutputpercent < 10) {
        lcd.print('0');
        lcd.print(DACoutputpercent);
      }
      else {
        lcd.print(DACoutputpercent);
      }
      break;
  }
  lcd.setCursor(13, 1);
  if (gps.satellites.value() < 10) {
    lcd.print('0');
    lcd.print(gps.satellites.value());
  }
  else {
    lcd.print(gps.satellites.value());
  }
}

void setup() {
  //Serial.begin(115200);
  lcd.backlight();
  lcd.init();
  lcd.createChar(0, fix);
  lcd.createChar(1, nofix);
  lcd.createChar(2, ccon);
  lcd.createChar(3, ccoff);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(50);
  digitalWrite(BUZZER_PIN, LOW);
  dac1.begin(0x60); // ADDR -> GND
  dac2.begin(0x61); // ADDR -> VCC
  encoder.setChangedHandler(encoderblib);
  encoder.setLeftRotationHandler(decreaseTS);
  encoder.setRightRotationHandler(increaseTS);
  button.setDebounceTime(40);
  //button.setClickHandler(disableCC);
  button.setLongClickHandler(enableCC);
  button.setChangedHandler(buttonblib);
  ss.begin(GPSBaud);
  myPID.setOutputRange(15, 99);
  t.every(250, updatelcd, 0);
  t.every(100, checksystemstatus, 0);
  lcd.setCursor(0, 0);
  lcd.print("NISSAN TIIDA '07");
  lcd.setCursor(1, 1);
  lcd.print("CRUISE CONTROL");
  delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Created 02/2021");
  lcd.setCursor(0, 1);
  lcd.print("Kareem A. Tawab");
  delay(2500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connect to AP");
  lcd.setCursor(0, 1);
  lcd.print("IP: 192.168.1.2");
  delay(2500);
  lcd.clear();
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID);
  server.on("/", []() {
    server.send(200, "text/html", MAIN_page()); //Send web page
  });
  ElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  digitalWrite(BUZZER_PIN, HIGH);
  delay(10);
  digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  server.handleClient();
  if (millis() - buzzerblib > 1) {
    digitalWrite(BUZZER_PIN, LOW);
    buzzerblib = millis();
  }
  encoder.loop();
  button.loop();
  t.update();
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }
  dir = constrain(gps.course.deg(), 0, 359);
  switch (systemstatus) {
    case 0:
      DACoutputpercent = 0;
      break;
    case 1:
      currentspeed = gps.speed.kmph();
      if (timeStatus() != timeSet) {
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
        adjustTime(7200);
      }
      DACoutputpercent = 0;
      digitalWrite(RELAY_PIN, HIGH);
      break;
    case 2:
      currentspeed = gps.speed.kmph();
      DACoutputpercent = myPID.step(targetspeed, currentspeed);
      if (DACoutputpercent < 0) {
        DACoutputpercent = 0;
      }
      dac1.setVoltage(DACoutputpercent / 100 * 4096, false);
      dac2.setVoltage(DACoutputpercent / 100 * 4096 / 2, false);
      digitalWrite(RELAY_PIN, LOW);
      break;
    case 3:
      currentspeed = TESTSPEED;
      DACoutputpercent = myPID.step(targetspeed, currentspeed);
      if (DACoutputpercent < 0) {
        DACoutputpercent = 0;
      }
      break;
  }
}
