#include <SPI.h>
#include <avr/pgmspace.h>

// Variables
byte initComplete = 0;
byte Motion = 0;
byte xH, xL, yH, yL;
int xy1dat[2];
double dP, dR, dY;
double vel_gain = 10.0; // Gain from ball rotation to analog out

const int ncs1 = 0;
const int pVelPin = 3;
const int rVelPin = 4;
const int yVelPin = 5;
const int valve1Pin = 14;
const int valve2Pin = 15;
const int lick1Pin = 16;
const int lick2Pin = 17;

unsigned long valve1Start = 0, valve2Start = 0;
unsigned int valve1State = 0, valve2State = 0;
unsigned long valve1Dur = 0, valve2Dur = 0;
unsigned long lickCount1 = 0, lickCount2 = 0;
unsigned long lastLick1 = 0, lastLick2 = 0;
unsigned long lastMsgTime = 0, absTime = micros(), dt = micros();

// Registers
#define REG_Motion 0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06
#define REG_Power_Up_Reset 0x3a

void setup() {
  Serial.begin(57600);
  Serial.println('S');

  analogWriteFrequency(pVelPin, 11500);
  analogWriteFrequency(rVelPin, 11500);
  analogWriteFrequency(yVelPin, 11500);
  analogWriteResolution(12);

  pinMode(ncs1, OUTPUT);
  pinMode(valve1Pin, OUTPUT);
  pinMode(valve2Pin, OUTPUT);
  pinMode(lick1Pin, INPUT);
  pinMode(lick2Pin, INPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  delay(1000);
  performStartup1();
  delay(10);
  dispRegisters1();
  delay(1500);
  initComplete = 9;
}

void adns1_com_begin() { digitalWrite(ncs1, LOW); }
void adns1_com_end() { digitalWrite(ncs1, HIGH); }

byte adns1_read_reg(byte reg_addr) {
  adns1_com_begin();
  SPI.transfer(reg_addr & 0x7F);
  delayMicroseconds(100);
  byte data = SPI.transfer(0);
  delayMicroseconds(1);
  adns1_com_end();
  delayMicroseconds(19);
  return data;
}

void adns1_write_reg(byte reg_addr, byte data) {
  adns1_com_begin();
  SPI.transfer(reg_addr | 0x80);
  SPI.transfer(data);
  delayMicroseconds(20);
  adns1_com_end();
  delayMicroseconds(100);
}

void performStartup1() {
  adns1_com_end();
  adns1_com_begin();
  adns1_com_end();
  adns1_write_reg(REG_Power_Up_Reset, 0x5A);
  delay(50);
  adns1_read_reg(REG_Motion);
  adns1_read_reg(REG_Delta_X_L);
  adns1_read_reg(REG_Delta_X_H);
  adns1_read_reg(REG_Delta_Y_L);
  adns1_read_reg(REG_Delta_Y_H);
  Serial.println("Optical Sensor 1 Initialized");
}

void dispRegisters1() {
  int oreg[4] = {0x00, 0x3F, 0x2A, 0x0F};
  const char* oregname[] = {"Product_ID", "Inverse_Product_ID", "SROM_Version", "CPI"};
  byte regres;

  digitalWrite(ncs1, LOW);
  for (int rctr = 0; rctr < 4; rctr++) {
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr], HEX);
    regres = SPI.transfer(0);
    Serial.println(regres, BIN);
    Serial.println(regres, HEX);
    delay(1);
  }
  digitalWrite(ncs1, HIGH);
}

void readXY1() {  
  digitalWrite(ncs1, LOW);
  adns1_write_reg(REG_Motion, 0x01);

  Motion = (adns1_read_reg(REG_Motion) & (1 << (8-1))) != 0;
  xL = adns1_read_reg(REG_Delta_X_L);
  xH = adns1_read_reg(REG_Delta_X_H);
  yL = adns1_read_reg(REG_Delta_Y_L);
  yH = adns1_read_reg(REG_Delta_Y_H);
  
  xy1dat[0] = (int16_t)((xH << 8) | xL);
  xy1dat[1] = (int16_t)((yH << 8) | yL);

  digitalWrite(ncs1, HIGH);
}

void interpretCommand(String message) {
  message.trim(); 
  if (message.length() != 6) {
    Serial.println("#"); 
    return;
  }
  
  String cmd1 = message.substring(0, 3);
  String cmd2 = message.substring(3);
  
  if (cmd1.toInt() > 0) {
    valve1Start = millis();
    valve1State = 1;
    valve1Dur = cmd1.toInt();
    digitalWrite(valve1Pin, HIGH);
  }

  if (cmd2.toInt() > 0) {
    valve2Start = millis();
    valve2State = 1;
    valve2Dur = cmd2.toInt();
    digitalWrite(valve2Pin, HIGH);
  }
}

void loop() {
  dt = micros() - absTime;
  absTime = micros();

  readXY1(); 

  dP = xy1dat[0];
  dR = xy1dat[1];
  dY = 0;

  analogWrite(pVelPin, dP * vel_gain + 2048);
  analogWrite(rVelPin, dR * vel_gain + 2048);
  analogWrite(yVelPin, dY * vel_gain + 2048);

  // Update lick detection
  unsigned long read1 = digitalRead(lick1Pin);
  unsigned long read2 = digitalRead(lick2Pin);
  if (lastLick1 != read1) lickCount1++;
  if (lastLick2 != read2) lickCount2++;
  lastLick1 = read1;
  lastLick2 = read2;

  // Update valve states
  if (valve1State == 1 && (millis() - valve1Start > valve1Dur)) {
    valve1State = 0;
    digitalWrite(valve1Pin, LOW);
    valve1Dur = 0;
  }
  if (valve2State == 1 && (millis() - valve2Start > valve2Dur)) {
    valve2State = 0;
    digitalWrite(valve2Pin, LOW);
    valve2Dur = 0;
  }

  // Read and process serial input
  static String usbMessage = "";
  while (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == '\n') {
      interpretCommand(usbMessage);
      usbMessage = "";
    } else {
      usbMessage += inByte;
    }
  }

  // Construct and send data string
  String dataString = "dp," + String(dP, 3) + 
                      ",dr," + String(dR, 3) + 
                      ",dy," + String(dY, 3) + 
                      ",l1," + String(lickCount1) + 
                      ",l2," + String(lickCount2) + 
                      ",v1," + String(valve1State) + 
                      ",v2," + String(valve2State) + 
                      ",dta," + String(dt) + 
                      ",dtmsg," + String(millis() - lastMsgTime);
  
  Serial.println(dataString);
  lastMsgTime = millis();
  lickCount1 = 0;
  lickCount2 = 0;

  delayMicroseconds(10);
}
