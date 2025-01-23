#include <SPI.h>
#include <avr/pgmspace.h>

// const unsigned short firmware_length = 4094;

// Variables from the second code
byte initComplete = 0;
byte Motion = 0;
byte xH;
byte xL;
byte yH;
byte yL;
int xy1dat[2];
int xy2dat[2];
double dP;
double dR;
double dY;
double vel_gain = 10.0; // gain from ball rotation to analog out

const int ncs1 = 0;
const int ncs2 = 1;
const int pVelPin = 3;
const int rVelPin = 4;
const int yVelPin = 5;
const int valve1Pin = 14;
const int valve2Pin = 15;
const int lick1Pin = 16;
const int lick2Pin = 17;

const double px1 = 0.8151;
const double rx1 = 0.1849;
const double yx1 = -0.5959;
const double py1 = 0.2414;
const double ry1 = -0.2414;
const double yy1 = -0.7779;
const double px2 = -0.1849;
const double rx2 = -0.8151;
const double yx2 = 0.5959;
const double py2 = -0.2414;
const double ry2 = 0.2414;
const double yy2 = -0.7779;

// Variables for keeping track of valve start
unsigned long valve1Start = 0;
unsigned long valve2Start = 0;

unsigned int valve1State = 0;
unsigned int valve2State = 0;

unsigned long valve1Dur = 0; // Duration of valve 1 open in millis
unsigned long valve2Dur = 0; // Duration of valve 2 open in millis

unsigned long lickCount1 = 0;
unsigned long lickCount2 = 0;

unsigned long lastLick1 = 0;
unsigned long lastLick2 = 0;

unsigned long lastMsgTime = 0;
unsigned long absTime = micros();
unsigned long dt = micros();

// Registers from the second code
#define REG_Product_ID 0x00
#define REG_Revision_ID 0x01
#define REG_Motion 0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06
#define REG_SQUAL 0x07
#define REG_Pixel_Sum 0x08
#define REG_Maximum_Pixel 0x09
#define REG_Minimum_Pixel 0x0a
#define REG_Shutter_Lower 0x0b
#define REG_Shutter_Upper 0x0c
#define REG_Frame_Period_Lower 0x0d
#define REG_Frame_Period_Upper 0x0e
#define REG_Configuration_I 0x0f
#define REG_Configuration_II 0x10
#define REG_Frame_Capture 0x12
#define REG_SROM_Enable 0x13
#define REG_Run_Downshift 0x14
#define REG_Rest1_Rate 0x15
#define REG_Rest1_Downshift 0x16
#define REG_Rest2_Rate 0x17
#define REG_Rest2_Downshift 0x18
#define REG_Rest3_Rate 0x19
#define REG_Frame_Period_Max_Bound_Lower 0x1a
#define REG_Frame_Period_Max_Bound_Upper 0x1b
#define REG_Frame_Period_Min_Bound_Lower 0x1c
#define REG_Frame_Period_Min_Bound_Upper 0x1d
#define REG_Shutter_Max_Bound_Lower 0x1e
#define REG_Shutter_Max_Bound_Upper 0x1f
#define REG_LASER_CTRL0 0x20
#define REG_Observation 0x24
#define REG_Data_Out_Lower 0x25
#define REG_Data_Out_Upper 0x26
#define REG_SROM_ID 0x2a
#define REG_Lift_Detection_Thr 0x2e
#define REG_Configuration_V 0x2f
#define REG_Configuration_IV 0x39
#define REG_Power_Up_Reset 0x3a
#define REG_Shutdown 0x3b
#define REG_Inverse_Product_ID 0x3f
#define REG_Motion_Burst 0x50
#define REG_SROM_Load_Burst 0x62
#define REG_Pixel_Burst 0x64

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

void setup() {
  // Setup from the second code
  Serial.begin(57600);
  Serial.println('S');

  analogWriteFrequency(pVelPin, 11500);
  analogWriteFrequency(rVelPin, 11500);
  analogWriteFrequency(yVelPin, 11500);
  analogWriteResolution(12);
  pinMode(ncs1, OUTPUT);
  pinMode(ncs2, OUTPUT);

  pinMode(valve1Pin, OUTPUT);
  pinMode(valve2Pin, OUTPUT);
  pinMode(lick1Pin, INPUT);
  pinMode(lick2Pin, INPUT);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);

  // Resulution is set in performStartup1 > adns1_upload_firmware
  delay(1000);
  performStartup1();

  delay(10);
  performStartup2();

  delay(1500);
  dispRegisters1();
  delay(1500);
  dispRegisters2();
  delay(1500);
  initComplete = 9;
}

// Functions from the second code
void adns1_com_begin() {
  digitalWrite(ncs1, LOW);
}

void adns2_com_begin() {
  digitalWrite(ncs2, LOW);
}

void adns1_com_end() {
  digitalWrite(ncs1, HIGH);
}

void adns2_com_end() {
  digitalWrite(ncs2, HIGH);
}

byte adns1_read_reg(byte reg_addr) {
  adns1_com_begin();
  SPI.transfer(reg_addr & 0x7f);
  delayMicroseconds(100);
  byte data = SPI.transfer(0);
  delayMicroseconds(1);
  adns1_com_end();
  delayMicroseconds(19);
  return data;
}

byte adns2_read_reg(byte reg_addr) {
  adns2_com_begin();
  SPI.transfer(reg_addr & 0x7f);
  delayMicroseconds(100);
  byte data = SPI.transfer(0);
  delayMicroseconds(1);
  adns2_com_end();
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

void adns2_write_reg(byte reg_addr, byte data) {
  adns2_com_begin();
  SPI.transfer(reg_addr | 0x80);
  SPI.transfer(data);
  delayMicroseconds(20);
  adns2_com_end();
  delayMicroseconds(100);
}

void adns1_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 1...");

  //Write 0 to Rest_En bit of REG_Configuration_II register to disable Rest mode.
  adns1_write_reg(REG_Configuration_II, 0x20);
  
  // write 0x1d in REG_SROM_Enable reg for initializing
  adns1_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to REG_SROM_Enable to start SROM download
  adns1_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns1_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the REG_SROM_ID register to verify the ID before any other register reads or writes.
  adns1_read_reg(REG_SROM_ID);

  //Write 0x00 to REG_Configuration_II register for wired mouse or 0x20 for wireless mouse design.
  adns1_write_reg(REG_Configuration_II, 0x00);

  // set initial CPI resolution
  adns1_write_reg(REG_Configuration_I, 0x77); // Max resolution at 12000 cpi
  delay(10);
  
  adns1_com_end();										  
  }
  
  void adns2_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 2...");

  //Write 0 to Rest_En bit of REG_Configuration_II register to disable Rest mode.
  adns2_write_reg(REG_Configuration_II, 0x20);
  
  // write 0x1d in REG_SROM_Enable reg for initializing
  adns2_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to REG_SROM_Enable to start SROM download
  adns2_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns2_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the REG_SROM_ID register to verify the ID before any other register reads or writes.
  adns2_read_reg(REG_SROM_ID);

  //Write 0x00 to REG_Configuration_II register for wired mouse or 0x20 for wireless mouse design.
  adns2_write_reg(REG_Configuration_II, 0x00);

  // set initial CPI resolution
  adns2_write_reg(REG_Configuration_I, 0x77); // Max resolution at 12000 cpi
  delay(1500); 								
  
  adns2_com_end();				  
  }

void performStartup1(void){
  adns1_com_end(); // ensure that the serial port is reset
  adns1_com_begin(); // ensure that the serial port is reset
  adns1_com_end(); // ensure that the serial port is reset
  adns1_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns1_read_reg(Motion);
  adns1_read_reg(REG_Delta_X_L);
  adns1_read_reg(REG_Delta_X_H);
  adns1_read_reg(REG_Delta_Y_L);
  adns1_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns1_upload_firmware();
  delay(10);
  Serial.println("Optical Chip 1 Initialized");
  }
  
  void performStartup2(void){
  adns2_com_end(); // ensure that the serial port is reset
  adns2_com_begin(); // ensure that the serial port is reset
  adns2_com_end(); // ensure that the serial port is reset
  adns2_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns2_read_reg(Motion);
  adns2_read_reg(REG_Delta_X_L);
  adns2_read_reg(REG_Delta_X_H);
  adns2_read_reg(REG_Delta_Y_L);
  adns2_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns2_upload_firmware();
  delay(10);
  Serial.println("Optical Chip 2 Initialized");
  }

void dispRegisters1() {
  int oreg[7] = {
    0x00, 0x3F, 0x2A, 0x0F
  };
  const char* oregname[] = {
    "Product_ID", "Inverse_Product_ID", "SROM_Version", "CPI"
  };
  byte regres;

  digitalWrite(ncs1, LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs1,HIGH);
}

void dispRegisters2() {
  int oreg[7] = {
    0x00, 0x3F, 0x2A, 0x0F
  };
  const char* oregname[] = {
    "Product_ID2", "Inverse_Product_ID2", "SROM_Version2", "CPI2"
  };
  byte regres;

  digitalWrite(ncs2, LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(ncs2,HIGH);
}

// void UpdatePointer1(void) {
//   if (initComplete == 9) {
//     digitalWrite(ncs1, LOW);
//     adns1_write_reg(REG_Motion, 0x01);
//     adns1_read_reg(REG_Motion);

//     xy1dat[0] = (int)adns1_read_reg(REG_Delta_X_L);
//     xy1dat[1] = (int)adns1_read_reg(REG_Delta_Y_L);

//     digitalWrite(ncs1, HIGH);
//   }
// }

// void UpdatePointer2(void) {
//   if (initComplete == 9) {
//     digitalWrite(ncs2, LOW);
//     adns2_write_reg(REG_Motion, 0x01);
//     adns2_read_reg(REG_Motion);

//     xy2dat[0] = (int)adns2_read_reg(REG_Delta_X_L);
//     xy2dat[1] = (int)adns2_read_reg(REG_Delta_Y_L);

//     digitalWrite(ncs2, HIGH);
//   }
// }

void interpretCommand(String message) {
  message.trim(); // Remove leading and trailing white space
  int len = message.length();
  // Message will always be 6 bytes numeric
  // The first 3 characters are valve 1 duration
  // Second 3 characters are valve 2 duration

  if (len != 6) {
    Serial.println("#"); // "#" means error
    return;
  }
  String cmd1 = message.substring(0, 3);
  String cmd2 = message.substring(3);
  if (cmd1.toInt() > 0) {
    // Change valve 1 variables
    valve1Start = millis();
    valve1State = 1;
    valve1Dur = cmd1.toInt();
    digitalWrite(valve1Pin, HIGH);
  }
  // Do the same thing with valve 2
  if (cmd2.toInt() > 0) {
    // Change valve 2 variables
    valve2Start = millis();
    valve2State = 1;
    valve2Dur = cmd2.toInt();
    digitalWrite(valve2Pin, HIGH);
  }

  // Construct data string
  String dataString = "dp," + String(dP, 3) + ",dr," + String(dR, 3) + ",dy," + String(dY, 3) + ",l1," + String(lickCount1) + ",l2," + String(lickCount2) + ",v1," + String(valve1State) + ",v2," + String(valve2State) + ",dta," + String(dt) + ",dtmsg," + String(millis() - lastMsgTime);

  // Send message
  Serial.println(dataString);
  lastMsgTime = millis();
  lickCount1 = 0;
  lickCount2 = 0;
}

// int convTwosComp(int b) {
//   // Convert from 2's complement
//   if (b & 0x80) {
//     b = -1 * ((b ^ 0xff) + 1);
//   }
//   return b;
// }

// void readXY(int *xy, int sensor) {
//     byte xL = (sensor == 1) ? adns1_read_reg(REG_Delta_X_L) : adns2_read_reg(REG_Delta_X_L);
//     byte xH = (sensor == 1) ? adns1_read_reg(REG_Delta_X_H) : adns2_read_reg(REG_Delta_X_H);
//     byte yL = (sensor == 1) ? adns1_read_reg(REG_Delta_Y_L) : adns2_read_reg(REG_Delta_Y_L);
//     byte yH = (sensor == 1) ? adns1_read_reg(REG_Delta_Y_H) : adns2_read_reg(REG_Delta_Y_H);

//     xy[0] = (xH << 8) | xL;
//     xy[1] = (yH << 8) | yL;

//     // Convert from 2's complement for signed values
//     if (xy[0] & 0x8000) {
//         xy[0] = xy[0] - 0x10000;
//     }
//     if (xy[1] & 0x8000) {
//         xy[1] = xy[1] - 0x10000;
//     }
// }

void readXY1(void){  
  digitalWrite(ncs1, LOW);
  adns1_write_reg(REG_Motion, 0x01);
  // adns1_read_reg(REG_Motion);
  
  Motion = (adns1_read_reg(REG_Motion) & (1 << (8-1))) != 0;
  xL = adns1_read_reg(REG_Delta_X_L);
  xH = adns1_read_reg(REG_Delta_X_H);
  yL = adns1_read_reg(REG_Delta_Y_L);
  yH = adns1_read_reg(REG_Delta_Y_H);
  xy1dat[0] = (xH << 8) + xL;
  xy1dat[1] = (yH << 8) + yL;

  if(xy1dat[0] & 0x8000){
    xy1dat[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy1dat[1] & 0x8000){
    xy1dat[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  digitalWrite(ncs1, HIGH);
}

void readXY2(void){
  digitalWrite(ncs2,LOW);
  adns2_write_reg(REG_Motion, 0x01);
  
  Motion = (adns2_read_reg(REG_Motion) & (1 << (8-1))) != 0;
  xL = adns2_read_reg(REG_Delta_X_L);
  xH = adns2_read_reg(REG_Delta_X_H);
  yL = adns2_read_reg(REG_Delta_Y_L);
  yH = adns2_read_reg(REG_Delta_Y_H);
  xy2dat[0] = (xH << 8) + xL;
  xy2dat[1] = (yH << 8) + yL;

  if(xy2dat[0] & 0x8000){
    xy2dat[0] = -1 * ((xy[0] ^ 0xffff) + 1);
  }
  if (xy2dat[1] & 0x8000){
    xy2dat[1] = -1 * ((xy[1] ^ 0xffff) + 1);
  }
  digitalWrite(ncs2,HIGH);     
}

void loop() {

  dt = micros() - absTime;
  absTime = micros();

  // UpdatePointer1();
  // UpdatePointer2();

  // xy1dat[0] = convTwosComp(xy1dat[0]);
  // xy1dat[1] = convTwosComp(xy1dat[1]);
  // xy2dat[0] = convTwosComp(xy2dat[0]);
  // xy2dat[1] = convTwosComp(xy2dat[1]);

  // Read motion data from both sensors
  readXY1(); // Sensor 1
  readXY2(); // Sensor 2

  dP = px1 * xy1dat[0] + py1 * xy1dat[1] + px2 * xy2dat[0] + py2 * xy2dat[1];
  dR = rx1 * xy1dat[0] + ry1 * xy1dat[1] + rx2 * xy2dat[0] + ry2 * xy2dat[1];
  dY = yx1 * xy1dat[0] + yy1 * xy1dat[1] + yx2 * xy2dat[0] + yy2 * xy2dat[1];

  analogWrite(pVelPin,dP*vel_gain+2048);
  analogWrite(rVelPin,dR*vel_gain+2048);
  analogWrite(yVelPin,dY*vel_gain+2048);

  // Update licks - for some reason abs() was causing error 
  unsigned long read1 = digitalRead(lick1Pin);
  unsigned long read2 = digitalRead(lick2Pin);

  if (lastLick1 != read1) {
    lickCount1++;
  }
  if (lastLick2 != read2) {
    lickCount2++;
  }
  lastLick1 = read1;
  lastLick2 = read2;

  // Update valve 1
  if (valve1State == 1) {
    // Check to see if enough time has elapsed
    unsigned long tHigh = millis() - valve1Start; // abs() was causing error here too, switched to if()
    if ((tHigh > valve1Dur) || (tHigh < 0) || (tHigh > 10000)) {
      valve1State = 0;
      digitalWrite(valve1Pin, LOW);
      valve1Dur = 0;
    }
  }

  // Update valve 2
  if (valve2State == 1) {
    // Check to see if enough time has elapsed
    unsigned long tHigh = millis() - valve2Start;
    if ((tHigh > valve2Dur) || (tHigh < 0) || (tHigh > 10000)) {
      valve2State = 0;
      digitalWrite(valve2Pin, LOW);
      valve2Dur = 0;
    }
  }

  static String usbMessage = ""; // Initialize usbMessage to empty string,
  while (Serial.available() > 0) {
    // Read next char if available
    char inByte = Serial.read();
    if (inByte == '\n') {
      // The new-line character ('\n') indicates a complete message
      // So interpret the message and then clear buffer
      interpretCommand(usbMessage);
      usbMessage = ""; // Clear message buffer
    } else {
      // Append character to message buffer
      usbMessage = usbMessage + inByte;
    }
  }
  delayMicroseconds(10);

}