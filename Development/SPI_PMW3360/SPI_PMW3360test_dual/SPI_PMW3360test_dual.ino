/*
 * This example bypasses the hardware motion interrupt pin
 * and polls the motion data registers at a fixed interval
 */

#include <SPI.h>
#include <avr/pgmspace.h>

byte initComplete=0;
byte Mot1 = 0;
byte Mot2 = 0;
byte xH;
byte xL;
byte yH;
byte yL;
int xy1dat[2];
int xy2dat[2];
double dP;
double dR;
double dY;
int pCum = 0;
int rCum = 0;
int yCum = 0;

// Cumulative XY readings for debugging
int x1Cum = 0;
int y1Cum = 0;
int x2Cum = 0;
int y2Cum = 0;

const int ncs1 = 0;  //This is the SPI "slave select" pin that the sensor is hooked up to
const int ncs2 = 1;
const int pVelPin = 3;
const int rVelPin = 4;
const int yVelPin = 5;

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

// Registers
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

//Set this to what pin your "INT0" hardware interrupt feature is on
//#define Motion_Interrupt_Pin 18

//Be sure to add the SROM file into this sketch via "Sketch->Add File"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

void setup() {
  Serial.begin(38400);
  analogWriteFrequency(pVelPin,11500);
  analogWriteFrequency(rVelPin,11500);
  analogWriteFrequency(yVelPin,11500);
  analogWriteResolution(12);
  pinMode (ncs1, OUTPUT);
  pinMode (ncs2, OUTPUT);
  
  // pinMode(Motion_Interrupt_Pin, INPUT);
  // digitalWrite(Motion_Interrupt_Pin, HIGH);
  // attachInterrupt(9, UpdatePointer1, FALLING);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  //SPI.setClockDivider(4);

  // Resulution is set in performStartup1 > adns1_upload_firmware
  delay(1000);
  performStartup1();  
  delay(10);
  performStartup2();
  delay(10);
  
  delay(1500);
  dispRegisters1();
  delay(1500);
  dispRegisters2();
  delay(1500);
  initComplete=9;

}

void adns1_com_begin(){
  digitalWrite(ncs1, LOW);
}

void adns2_com_begin(){
  digitalWrite(ncs2, LOW);
}

void adns1_com_end(){
  digitalWrite(ncs1, HIGH);
}

void adns2_com_end(){
  digitalWrite(ncs2, HIGH);
}

byte adns1_read_reg(byte reg_addr){
  adns1_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-ncs1 for read operation is 120ns
  adns1_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-ncs1

  return data;
}

byte adns2_read_reg(byte reg_addr){
  adns2_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-ncs1 for read operation is 120ns
  adns2_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-ncs1

  return data;
}

void adns1_write_reg(byte reg_addr, byte data){
  adns1_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-ncs1 for write operation
  adns1_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-ncs1. Could be shortened, but is looks like a safe lower bound 
}

void adns2_write_reg(byte reg_addr, byte data){
  adns2_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-ncs1 for write operation
  adns2_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-ncs1. Could be shortened, but is looks like a safe lower bound 
}		
																
void adns1_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 1...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  adns1_write_reg(Config2, 0x20);
  
  // write 0x1d in SROM_enable reg for initializing
  adns1_write_reg(SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns1_write_reg(SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns1_com_begin();
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  adns1_read_reg(SROM_ID);

  //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
  adns1_write_reg(Config2, 0x00);

  // set initial CPI resolution
  adns1_write_reg(Config1, 0x77); // Max resolution at 12000 cpi
  delay(10);
  
  adns1_com_end();										  
  }
  
  void adns2_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware to chip 2...");

  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  adns2_write_reg(Config2, 0x20);
  
  // write 0x1d in SROM_enable reg for initializing
  adns2_write_reg(SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns2_write_reg(SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns2_com_begin();
  SPI.transfer(SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }

  //Read the SROM_ID register to verify the ID before any other register reads or writes.
  adns2_read_reg(SROM_ID);

  //Write 0x00 to Config2 register for wired mouse or 0x20 for wireless mouse design.
  adns2_write_reg(Config2, 0x00);

  // set initial CPI resolution
  adns2_write_reg(Config1, 0x77); // Max resolution at 12000 cpi
  delay(1500); 								
  
  adns2_com_end();				  
  }


void performStartup1(void){
  adns1_com_end(); // ensure that the serial port is reset
  adns1_com_begin(); // ensure that the serial port is reset
  adns1_com_end(); // ensure that the serial port is reset
  adns1_write_reg(Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns1_read_reg(Motion);
  adns1_read_reg(Delta_X_L);
  adns1_read_reg(Delta_X_H);
  adns1_read_reg(Delta_Y_L);
  adns1_read_reg(Delta_Y_H);
  // upload the firmware
  adns1_upload_firmware();
  delay(10);
  Serial.println("Optical Chip 1 Initialized");
  }
  
  void performStartup2(void){
  adns2_com_end(); // ensure that the serial port is reset
  adns2_com_begin(); // ensure that the serial port is reset
  adns2_com_end(); // ensure that the serial port is reset
  adns2_write_reg(Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns2_read_reg(Motion);
  adns2_read_reg(Delta_X_L);
  adns2_read_reg(Delta_X_H);
  adns2_read_reg(Delta_Y_L);
  adns2_read_reg(Delta_Y_H);
  // upload the firmware
  adns2_upload_firmware();
  delay(10);
  Serial.println("Optical Chip 2 Initialized");
  }

void UpdatePointer1(void){
  if(initComplete==9){

    digitalWrite(ncs1,LOW);

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    adns1_write_reg(Motion, 0x01);
    adns1_read_reg(Motion);

    xy1dat[0] = (int)adns1_read_reg(Delta_X_L);
    xy1dat[1] = (int)adns1_read_reg(Delta_Y_L);
    
    digitalWrite(ncs1,HIGH);
    }
  }
  
  void UpdatePointer2(void){
  if(initComplete==9){

    digitalWrite(ncs2,LOW);

    //write 0x01 to Motion register and read from it to freeze the motion values and make them available
    adns2_write_reg(Motion, 0x01);
    adns2_read_reg(Motion);

    xy2dat[0] = (int)adns2_read_reg(Delta_X_L);
    xy2dat[1] = (int)adns2_read_reg(Delta_Y_L);
    
    digitalWrite(ncs2,HIGH);
    }
  }

void dispRegisters1(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  const char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","CPI"  };
  byte regres;

  digitalWrite(ncs1,LOW);

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

void dispRegisters2(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x0F  };
  const char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","CPI"  };
  byte regres;

  digitalWrite(ncs2,LOW);

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


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }


void loop() {

  Mot1 = (adns1_read_reg(Motion) & (1 << (8-1))) != 0;
  UpdatePointer1();
  delay(10);
  Mot2 = (adns2_read_reg(Motion) & (1 << (8-1))) != 0;
  UpdatePointer2();

	xy1dat[0] = convTwosComp(xy1dat[0]);
	xy1dat[1] = convTwosComp(xy1dat[1]);
	xy2dat[0] = convTwosComp(xy2dat[0]);
	xy2dat[1] = convTwosComp(xy2dat[1]);

	dP = px1*xy1dat[0] + py1*xy1dat[1] + px2*xy2dat[0] + py2*xy2dat[1];
  dR = rx1*xy1dat[0] + ry1*xy1dat[1] + rx2*xy2dat[0] + ry2*xy2dat[1];
  dY = yx1*xy1dat[0] + yy1*xy1dat[1] + yx2*xy2dat[0] + yy2*xy2dat[1];

  analogWrite(pVelPin,dP+2048);
  analogWrite(rVelPin,dR+2048);
  analogWrite(yVelPin,dY+2048);

  pCum = pCum*0.9 + dP;
  rCum = rCum*0.9 + dR;
  yCum = yCum*0.9 + dY;					  

	x1Cum = x1Cum + xy1dat[0];
	y1Cum = y1Cum + xy1dat[1];
	
	x2Cum = x2Cum + xy2dat[0];
	y2Cum = y2Cum + xy2dat[1];
	  
	Serial.println("Prod ID = " + String(adns1_read_reg(Product_ID)));
	Serial.println("Prod2 ID = " + String(adns2_read_reg(Product_ID)));
	Serial.println("Mot1 = " + String(Mot1));
	Serial.println("Mot2 = " + String(Mot2));
  Serial.println("x1 = " + String(xy1dat[0]));
	Serial.println("y1 = " + String(xy1dat[1]));
	Serial.println("x2 = " + String(xy2dat[0]));
	Serial.println("y2 = " + String(xy2dat[1]));
	Serial.println("x1Cum = " + String(x1Cum));
	Serial.println("y1Cum = " + String(y1Cum));
	Serial.println("x2Cum = " + String(x2Cum));
	Serial.println("y2Cum = " + String(y2Cum));
	Serial.println("Squal1 = " + String(adns1_read_reg(SQUAL)));
	Serial.println("Squal2 = " + String(adns2_read_reg(SQUAL)));
	  
	delay(10);
    
  }

