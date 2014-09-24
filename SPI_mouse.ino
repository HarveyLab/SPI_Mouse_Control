// Define the pins
const int chipSelectPin = 10;
const int xVelPin = 5;
const int yVelPin = 20;
int xVel = 0;
int yVel = 0;
int xFilt = 0;
int yFilt = 0;
  
// the sensor communicates using SPI, so include the library:
#include <SPI.h>

void setup() {
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin,HIGH);
  analogWriteFrequency(xVelPin,187500);
  analogWriteFrequency(yVelPin,187500);
  analogWriteResolution(8);
  // start the SPI library:
  SPI.begin();
  // give the sensor time to set up:
  delay(100);
  SPI.setDataMode(SPI_MODE3);
  delay(100);
  SPI.setBitOrder(MSBFIRST);
  delay(100);
  analogWriteResolution(8);
  delay(100);
}

void loop() {
    delay(5);
    byte Motion = ((readRegister(0x02) & (1 << 8-1)) != 0);
    if (Motion == 1){
      xVel = readRegister(0x03);
      yVel = readRegister(0x04); 
      if (xVel > 127){
        xVel = xVel-256;
      }
      if (yVel > 127){
        yVel = yVel-256;
      }
     }
      else {
        xVel = 0;
        yVel = 0;
      }
    xFilt = xFilt*.67 + xVel*.33;
    yFilt = yFilt*.67 + yVel*.33;
    analogWrite(xVelPin, (xFilt+128));
    analogWrite(yVelPin, (yFilt+128));
}

byte readRegister(byte address){ 
  //Take CS pin low to select chip
  digitalWrite(chipSelectPin,LOW);
  //Send register address and value
  SPI.transfer(address & 0x7f);
  delayMicroseconds(100);
  byte data = SPI.transfer(0x00);
  delayMicroseconds(1);
  digitalWrite(chipSelectPin,HIGH);
  return data;
}

void writeRegister(byte address, byte data){
  digitalWrite(chipSelectPin,LOW);
  SPI.transfer(address | 0x80);  
  SPI.transfer(data);
  delayMicroseconds(20);
  digitalWrite(chipSelectPin,HIGH);
}
