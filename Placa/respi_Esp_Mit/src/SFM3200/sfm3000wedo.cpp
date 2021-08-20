/*
  SFM3000wedo.cpp - Library for reading values from flowmeter Sensirion SFM3000wedo
  https://www.sensirion.com/en/flow-sensors/mass-flow-meters-for-high-precise-measurement-of-gases/low-pressure-drop-mass-flow-meter/
  Created by WeDo, Zuerich 20170616
  
  Released into the public domain.
*/

#include "sfm3000wedo.h"
#include <Wire.h>

 
//SFM3000wedo::SFM3000wedo(uint8_t i2cAddress)
SFM3000wedo::SFM3000wedo(int i2cAddress)
{
	//: mI2cAddress(i2cAddress)
	mI2cAddress = i2cAddress;
}



void SFM3000wedo::init()
{
	int a = 0;
	int b = 0;
	int c = 0; 
	
	Wire.begin();
	Wire.setClock(40000);
	//Serial.begin(9600);
	delay(1000);
	Wire.beginTransmission(byte(mI2cAddress)); // transmit to device with I2C mI2cAddress
	// Wire.beginTransmission(byte(mI2cAddress)); // transmit to device with I2C mI2cAddress
	Wire.write(byte(0x10));      //
	Wire.write(byte(0x00));      //
	Wire.endTransmission();
	delay(5);
	
	Wire.requestFrom(mI2cAddress, 3); //
	a = Wire.read(); // first received byte stored here
	b = Wire.read(); // second received byte stored here
	c = Wire.read(); // third received byte stored here
	Wire.endTransmission();
	//Serial.print(a);
	//Serial.print(b);
	//Serial.println(c);
	
	delay(5);
	
	Wire.requestFrom(mI2cAddress, 3); //
	a = Wire.read(); // first received byte stored here
	b = Wire.read(); // second received byte stored here
	c = Wire.read(); // third received byte stored here
	Wire.endTransmission();
	//Serial.print(a);
	//Serial.print(b);
	//Serial.println(c);
	
	//delay(5);
	
}
 
SFM3000_Value_t SFM3000wedo::getvalue()
{
    Wire.requestFrom(mI2cAddress, 3); // read 3 bytes from device with address 0x40
	uint16_t a = Wire.read(); // first received byte stored here. The variable "uint16_t" can hold 2 bytes, this will be relevant later
	uint8_t b = Wire.read(); // second received byte stored here
	uint8_t crc = Wire.read(); // crc value stored here
	uint8_t mycrc = 0; // initialize crc variable
	mycrc = crc8(a, mycrc); // let first byte through CRC calculation
	mycrc = crc8(b, mycrc); // and the second byte too
	SFM3000_Value_t value;
	if (mycrc != crc) { // check if the calculated and the received CRC byte matches
		//Serial.println("Error: wrong CRC");
		value.crcOK = false;
	} else {
		value.crcOK = true;
	}
	a = (a << 8) | b; // combine the two received bytes to a 16bit integer value
	// a >>= 2; // remove the two least significant bits
	//float Flow = (float)a;
	int Flow=a;
	
	value.value = Flow;
	return value;
}

void SFM3000wedo::reset(void) {
	Wire.beginTransmission(byte(mI2cAddress)); // transmit to device with I2C mI2cAddress
	Wire.write(byte(0x20));      //
	Wire.write(byte(0x00));      //
	Wire.endTransmission();

}

uint8_t SFM3000wedo::crc8(const uint8_t data, uint8_t crc)
{
	   crc ^= data;
	   for ( uint8_t i = 8; i; --i ) {
		   crc = ( crc & 0x80 )
		   ? (crc << 1) ^ 0x31
		   : (crc << 1);
		}
	return crc;
}
