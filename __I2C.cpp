//======================================
#ifndef	I2Ccommon_cpp
#define I2Ccommon_cpp

#include <Arduino.h>
#include <Wire.h>
//======================================
/*
#define INLINE   __attribute__((__always_inline__))
#define NOINLINE __attribute__((__noinline__))
*/
//======================================
// Wire.h read and write protocols
//======================================
class I2C
{ 
public:
	//------------------------------------
	void I2C_Init()
	{
		Wire.begin();												// Initialize the I2C bus		
    #ifdef STM32
    Wire.setSDA(PB_9);
    Wire.setSCL(PB_8);  
    Wire.setClock(100000);
    #endif
	}
	//------------------------------------
	// The I2C_Check uses the return value of
	// the Write.endTransmisstion to see if
	// a device did acknowledge to the address.
	byte I2Ccheck(byte I2Cadr)
	{
		Wire.beginTransmission(I2Cadr);			// Initialize the Tx buffer
		return Wire.endTransmission();  		// Send the Tx buffer
	}
	//------------------------------------
	byte I2Cwrite(byte I2Cadr, byte Adr, byte Data)
	{
		Wire.beginTransmission(I2Cadr);			// Initialize the Tx buffer
		Wire.write(Adr);           					// Put slave register address in Tx buffer
		Wire.write(Data);                 		// Put data in Tx buffer
		return Wire.endTransmission();   		// Send the Tx buffer
	}
	//------------------------------------
	byte I2Cread(byte I2Cadr, byte Adr)
	{
		Wire.beginTransmission(I2Cadr);			// Initialize the Tx buffer
		Wire.write(Adr);											// Put slave register address in Tx buffer
		Wire.endTransmission(false);				// Send the Tx buffer, but send a restart to keep connection alive
		Wire.requestFrom(I2Cadr,(byte)1);		// Read one byte from slave register address 
		return (Wire.read());								// Fill Rx buffer with result
	}
	//------------------------------------
	byte I2Cread(byte I2Cadr, byte Adr, byte Count, byte Buf[])
	{  
		Wire.beginTransmission(I2Cadr);			// Initialize the Tx buffer
		Wire.write(Adr);											// Put slave register address in Tx buffer
		Wire.endTransmission(false);				// Send the Tx buffer, but send a restart to keep connection alive
		byte i = 0;
		Wire.requestFrom(I2Cadr, Count);		// Read bytes from slave register address 
		while (Wire.available())
    Buf[i++] = Wire.read();							// Put read results in the Rx buffer
		return(i);
	}
	//------------------------------------
	float Bytes2Float(byte HByte, byte LByte)
	{
		return ((((int)HByte) << 8) | LByte);	//convert 2 bytes to float
	}
	//------------------------------------
};	//class
//======================================
#endif	/* I2Ccommon_cpp */
