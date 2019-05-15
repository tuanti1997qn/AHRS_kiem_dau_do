/*   This file is part of the Razor AHRS Firmware    */
#ifndef	HMC5883L_cpp
#define HMC5883L_cpp

#include <Arduino.h>
#include "__I2C.cpp"
//======================================
// DCM parameters:
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f
//======================================
#define MAGN_ADDRESS  0x1E							// 0x1E = 0x3C / 2

#define CRA           0x00							// Configuration Register A
#define CRB           0x01							// Configuration Register B
#define MODE          0x02

#define DATA_XM       0x03							// Output Data
#define DATA_XL       0x04
#define DATA_YM       0x05
#define DATA_YL       0x06
#define DATA_ZM       0x07
#define DATA_ZL       0x08
//--------------------------------------
#define CM_Mode				0x00							//Continuouse Measurement Mode
//--------------------------------------
#define DO0						0x02							// data output rate lsb (CRA:02)
#define DORATE7_5Hz		(0x03 << DO0)			//7.5Hz
#define DORATE15Hz		(0x04 << DO0)			//default
#define DORATE30Hz		(0x05 << DO0)
#define DORATE75Hz		(0x06 << DO0)			//75Hz instead of 50Hz
//--------------------------------------
#define mRes          (1.0f/10.9f)				// im microTesla
//======================================
class HMC5883L: virtual public I2C
{
protected:
	//-----------------------------------
	void Mag_Init()
	{
		I2Cwrite(MAGN_ADDRESS, MODE, CM_Mode);// Set continuous mode (default 10Hz)
		delay(5);
		I2Cwrite(MAGN_ADDRESS,CRA,DORATE75Hz);// Set 75Hz
		delay(5);
	};
	//-----------------------------------
	byte ReadMagBuf(byte buff[])
	{
		return(I2Cread(MAGN_ADDRESS,DATA_XM,6,buff));  // All bytes received?
	};
	//-----------------------------------
};	//class
//======================================
#endif	/* HMC5883L_cpp */
