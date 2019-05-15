/*   This file is part of the Razor AHRS Firmware    */
#ifndef	 ADXL345_cpp
#define ADXL345_cpp

#include <Arduino.h>
#include "__I2C.cpp"
//======================================
// DCM parameters:
#define Kp_ROLLPITCH (0.02f    * 256.0f)
#define Ki_ROLLPITCH (0.00002f * 256.0f)
//=====================================
#define ACCEL_ADDRESS 0x53							// 0x53 = 0xA6 / 2

#define BW_RATE       0x2C
#define POWER_CTL     0x2D
#define DATA_FORMAT   0x31

#define DATAX0        0x32							// Output data
#define DATAX1        0x33
#define DATAY0        0x34
#define DATAY1        0x35
#define DATAZ0        0x36
#define DATAZ1        0x37
//-------------------------------------
#define Measure				0x08

#define FULL_RES			0x08
#define Range2G				0x00
#define Range4G				0x01
#define Range8G				0x02
#define Range16G			0x03

#define RATE50Hz			0x09
#define RATE100Hz			0x0A
//-------------------------------------
#define aRes          (1.0/256.0)				// for +/- 2G
//=====================================
class ADXL345: virtual public I2C
{
protected:
	//-----------------------------------
	void Acc_Init()
	{
		I2Cwrite(ACCEL_ADDRESS, POWER_CTL, Measure);//Power register, Measurement mode
		delay(5);
		I2Cwrite(ACCEL_ADDRESS, DATA_FORMAT, (FULL_RES | Range2G));//Data format register, Set to full resolution 
		delay(5);
		// Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
		I2Cwrite(ACCEL_ADDRESS, BW_RATE, RATE50Hz);// Rate, Set to 50Hz, normal operation 
		delay(5);
	}
	//-----------------------------------
	byte ReadAccBuf(byte buff[])
	{
		return (I2Cread(ACCEL_ADDRESS, DATAX0, 6, buff));  // All bytes received?
	}
	//-----------------------------------
};	//class
//=====================================
#endif	/* ADXL345_cpp */

