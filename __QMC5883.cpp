/*   This file is part of the Razor AHRS Firmware    */
#ifndef	QMC5883_cpp
#define QMC5883_cpp

#include <Arduino.h>
#include "__I2C.cpp"
//=====================================
// DCM parameters:
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f
//=====================================
#define MAGN_ADDRESS  0x0D
//------- Register Map: ---------------
#define DATA_XL       0x00							// Output Data
#define DATA_XM       0x01
#define DATA_YL       0x02
#define DATA_YM       0x03
#define DATA_ZL       0x04
#define DATA_ZM       0x05

#define STATUS        0x06							//DOR, OVL, DRDY

#define TOUT_L        0x07							//Temperature output Data
#define TOUT_M        0x08							//100LSB/grad Celsius

#define CTRL1					0x09							//OSR, RNG , ODR, MODE
#define CTRL2					0x0A							//SOFT_RST, ROL_PNT, INT_ENB

#define SET_RST_FBR		0x0B

#define CHIP_ID				0x0D							//read only CHIP_ID == 0xFF
//------- STATUS bits: ----------------
#define DOR						2									// Data skipped for reading
#define OVR						1									// Data overflow 
#define DRDY					0									// New data is ready
//------- CTRL1 bits: -----------------
#define Standby_Mode		0x00						//Operational modes bits
#define Continuous_Mode	0x01

#define ODR_10Hz			0x00							//Output data rate
#define ODR_50Hz			0x04
#define ODR_100Hz			0x08
#define ODR_200Hz			0x0C
																				//Full scale field range
#define RNG_2G				0x00							//12000 LSB/G
#define RNG_8G				0x10							// 3000 LSB/G

#define OSR_512				0x00							//Over sample Rate
#define OSR_256				0x40
#define OSR_128				0x80
#define OSR_64				0xC0
//------- CTRL2 bits: -----------------
#define SOFT_RST			0x80
#define ROL_PNT				0x40							//point roll-over function
#define INT_ENB				0x01
//-------------------------------------
#define mRes          (1.0f/120.0f)			// in microTesla
//=====================================
class QMC5883: virtual public I2C
{
protected:
	//-----------------------------------
	void Mag_Init()
	{
		I2Cwrite(MAGN_ADDRESS, SET_RST_FBR, 0x01);
		I2Cwrite(MAGN_ADDRESS, CTRL1, (Continuous_Mode | ODR_100Hz | RNG_2G | OSR_256));
	}
	//-----------------------------------
	byte ReadMagBuf(byte buff[])
	{
		return(I2Cread(MAGN_ADDRESS, DATA_XL, 6, buff));  // All bytes received?
	};
	//-----------------------------------
};	//class
//=====================================
#endif	/* QMC5883_cpp */

