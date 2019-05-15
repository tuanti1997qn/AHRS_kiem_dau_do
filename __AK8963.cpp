//=====================================
#ifndef	AK8963_cpp
#define AK8963_cpp

#include <Arduino.h>
#include "__I2C.cpp"
//=====================================
// DCM parameters:
#define Kp_YAW 0.22f
#define Ki_YAW 0.0001f
//#define Kp_YAW 0.5f
//#define Ki_YAW 0.0001f

//=====================================

#define AK8963_ADDRESS  		0x0C				//Address of magnetometer

//Magnetometer Registers
#define AK8963_XOUT_L			0x03  			// output data
#define AK8963_XOUT_H			0x04
#define AK8963_YOUT_L			0x05
#define AK8963_YOUT_H			0x06
#define AK8963_ZOUT_L			0x07
#define AK8963_ZOUT_H			0x08

#define AK8963_CNTL				0x0A  			// Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASAX				0x10  			// Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY				0x11  			// Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ				0x12  			// Fuse ROM z-axis sensitivity adjustment value
//=====================================
#define MFS_16BITS				0x01					// 0.15 mG per LSB
#define MFS_14BITS				0x00
//-------------------------------------
#define Mscale MFS_16BITS
#define Mmode  0x06										// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
#define mRes (4912.0/32760.0)					// Proper scale to return microTesla
//=====================================
class	AK8963: virtual public I2C
{
public:
	//-----------------------------------
	void Init()
	{
		I2Cwrite(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer  
		delay(10);
		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		I2Cwrite(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
		delay(10);
	}
	//-----------------------------------
	byte ReadMagBuf(byte buff[])
	{
		return (I2Cread(AK8963_ADDRESS, AK8963_XOUT_L, 7, buff));
	};
	//-----------------------------------
};	//class
//=====================================
#endif	/* AK8963_cpp */

