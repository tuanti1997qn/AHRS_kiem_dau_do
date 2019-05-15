//======================================
#ifndef	ITG3200_cpp
#define ITG3200_cpp

#include <Arduino.h>
#include "__I2C.cpp"
//======================================
#define GYRO_ADDRESS   0x68 						// 0x68 = 0xD0 / 2

#define SMPLRT_DIV		 0x15
#define DLPF_CFG       0x16
#define INT_CFG        0x17

#define TEMP_OUT_H		 0x1B
#define TEMP_OUT_L		 0x1C

#define GYRO_XOUT_H		 0x1D
#define GYRO_XOUT_L		 0x1E
#define GYRO_YOUT_H		 0x1F
#define GYRO_YOUT_L		 0x20
#define GYRO_ZOUT_H		 0x21
#define GYRO_ZOUT_L		 0x22

#define PWR_MGM        0x3E
//-------------------------------------
#define H_RESET        0x80
#define GFS_2000DPS		 0x03							// 2000DPS
#define LPFB_42Hz			 0x03							// LPF 42Hz, Sample Rate 1kHz
//-------------------------------------
#define gRes (1.0f / 14.375f)						// for Grad/Sec
//=====================================
class	ITG3200: virtual public I2C
{
protected:
	//-----------------------------------
	void Gyr_Init()
	{
		// Power up reset defaults
		I2Cwrite(GYRO_ADDRESS, PWR_MGM, H_RESET);	// Power up reset defaults
		delay(5);
		// Select full-scale range of the gyro sensors
		// Set LP filter bandwidth to 42Hz
		I2Cwrite(GYRO_ADDRESS, DLPF_CFG, ( GFS_2000DPS << 3) | LPFB_42Hz );	// DLPF_CFG = 3, FS_SEL = 3
		delay(5);
		// Set sample rato to 100Hz
		I2Cwrite(GYRO_ADDRESS, SMPLRT_DIV, 0x09);	//  SMPLRT_DIV = 9 (100Hz)
		delay(5);
		// Set clock to PLL with z gyro reference
		I2Cwrite(GYRO_ADDRESS, PWR_MGM, 0x00);
		delay(5);
	};
	//-----------------------------------
	byte ReadGyrBuf(byte buff[])
	{
		return (I2Cread(GYRO_ADDRESS, GYRO_XOUT_H, 6, buff));  // All bytes received?
	};
	//-----------------------------------
public:
	//-----------------------------------
	float ReadTemperature(void)						//temperature in degrees of Celsius
	{
		byte buf[2];
		I2Cread(GYRO_ADDRESS, TEMP_OUT_H, 2, buf);
		float T = Bytes2Float(buf[0], buf[1]);
		T = (T + 13200.0f) * (1.0f/280.0f) + 35.0f;		//convert to degrees of Celsius
		return (T);
	};
	//-----------------------------------
};	//class
//=====================================
#endif	/* ITG3200_cpp */

