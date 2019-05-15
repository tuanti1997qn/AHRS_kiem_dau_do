/*===========================================================\
| "9DOF Razor IMU" hardware version:                         |
| GY-9250                                                    |
| https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU9250REV1.0.pdf
| https://www.akm.com/akm/en/file/datasheet/AK8963C.pdf      |
| ---------------------------------------------------------- |
| MPU6500 : Gyroscope and Accelerometer                      |
| AK8963  : Magnetometer                                     |
\===========================================================*/
#include <Arduino.h>
//=====================================
#ifndef	SENSOR
#define SENSOR
//-------- sen-13762 (GY9250) ---------
#include "__MPU6500.cpp"
#include "__AK8963.cpp"
#include "__bmp280.cpp"
//=====================================
class GY91:	public MPU6500,
						public AK8963,
						public BMP280
{
public:
	//-----------------------------------
	void Init()
	{
		I2C_Init();
		MPU6500::Init();
		AK8963::Init();
		BMP280::Init();
	};
	//-----------------------------------
	void Read(float G[],float A[],float M[])
	{
		byte buf[6+2+6]; 
		ReadAccTemGyrBuf(buf);
		//Read_Acc
		A[0] = 	aRes*Bytes2Float(buf[2], buf[3]);			// X axis (internal sensor y axis)
		A[1] = 	aRes*Bytes2Float(buf[0], buf[1]);			// Y axis (internal sensor x axis)
		A[2] = 	aRes*Bytes2Float(buf[4], buf[5]);			// Z axis (internal sensor z axis)
		//Read_Gyr
		G[0] = -gRes*Bytes2Float(buf[8+2],buf[8+3]);	// X axis (internal sensor -y axis)
		G[1] = -gRes*Bytes2Float(buf[8+0],buf[8+1]);	// Y axis (internal sensor -x axis)
		G[2] = -gRes*Bytes2Float(buf[8+4],buf[8+5]);	// Z axis (internal sensor -z axis)
		//Read_Mag
		ReadMagBuf(buf);
		M[0] = -mRes*Bytes2Float(buf[1], buf[0]);			// X axis (internal sensor -x axis)
		M[1] = -mRes*Bytes2Float(buf[3], buf[2]);			// Y axis (internal sensor -y axis)
		M[2] =  mRes*Bytes2Float(buf[5], buf[4]);			// Z axis (internal sensor  z axis)
	};
	//--------------------------------------
	float ReadTemperature(void)	{
		return(ReadTemperatureBMP()); 								//temperature of bmp180
		// return(ReadTemperature());									//temperature of mpu6500
	}
	//--------------------------------------
};	//class
//======================================
#endif	/* SENSOR */

