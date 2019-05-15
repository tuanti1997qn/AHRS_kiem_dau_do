/*===========================================================\
| "9DOF Razor IMU" hardware version:                         |
| GY-85 with QMC5883 magnetrometer						                |               
| https://www.sparkfun.com/products/retired/10736            |
| ---------------------------------------------------------- |
| ITG-3200 : Gyro                                            |U
| ADXL345  : Accelerometer                                   |R
| QMC5883  : Magnetometer                                    |L
\===========================================================*/
#ifndef	SENSOR
#define SENSOR

#include <Arduino.h>
#include "__ITG3200.cpp"
#include "__ADXL345.cpp"
#include "__QMC5883.cpp"
//========================================
class GY85QMC:	public ITG3200,
								public ADXL345,
								public QMC5883
{ 
public:
	//--------------------------------------
	void Init()
	{
		I2C_Init();
		Gyr_Init();
		Acc_Init();
		Mag_Init();
	};
	//--------------------------------------
	void Read(float G[],float A[],float M[])
	{
		Read_Gyr(G);
		Read_Acc(A);
		Read_Mag(M);
	}
	//--------------------------------------
	void Read_Gyr(float gyr[])							//in Grad/Sec
	{
		byte buf[6]; ReadGyrBuf(buf);
		gyr[0] = -gRes*Bytes2Float(buf[2],buf[3]);// X axis (internal sensor -y axis)
		gyr[1] = -gRes*Bytes2Float(buf[0],buf[1]);// Y axis (internal sensor -x axis)
		gyr[2] = -gRes*Bytes2Float(buf[4],buf[5]);// Z axis (internal sensor -z axis)
	}
	//--------------------------------------
	void Read_Acc(float acc[])							// in G (gravity == 1)
	{
		byte buf[6]; ReadAccBuf(buf);
		acc[0] = aRes*Bytes2Float(buf[3], buf[2]);// X axis (internal sensor y axis)
		acc[1] = aRes*Bytes2Float(buf[1], buf[0]);// Y axis (internal sensor x axis)
		acc[2] = aRes*Bytes2Float(buf[5], buf[4]);// Z axis (internal sensor z axis)
	}
	//--------------------------------------
	void Read_Mag(float mag[])							// in microTesla
	{
		byte buf[6]; ReadMagBuf(buf);
		mag[0] = -mRes*Bytes2Float(buf[3], buf[2]);	// X axis (internal sensor -y axis)
		mag[1] = -mRes*Bytes2Float(buf[1], buf[0]);	// Y axis (internal sensor -x axis)
		mag[2] = -mRes*Bytes2Float(buf[5], buf[4]);	// Z axis (internal sensor -z axis)
	};
	//--------------------------------------
	float ReadPressure(void){ return(0.0f); }	//dummy pressure
};
//========================================
#endif	/* SENSOR */

