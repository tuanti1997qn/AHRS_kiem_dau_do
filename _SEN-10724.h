/*===========================================================\
| "9DOF Razor IMU" hardware versions:                        |
| SEN-10724                                                  |
https://www.sparkfun.com/products/retired/10724              |
| ---------------------------------------------------------- |
| ?ATMega328@3.3V, 8MHz                                      |?
| ITG-3200 : Gyro                                            |U
| ADXL345  : Accelerometer                                   |R
| HMC5883L : Magnetometer                                    |U
| ---------------------------------------------------------- |
|  Arduino IDE : Select board                                |
|	?"Arduino Pro or Pro Mini                                  |?
|	"(3.3v, 8Mhz) w/ATmega328"                                 |?
\===========================================================*/
#ifndef	SENSOR
#define SENSOR

#include <Arduino.h>
#include "__ITG3200.cpp"
#include "__ADXL345.cpp"
#include "__HMC5883L.cpp"
//======================================
class SEN_10724:	public ITG3200,
									public ADXL345,
									public HMC5883L
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
	void Read_Gyr(float gyr[])								//in Grad/Sec
	{
		byte buf[6]; ReadGyrBuf(buf);
		gyr[0] = -Bytes2Float(buf[2],buf[3])*gRes;// X axis (internal sensor -y axis)
		gyr[1] = -Bytes2Float(buf[0],buf[1])*gRes;// Y axis (internal sensor -x axis)
		gyr[2] = -Bytes2Float(buf[4],buf[5])*gRes;// Z axis (internal sensor -z axis)
	}
	//--------------------------------------
	void Read_Acc(float acc[])								// in G (gravity == 1)
	{
		byte buf[6]; ReadAccBuf(buf);
		acc[0] = Bytes2Float(buf[3], buf[2])*aRes;// X axis (internal sensor y axis)
		acc[1] = Bytes2Float(buf[1], buf[0])*aRes;// Y axis (internal sensor x axis)
		acc[2] = Bytes2Float(buf[5], buf[4])*aRes;// Z axis (internal sensor z axis)
	}
	//--------------------------------------
	void Read_Mag(float mag[])								// in microTesla
	{
		byte buf[6]; ReadMagBuf(buf);
		mag[0] =  mRes*Bytes2Float(buf[0], buf[1]);	// X axis (internal sensor +x axis)
		mag[1] = -mRes*Bytes2Float(buf[4], buf[5]);	// Y axis (internal sensor -z axis)
		mag[2] = -mRes*Bytes2Float(buf[2], buf[3]);	// Z axis (internal sensor -y axis)
	};
	//--------------------------------------
	float ReadPressure(void){ return(0.0f); }	//dummy pressure
	//--------------------------------------
};
//========================================
#endif	/* SENSOR */
