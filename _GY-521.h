#include <Arduino.h>
//========================================
#ifndef	SENSOR
#define SENSOR
//--------- GY-521 (GY6050) --------------
#include "__MPU6050.cpp"
//========================================
// dummy DCM parameters:
//#define Kp_YAW 0.0f
//#define Ki_YAW 0.0f
//========================================
class GY521: public MPU6050
{
public:
	//--------------------------------------
	void Init()
	{
		I2C_Init();
		MPU_Init();
	};
	//--------------------------------------
	void Read(float G[],float A[],float M[])
	{
		byte buf[6+2+6]; 
		I2Cread(MPU6050_ADDRESS, ACCEL_XOUT_H, 6+2+6, buf);
		//Read_Acc(A);
		A[0] =  aRes*Bytes2Float(buf[2], buf[3]);			// X axis (internal sensor y axis)
		A[1] =  aRes*Bytes2Float(buf[0], buf[1]);			// Y axis (internal sensor x axis)
		A[2] =  aRes*Bytes2Float(buf[4], buf[5]);			// Z axis (internal sensor z axis)
		//Read_Gyr(G);
		G[0] = -gRes*Bytes2Float(buf[8+2],buf[8+3]);	// X axis (internal sensor -y axis)
		G[1] = -gRes*Bytes2Float(buf[8+0],buf[8+1]);	// Y axis (internal sensor -x axis)
		G[2] = -gRes*Bytes2Float(buf[8+4],buf[8+5]);	// Z axis (internal sensor -z axis)
		//Read_Mag(M);
		for (int i=0; i<3; i++) M[i] = 0.0f;	//dummy
	};
	//--------------------------------------
	float ReadPressure(void){ return(0.0f); }	//dummy pressure
	//--------------------------------------
};
//========================================
#endif	/* SENSOR */

