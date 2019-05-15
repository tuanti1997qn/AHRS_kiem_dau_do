//======================================
#ifndef	MPU6050_cpp
#define MPU6050_cpp

#include <Arduino.h>
#include "__I2C.cpp"
//======================================
// DCM parameters:
#define Kp_ROLLPITCH (0.02f    * 256.0f)
#define Ki_ROLLPITCH (0.00002f * 256.0f)
//======================================
#define MPU6050_ADDRESS 		0x68					// Device address when ADO = 0

#define CLOCK_PLL_XGYRO		0x01

#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D

#define INT_PIN_CFG				0x37
#define INT_ENABLE        0x38

#define ACCEL_XOUT_H				0x3B
#define ACCEL_XOUT_L				0x3C
#define ACCEL_YOUT_H				0x3D
#define ACCEL_YOUT_L				0x3E
#define ACCEL_ZOUT_H				0x3F
#define ACCEL_ZOUT_L				0x40

#define TEMP_OUT_H					0x41
#define TEMP_OUT_L					0x42

#define GYRO_XOUT_H				0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

#define PWR_MGMT_1					0x6B					// Device defaults to the SLEEP mode
//======================================
#define GFS_250DPS					0x00					//  250DPS
#define GFS_500DPS					0x01					//  500DPS
#define GFS_1000DPS				0x02					// 1000DPS
#define GFS_2000DPS				0x03					// 2000DPS

#define AFS_2G							0x00					// +/- 2G
//--------------------------------------
#define Gscale						GFS_1000DPS			//maybe changed
#define Ascale						AFS_2G

#define gRes ((float)(250L << Gscale)/32768.0f)	// for Grad/Sec
#define aRes (2.0/32768.0)								// for Gravity = 1;
//======================================
class	MPU6050: 	virtual public I2C
{
protected:
	//------------------------------------
	void MPU_Init()
	{
		// Reset Gyro
		I2Cwrite(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);
		delay(20);
		// Set_Clock PLL X
		I2Cwrite(MPU6050_ADDRESS, PWR_MGMT_1, CLOCK_PLL_XGYRO);
		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		I2Cwrite(MPU6050_ADDRESS, CONFIG, 0x03);  
		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		I2Cwrite(MPU6050_ADDRESS, SMPLRT_DIV, 0x09);  // Use a 100 Hz rate; a rate consistent with the filter update rate 
		//set Full Scale Gyro Range
		I2Cwrite(MPU6050_ADDRESS, GYRO_CONFIG, (Gscale << 3) );
		//set Full Scale Accel Range
		I2Cwrite(MPU6050_ADDRESS, ACCEL_CONFIG, (Ascale << 3) );
	};
	//------------------------------------
	byte ReadGyrBuf(byte buff[])
	{
		return (I2Cread(MPU6050_ADDRESS, GYRO_XOUT_H, 6, buff));
	};
	//------------------------------------
	byte ReadAccBuf(byte buff[])
	{
		return (I2Cread(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, buff));
	};
	//------------------------------------
	byte ReadAccTemGyrBuf(byte buff[])
	{
		return (I2Cread(MPU6050_ADDRESS, ACCEL_XOUT_H, 6+2+6, buff));
	};
	//------------------------------------
public:
	//------------------------------------
	float ReadTemperature(void)						//temperature in degrees of Celsius
	{
		byte buf[2];
		I2Cread(MPU6050_ADDRESS, TEMP_OUT_H, 2, buf);
		float T = Bytes2Float(buf[0], buf[1]);
		T = (T + 521.0f) * (1.0f/ 340.0f) + 35.0f;				//convert to degrees of Celsius
		return (T);
	};
};	//class
//======================================
#endif	/* MPU6050_cpp */
