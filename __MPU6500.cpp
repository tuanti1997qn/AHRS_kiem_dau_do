//=====================================
#ifndef	MPU6500_h
#define MPU6500_h

#include <Arduino.h>
#include "__I2C.cpp"
//=====================================
// DCM parameters:
#define Kp_ROLLPITCH (0.02f    * 256.0f)
#define Ki_ROLLPITCH (0.00002f * 256.0f)
//=====================================
#define MPU6500_ADDRESS 	0x68					// Device address when ADO = 0

#define SMPLRT_DIV        0x19
#define CONFIG            0x1A
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D

#define INT_PIN_CFG				0x37
#define INT_ENABLE        0x38

#define ACCEL_XOUT_H			0x3B
#define ACCEL_XOUT_L			0x3C
#define ACCEL_YOUT_H			0x3D
#define ACCEL_YOUT_L			0x3E
#define ACCEL_ZOUT_H			0x3F
#define ACCEL_ZOUT_L			0x40

#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42

#define GYRO_XOUT_H				0x43
#define GYRO_XOUT_L       0x44
#define GYRO_YOUT_H       0x45
#define GYRO_YOUT_L       0x46
#define GYRO_ZOUT_H       0x47
#define GYRO_ZOUT_L       0x48

#define PWR_MGMT_1				0x6B						// Device defaults to the SLEEP mode
//=====================================
#define GFS_250DPS				0x00						//  250DPS
#define GFS_500DPS				0x01						//  500DPS
#define GFS_1000DPS				0x02					// 1000DPS
#define GFS_2000DPS				0x03					// 2000DPS

#define AFS_2G						0x00						// +/- 2G
//-------------------------------------
#define Gscale						GFS_1000DPS			//maybe changed
#define Ascale						AFS_2G

#define gRes ((float)(250L << Gscale)/32768.0f)	// for Grad/Sec
#define aRes (2.0/32768.0)								// for Gravity = 1;
//=====================================
class	MPU6500: virtual public I2C
{
public:
	//-----------------------------------
	void Init(){
		// wake up device
		I2Cwrite(MPU6500_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
		delay(100); // Wait for all registers to reset 

		// get stable time source
		I2Cwrite(MPU6500_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
		delay(200); 
  
		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
		I2Cwrite(MPU6500_ADDRESS, CONFIG, 0x03);  

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		I2Cwrite(MPU6500_ADDRESS, SMPLRT_DIV, 0x09);  // Use a 100 Hz rate; a rate consistent with the filter update rate 
                                    // determined inset in CONFIG above
 
		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
		I2Cwrite(MPU6500_ADDRESS, GYRO_CONFIG, (Gscale << 3) ); // Write new GYRO_CONFIG value to register
  
		// Set accelerometer full-scale range configuration
		I2Cwrite(MPU6500_ADDRESS, ACCEL_CONFIG, (Ascale << 3) ); // Write new ACCEL_CONFIG register value

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		I2Cwrite(MPU6500_ADDRESS, ACCEL_CONFIG2, (8 | 0x03) ); // Write new ACCEL_CONFIG2 register value
		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
		// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips 
		// can join the I2C bus and all can be controlled by the Arduino as master
		I2Cwrite(MPU6500_ADDRESS, INT_PIN_CFG, 0x22);    
		I2Cwrite(MPU6500_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
		delay(100);
	}
	//-----------------------------------
	byte ReadGyrBuf(byte buff[])
	{
		return (I2Cread(MPU6500_ADDRESS, GYRO_XOUT_H, 6, buff));
	}
	//-----------------------------------
	byte ReadAccBuf(byte buff[])
	{
		return (I2Cread(MPU6500_ADDRESS, ACCEL_XOUT_H, 6, buff));
	}
	//-----------------------------------
	byte ReadAccTemGyrBuf(byte buff[])
	{
		return (I2Cread(MPU6500_ADDRESS, ACCEL_XOUT_H, 6+2+6, buff));
	}
	//-----------------------------------
	float ReadTemperature(void)						//temperature in degrees of Celsius
	{
		byte buf[2];
		I2Cread(MPU6500_ADDRESS, TEMP_OUT_H, 2, buf);
		float T = Bytes2Float(buf[0], buf[1]);
		T = T * (1.0f/ 333.87f) + 21.0f;				//convert to degrees of Celsius
		return (T);
	};
	//-----------------------------------
};	//class
//======================================
#endif	/* MPU6500_h */

