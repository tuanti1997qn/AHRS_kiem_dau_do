#ifndef BMP280_CPP
#define BMP280_CPP

#include "Arduino.h"
#include "__I2C.cpp"

#define BMP280_ADDRESS							0x76

//-- Calibration data registers: -------
#define BMP280_REGISTER_DIG_T1		0x88
#define BMP280_REGISTER_DIG_T2		0x8A
#define BMP280_REGISTER_DIG_T3		0x8C

#define BMP280_REGISTER_DIG_P1		0x8E
#define BMP280_REGISTER_DIG_P2		0x90
#define BMP280_REGISTER_DIG_P3		0x92
#define BMP280_REGISTER_DIG_P4		0x94
#define BMP280_REGISTER_DIG_P5		0x96
#define BMP280_REGISTER_DIG_P6		0x98
#define BMP280_REGISTER_DIG_P7		0x9A
#define BMP280_REGISTER_DIG_P8		0x9C
#define BMP280_REGISTER_DIG_P9		0x9E

//--------- REGISTERS MAP: -------
#define BMP280_CHIPID							0xD0	//BMP280_CHIPID == 0x58
#define BMP280_REGISTER_VERSION		0xD1
#define BMP280_REGISTER_SOFTRESET	0xE0

#define BMP280_REGISTER_CAL26			0xE1  // R calibration stored in 0xE1-0xF0

#define BMP280_REGISTER_CONTROL		0xF4
#define BMP280_REGISTER_CONFIG		0xF5
#define BMP280_REGISTER_PRESSUREDATA	0xF7
#define BMP280_REGISTER_TEMPDATA			0xFA
//--------------------------------
#define osrs_t_16									0x20
#define osrs_t_17									0x40
#define osrs_t_18									0x60
#define osrs_t_19									0x80
#define osrs_t_20									0xE0
//--------------------------------
#define osrs_p_16									0x04
#define osrs_p_17									0x08
#define osrs_p_18									0x0C
#define osrs_p_19									0x10
#define osrs_p_20									0x1C
//--------------------------------
#define mode_sleep									0x00
#define mode_forced								0x01
#define mode_normal								0x03
//--------------------------------
class BMP280: virtual public I2C
{
private:
	//------------------------------------
	struct
	{
		word	dig_T1;
		int		dig_T2, dig_T3;
		word	dig_P1;
		int		dig_P2, dig_P3, dig_P4, dig_P5,
					dig_P6, dig_P7, dig_P8, dig_P9;
	};
	long	t_fine;
	float pressure;
	//------------------------------------
	unsigned long readADC(byte reg)
	{
		unsigned long value;
		byte buf[3];
		I2Cread(BMP280_ADDRESS, reg, 3, buf);
		value = ((unsigned long)buf[0] << 16)|((unsigned long)buf[1] << 8)|buf[2];
		return (value);		
	}
	//------------------------------------
public:
	//------------------------------------
	void Init(){
		//readCoefficients():
		I2Cread (BMP280_ADDRESS, BMP280_REGISTER_DIG_T1, 24, (byte *)&dig_T1);
		I2Cwrite(BMP280_ADDRESS, BMP280_REGISTER_CONTROL,(osrs_t_16|osrs_p_20|mode_normal));
	}
	//------------------------------------
	float ReadTemperatureBMP(){							//36-40 uSec
		long var1, var2;
		long adc_T = readADC(BMP280_REGISTER_TEMPDATA);
		adc_T >>= 4;

		var1  = ((adc_T >> 3) - (dig_T1 << 1))*dig_T2 >> 11;
		var2  = (((((adc_T>>4)-dig_T1)*((adc_T>>4)-dig_T1))>>12)*dig_T3)>>14;
		t_fine = var1 + var2;
		float T  = (t_fine * 5 + 128) >> 8;
		return T/100;
	}
	//------------------------------------
	float ReadPressure(){					//1020-1052 uSec
		int64_t var1, var2, p;
		// Must be done first to get the t_fine variable set up
		//ReadTemperature();
		long adc_P = readADC(BMP280_REGISTER_PRESSUREDATA);
		adc_P >>= 4;

		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1*var1*dig_P6;
		var2 = var2+((var1*dig_P5) << 17);
		var2 = var2+(((int64_t)dig_P4) << 35);
		var1 = (var1*var1*dig_P3 >> 8)+((var1*dig_P2)<<12);
		var1 = (0x800000000000L+var1)*(dig_P1)>>33;

		if (var1 == 0) return 0;  // avoid exception caused by division by zero

		p = 1048576 - adc_P;
		p = (((p << 31) - var2) * 3125) / var1;

		var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2 = (((int64_t)dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);

		pressure = (float)p/256;
		return (pressure);
	}
	//------------------------------------
	float ReadAltitude(float Pzero){	//2128-2140 uSec
		float altitude;
//		ReadTemperature();
//		float pressure = ReadPressure(); // in Si units for Pascal
		altitude = 44330.0 * (1.0 - pow(pressure / Pzero, 0.1903));
		return (altitude);
	}
	//------------------------------------
};
//--------------------------------
#endif	// BMP280_CPP
