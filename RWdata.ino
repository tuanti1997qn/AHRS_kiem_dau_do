/* This file is part of the Razor AHRS Firmware */
//================================================
#include <EEPROM.h>

#define CSkey							(0xAB)
#define CalibrSize 				( 127)
#define AdrCalibr					( 128 ) 
#define AdrCenter					(AdrCalibr + CalibrSize) 
#define TimeOut						( 500)
//===============================================
byte IsEEpromError()
{
	byte CS = CSkey;
	for (int i = 0; i < (CalibrSize-1); i++) CS += EEPROM[AdrCalibr + i];
	return(CS != EEPROM[AdrCalibr + CalibrSize - 1]);
}
//-----------------------------------------------
void CheckEEprom()
{
	Serial.write(IsEEpromError());
}
//-----------------------------------------------
void EEprom2RAM()
{
	if (!IsEEpromError()) EEPROM.get(AdrCalibr, Calibr);
}
//-----------------------------------------------
void EEprom2Serial()
{
	EEprom2RAM();
	Serial.write(Calibr.B, CalibrSize);
}
//-----------------------------------------------
void Serial2EEprom()
{
	for (int i = 0; i < CalibrSize; i++)
	{
		unsigned long time0 = millis();
		while (Serial.available() == 0) if ((millis() - time0) >= TimeOut) goto RX_error;
		Calibr.B[i] = Serial.read();
	};
	EEPROM.put(AdrCalibr, Calibr);
	CheckEEprom();
	return;

RX_error:
	Serial.write(CSkey);
}
//-----------------------------------------------
void EraseEEprom()
{
	for (byte i = 0; i < (AdrCenter + sizeof(Center)); i++)
		EEPROM.put(i,  0xFF);
	Serial.write(CSkey);
}
//-----------------------------------------------
void SaveCenter()
{
	EEPROM.put(AdrCenter,  Center);
}
//-----------------------------------------------
void LoadCenter()
{
	EEPROM.get(AdrCenter,  Center);
}
//===============================================
