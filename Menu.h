/*   This file is part of the Razor AHRS Firmware    */

/*============== SENSOR SELECTOR =====================\
| 1 | SparkFun "9DOF Razor IMU" version "SEN-10125"   | https://www.sparkfun.com/products/retired/10125
|   | (HMC5843 magnetometer)                          | (not tested)
|---|-------------------------------------------------|
| 2 | (This variant for GY-85 sensor)                 | http://www.thaieasyelec.com/downloads/ESEN237/GY85_USG.pdf
|   | (HMC5883L magnetometer)                         | (good working!)
|---|-------------------------------------------------|
| 3 | SparkFun "9DOF Sensor Stick" version "SEN-10183"| https://www.sparkfun.com/products/retired/10183
|   | (HMC5843 magnetometer)                          | (not tested)
|---|-------------------------------------------------|
| 4 | SparkFun "9DOF Sensor Stick" version "SEN-10321"| https://www.sparkfun.com/products/retired/10321
|   | (HMC5843 magnetometer):                         | (not tested)
|---|-------------------------------------------------|
| 5 | SparkFun "9DOF Sensor Stick" version "SEN-10724"| https://www.sparkfun.com/products/retired/10724
|   | (HMC5883L magnetometer)                         | (not tested)
|---|-------------------------------------------------|
| 6 | GY-521 (no one magnetometer)                    | http://www.thaieasyelec.com/downloads/ESEN247/GY521_USG.pdf
|   |        (driftage on Yaw)  :(                    | not recommended, driftage on Yaw
|---|-------------------------------------------------|
| 7 | GY521 & GY271 (MPU6050 + HMC5883L)              | http://www.thaieasyelec.com/downloads/ESEN247/GY521_USG.pdf
|   |        (good working)                           | (good working!)
|---|-------------------------------------------------|
| 8 | GY-9250 (MPU6500 + AK8963)                      | https://sites.google.com/site/hardzet/home/arduino/gy-9250-mpu-9250
|   |        (good working)                           | (good working!)
|---|-------------------------------------------------|
| 9 | GY-85 sensor (QMC5883 magnetometer)             | http://www.thaieasyelec.com/downloads/ESEN237/GY85_USG.pdf
|   |              (tested by Charas04)               | (good working!)
|---|-------------------------------------------------|
|10 | GY-91 sensor (MPU6500, AK8963, BMP280)          | https://yandex.ru/images/search?text=gy-91&stype=image&lr=51&source=wiz
|   |        (good working)                           | (good working!)
|-----------------------------------------------------|
| In next string select Your SensorVariant (1..10):   |
| --------------------------------------------------- |
| (В следующей строке выберите номер                  |
|  Вашего варианта сенсора (1..10):)                  |
\----------------------------------------------------*/
#define SensorVariant 9

/*============== BLUETOOTH SELECTOR ==================\
|       if You have a BlueTooth Module attached       |
|            change 0 to 1 in next string:            |
|       ---------------------------------------       |
|       (Если Вы подключили BlueTooth модуль,         |
|        замените 0 на 1 в следующей строке:)         |
\----------------------------------------------------*/
#define OUTPUT__HAS_RN_BLUETOOTH 0




//void Matrix_update(void);
//void Normalize(void);  //overload;
//void Compass_Heading();
//void Drift_correction(void);
//
//void FixCenter();
//void ResetCenterMatrix();
//void FT_Setup();
//void FT_Data();
//
//float Vector_Dot_Product(const float v1[3], const float v2[3]);
//void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
//void Vector_Scale(float out[3], const float v[3], float scale);
//void Vector_Add(float out[3], const float v1[3], const float v2[3]);
//void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);  //override;
//void Matrix_Multiply33(const float a[3][3], const char b[3][3], float out[3][3]);  //override;
//void IdentityMatrix(float M[3][3]);
//float Len(float V[3]);
//float  Weight(const float V[3]);
//void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
//void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
