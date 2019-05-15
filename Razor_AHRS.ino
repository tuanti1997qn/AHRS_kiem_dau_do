/***************************************************************************************************************
* Razor AHRS Firmware version [20180306]

/=========================================================================\
| for update and for more information visit:                              |
| https://sites.google.com/site/diyheadtracking/                          |
| Email: go1@list.ru    GO63 Samara Russia 2018                           |
\=========================================================================/

* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*     * v1.4.1
*       * Added output modes to read raw and/or calibrated sensor data in text or binary format.
*       * Added static magnetometer soft iron distortion compensation
*     * v1.4.2
*       * (No core firmware changes)
*
* TODOs:
*   * Use self-test and temperature-compensation features of the sensors.
***************************************************************************************************************/
/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:
  
  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
  
      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.
      
      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      "#of" - Output angles in FaceTrack format              
      
      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.
      
      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
      
      // Error message output        
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.
    
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
         
         
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.
          
          
  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)
  
  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
  
  The status LED will be on if streaming output is enabled and off otherwise.
  
  Byte order of binary output is little-endian: least significant byte comes first.
*/

/*****************************************************************/
#define SketchVersion 20180306
#define DEVELOP 1
#include "Menu.h"
/*****************************************************************/
#if   SensorVariant == 1
	#include "_SEN-10125.h"
	SEN_10125 	Sensor;
#elif SensorVariant == 2
	#include "_GY-85-hmc.h"
	GY85      	Sensor;
#elif SensorVariant == 3
	#include "_SEN-10321.h"
	SEN_10321 	Sensor;
#elif SensorVariant == 4
	#include "_SEN-10321.h"
	SEN_10321 	Sensor;
#elif SensorVariant == 5
	#include "_SEN-10724.h"
	SEN_10724 	Sensor;
#elif SensorVariant == 6
	#include "_GY-521.h"
	GY521     	Sensor;
#elif SensorVariant == 7
	#include "_GY521-GY271.h"
	GY521_GY271	Sensor;
#elif SensorVariant == 8
	#include "_GY-9250.h"
	MPU9250   	Sensor;
#elif SensorVariant == 9
	#include "_GY-85-QMC.h"
	GY85QMC   	Sensor;
#elif SensorVariant == 10
	#include "_GY-91.h"
	GY91   			Sensor;
#else
  #error YOU MUST TO SELECT THE SensorVariant FROM 1 TO 10! See "SENSOR SELECTOR" at top of "MENU.h" !
#endif
//#ifdef STM32
//#define Serial Serial2
//HardwareSerial Serial2(USART2);
//#endif

// OUTPUT OPTIONS
/*****************************************************************/
// Dont change serial port baud rate for working of Razor_Ahrs_All_in_1!
#define OUTPUT__BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20000UL  // in microseconds
//#define OUTPUT__DATA_INTERVAL 10000UL  // in microseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 	0	// Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 						1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 			2	// Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 				3	// Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 			4 // Outputs calibrated AND raw sensor values for all 9 axes
#define OUTPUT__MY_CUSTOM_MODE          5 // Outputs I defined
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 						0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 					1 // Outputs data as binary float
#define OUTPUT__FORMAT_FACETRACK 				3 // Outputs data to facetrack

// Dont change startup output mode and format for working of OpenTrack!
int output_mode	 = OUTPUT__MY_CUSTOM_MODE;
int output_format = OUTPUT__FORMAT_TEXT;
//================================================================/
// Bluetooth options
/*****************************************************************/
// if defined OUTPUT__HAS_RN_BLUETOOTH = 1
// and Arduino has chip ATmega32U4,
// output data will flows through Serial1,
// for correct working Bluetooth unit
//================================================================/
#if OUTPUT__HAS_RN_BLUETOOTH != 0
	#if defined (__AVR_ATmega32U4__)
		#define Serial Serial1
	#endif
#endif
/*****************************************************************/
#include <Wire.h>
//======================================
#define STATUS_LED_PIN 13	// Pin number of status LED
#define TO_RAD(x) (x * (PI / 180.0))
#define TO_DEG(x) (x * (180.0 / PI))
//================================================
#define none			0
#define standard	1
#define extended	2

struct tCalibr
{
	byte	YY,MM,DD,hh,mm;									//  5
	byte	mag_mode;													//  1
	float mag_ellipsoid_center[3];				// 12
	float mag_ellipsoid_transform[3][3];	// 36
	byte	acc_mode;													//  1
	float acc_ellipsoid_center[3];				// 12
	float acc_ellipsoid_transform[3][3];	// 36
	byte	gyr_mode;													//  1
	float gyr_center[3];										// 12
	byte	center_mode;											//  1
	char	orient_matrix[3][3];							//	9
	byte  CS;																//  1						
};				 																//= 127 bytes
union tUnionCalibr
{
	tCalibr D;
	byte B[sizeof(tCalibr)];
};
//======================================
tUnionCalibr Calibr {
	// -- YY/MM/DD hh:mm: --------------------
			0x18, 0x02, 0x10, 0x20, 0x20,
	// -- magmode: ---------------------------
			extended,
	// -- mag_ellipsoid_center[3]: -----------
			0, 0, 0,
	// -- mag_ellipsoid_transform[3][3]: -----
		  1,  0,  0,
			0,  1,  0,
			0,  0,  1,
	// -- accmode: ---------------------------
			standard,
	// -- acc_ellipsoid_center[3]: -----------
			0, 0, 0,
	// -- acc_ellipsoid_transform[3][3]: -----
		  1,  0,  0,
			0,  1,  0,
			0,  0,  1,
	// -- gyrmode: ---------------------------
			standard,
	// -- gyr_center[3]: ---------------------
			0, 0, 0,
	// -- center_mode: -----------------------
			standard,
	// -- orient_matrix[3][3]: ---------------
			1, 0, 0,
			0, 1, 0,
			0, 0, 1,
	// -- CS: --------------------------------
			0x97 };	// Control Sum
//======================================
float& mag_x_scale = Calibr.D.mag_ellipsoid_transform[0][0];
float& mag_y_scale = Calibr.D.mag_ellipsoid_transform[1][1];
float& mag_z_scale = Calibr.D.mag_ellipsoid_transform[2][2];
//======================================

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];
float accel_tmp[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int   gyro_num_samples = 0;

float MAG_Heading;
float DCM_Matrix[3][3];

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors  = 0;
int num_gyro_errors  = 0;

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

	Sensor.Read(gyro,accel,magnetom);
	compensate_sensor_errors();	// Apply sensor calibration
  timestamp = micros();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // Init rotation matrix without yaw
  init_rotation_matrix(DCM_Matrix, 0,   pitch, roll);
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  // Init rotation matrix with yaw
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to this sensor
void compensate(float vector[3], const byte mode, const float center[3], const float matrix[3][3])
{
	float tmp[3];
	if (mode == extended) {
    for (int i = 0; i < 3; i++) tmp[i] = vector[i] - center[i];
    Matrix_Vector_Multiply(matrix, tmp, vector);
	} else if (mode == standard) {
    for (int i = 0; i < 3; i++) vector[i] = (vector[i] - center[i]) * matrix[i][i];
	}
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  // ----- Compensate magnetometer error ------
	compensate(magnetom, Calibr.D.mag_mode, Calibr.D.mag_ellipsoid_center, Calibr.D.mag_ellipsoid_transform);
  // ----- Compensate accelerometer error -----
	compensate(accel, Calibr.D.acc_mode, Calibr.D.acc_ellipsoid_center, Calibr.D.acc_ellipsoid_transform);
	// ----- Compensate gyroscope error ---------
	if (Calibr.D.gyr_mode != none) for (int i = 0; i < 3; i++) gyro[i] -= Calibr.D.gyr_center[i];
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);	//light when working
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);	//dark when stoped
}

// Blocks for 500 mS until another byte is available on serial port
char readChar()
{
	long time0 = millis();
	while (Serial.available() == 0) if ((millis() - time0) >= 500) return(0xFF);
	return(Serial.read());
}

void setup()
{
	//EEprom2Serial();
	EEprom2RAM();	

  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);

  // Init sensors
  delay(50);  // Give sensors enough time to start
	Sensor.Init();
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
	IdentityMatrix(DCM_Matrix);
	LoadCenter();
  reset_sensor_fusion();

  // Modified by Vitaly "MegaMozg" Naidentsev and "GO63" for "Razor_AHRS"+"FaceTrack" compatibility
  FT_Setup();
	
  // Init output
  turn_output_stream_on();

	while (!Serial); // Needed only for built-in USB ports.
}

// Main loop
void loop()
{
  // Read incoming control messages

          
  if (Serial.available())
  {
    if (readChar() == '#') 							// Start of new control message
    {
      int command = readChar();	 				// Commands
      if (command == 'f') 								// request one output _f_rame
        output_single_on = true;
      else if (command == 's') 					// _s_ynch request
      {
        // Read ID
        byte id[2];
        id[0] = readChar();
        id[1] = readChar();
        
        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
      else if (command == 'o') 					// Set _o_utput mode
      {
        char output_param = readChar();
        if (output_param == 'n')  				// Calibrate _n_ext sensor
        {
          curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 't') 		// Output angles as _t_ext
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_TEXT;
        }
        else if (output_param == 'b') 		// Output angles in _b_inary format
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_BINARY;
        }
        // Modified by Vitaly "MegaMozg" Naidentsev and "GO63" for "Razor_AHRS"+"FaceTrack" compatibility
        else if (output_param == 'f') 		// Output angles in FACETRACK format
        {
          output_mode = OUTPUT__MODE_ANGLES;
          output_format = OUTPUT__FORMAT_FACETRACK;
        }        
        else if (output_param == 'c') 		// Go to _c_alibration mode
        {
          output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
          reset_calibration_session_flag = true;
        }
        else if (output_param == 's') 		// Output _s_ensor values
        {
          char values_param = readChar();
          char format_param = readChar();
          if (values_param == 'r')  			// Output _r_aw sensor values
            output_mode = OUTPUT__MODE_SENSORS_RAW;
          else if (values_param == 'c')	// Output _c_alibrated sensor values
            output_mode = OUTPUT__MODE_SENSORS_CALIB;
          else if (values_param == 'b')	// Output _b_oth sensor values (raw and calibrated)
            output_mode = OUTPUT__MODE_SENSORS_BOTH;

          if (format_param == 't') 			// Output values as _t_text
            output_format = OUTPUT__FORMAT_TEXT;
          else if (format_param == 'b') 	// Output values in _b_inary format
            output_format = OUTPUT__FORMAT_BINARY;
        }
        else if (output_param == '0') 		// Disable continuous streaming output
        {
          turn_output_stream_off();
          reset_calibration_session_flag = true;
        }
        else if (output_param == '1') 		// Enable continuous streaming output
        {
          reset_calibration_session_flag = true;
          turn_output_stream_on();
        }
				else if (output_param == 'z')		// #oz center YPR for FaceTrack mode
					FixCenter();
				else if (output_param == 'r')		// #or reset  YPR for FaceTrack mode
					ResetCenterMatrix();
        else if(output_param == 'm') 
        {
          int output_mode   = OUTPUT__MY_CUSTOM_MODE;
        }
      }
      else if (command == '?')
      {
        char param = readChar();
        if (param == 'v')								// #?v sensor variant and sketch version 
				{
					#if SensorVariant < 16
						Serial.print(0);
					#endif					
					Serial.print((byte)SensorVariant, HEX);
					Serial.print(SketchVersion);
				}

				#if DEVELOP != 0
				else if (param == 't')						// #?t read temperature
				{
					float Temperature = Sensor.ReadTemperature();
					output_float(Temperature);
				}
				else if (param == 'p')						// #?p read pressure
				{
					float Pressure = Sensor.ReadPressure();
					output_float(Pressure);
				}
				else if (param == 'a')						// #?a Check I2C
				{
					byte I2Cadr = readChar();
					Serial.write(Sensor.I2Ccheck(I2Cadr));
				}
				else if (param == 'r')						// #?r Read I2C [1..16 bytes]
				{
					byte Buf[16];
					byte I2Cadr = readChar();
					byte Adr    = readChar();
					byte Num    = readChar();
					Num = Sensor.I2Cread(I2Cadr, Adr, Num, Buf);
					Serial.write(Buf, Num);
				}
				else if (param == 'w')						// #?w Write I2C
				{
					byte I2Cadr = readChar();
					byte Adr    = readChar();
					byte Data   = readChar();
					Serial.write(Sensor.I2Cwrite(I2Cadr, Adr, Data));
				}
				#endif

			}
      else if (command == 'e')						// Read/Write eeprom data
      {
        char eeprom_command = readChar();
        if      (eeprom_command == 'w') Serial2EEprom();		//#ew...
				else if (eeprom_command == 'r') EEprom2Serial();		//#er
				else if (eeprom_command == 'c') CheckEEprom();			//#ec
				else if (eeprom_command == 'z') SaveCenter();				//#ez
				else if (eeprom_command == 'e') EraseEEprom();			//#ee
			}
      else if (command == 'w')						// #w write data to Calibr.B
      {
        char adr = readChar();		//address of begin
        char len = readChar();		//length
				for(int i = 0; i < len; i++) Calibr.B[adr + i] = readChar();
			}
      else if (command == 'Z')						// Select type of center_mode #Z0..#Z2
      {
        Calibr.D.center_mode = (readChar() & 0x03);
			}
			#if OUTPUT__HAS_RN_BLUETOOTH == 1
      // Read messages from bluetooth module
      // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
      else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
        turn_output_stream_on();
      else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
        turn_output_stream_off();
			#endif // OUTPUT__HAS_RN_BLUETOOTH == 1
    }
  }
	//-----------------------------------------------------------------------

	//-----------------------------------------------------------------------
  // Time to read the sensors again?
  if((micros() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = micros();
		G_Dt = (float) (timestamp - timestamp_old) * 1.0e-6; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    // Update sensor readings
		Sensor.Read(gyro,accel,magnetom);
    if (output_mode == OUTPUT__MY_CUSTOM_MODE) 
    {
        //digitalWrite(STATUS_LED_PIN, HIGH);               //light when working
        // Apply sensor calibration
        compensate_sensor_errors();
        // Run DCM algorithm
        Matrix_update();
        Normalize();
        Compass_Heading();                                // Calculate magnetic heading
        Drift_correction();

        my_output_angles();
    }
    else if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)		// We're in calibration mode
    {
      check_reset_calibration_session();									// Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)					// Output angles
    {
			//digitalWrite(STATUS_LED_PIN, HIGH);								//light when working
				// Apply sensor calibration
				compensate_sensor_errors();
				// Run DCM algorithm
				Matrix_update();
				Normalize();
				Compass_Heading();																// Calculate magnetic heading
				Drift_correction();
       
				if (output_stream_on || output_single_on) output_angles();
//			//digitalWrite(STATUS_LED_PIN, LOW);								//dark when work done
    }
    else 																									// Output sensor values
    {      
      if (output_stream_on || output_single_on) output_sensors();
    }
    output_single_on = false;
  }
}
