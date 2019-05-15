/* This file is part of the Razor AHRS Firmware */
//==============================================//
#define Sinchro ("#")
void TXsinchro(char Mode)
{
	Serial.print(Sinchro);
	Serial.print(Mode);
}
// Output angles: yaw, pitch, roll
void output_angles()
{
	if (output_format == OUTPUT__FORMAT_FACETRACK)
	{
		FT_Data();
	}
  else
	{
		float OutMat[3][3];
		float ypr[3];
		Matrix_Multiply33(DCM_Matrix, Calibr.D.orient_matrix, OutMat); //a*b=c
		ypr[0] = TO_DEG(atan2(OutMat[1][0], OutMat[0][0]));	//yaw
		ypr[1] = TO_DEG(-asin(OutMat[2][0]));									//pitch
		ypr[2] = TO_DEG(atan2(OutMat[2][1], OutMat[2][2]));	//roll
		if (output_format == OUTPUT__FORMAT_BINARY)
		{
			TXsinchro('A');
			Serial.write((byte*) ypr, 12);
		}
		else if (output_format == OUTPUT__FORMAT_TEXT)
		{
			Serial.print("#YPR=");
			Serial.print(ypr[0]); Serial.print(",");
			Serial.print(ypr[1]); Serial.print(",");
			Serial.print(ypr[2]); Serial.println();
		}
	}
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    Serial.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      Serial.print(accel_min[i]);
      Serial.print("/");
      Serial.print(accel_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      Serial.print(magnetom_min[i]);
      Serial.print("/");
      Serial.print(magnetom_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print("/");
      Serial.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void output_sensors_binary(char Mode)
{
	TXsinchro(Mode);
  Serial.write((byte*) accel, 12);
  Serial.write((byte*) magnetom, 12);
  Serial.write((byte*) gyro, 12);
  Serial.write((byte*) &G_Dt, 4);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary('R');
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary('C');
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary('R');
      compensate_sensor_errors();
      output_sensors_binary('C');
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}

void output_float(float T)
{
	if (output_format == OUTPUT__FORMAT_TEXT)
  {
		Serial.println(T);							// text
  }
	else
	{
		Serial.write((byte*) &T, 4);		// binary
	}
}
