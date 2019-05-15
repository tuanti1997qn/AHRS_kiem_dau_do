/* This file is part of the Razor AHRS Firmware */

//**************************************************/
// DCM variables
float Gyro_Vector[3]    = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]   = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]        = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]        = {0, 0, 0}; // Omega Integrator
float Omega[3]          = {0, 0, 0};
float errorRollPitch[3];
float errorYaw[3];
/**************************************************/
// DCM algorithm
//==================================================
void Matrix_update(void)
{
  Gyro_Vector[0] = TO_RAD(gyro[0]);	//gyro x roll
  Gyro_Vector[1] = TO_RAD(gyro[1]);	//gyro y pitch
  Gyro_Vector[2] = TO_RAD(gyro[2]);	//gyro z yaw
  
  Vector_Add(Omega, Gyro_Vector, Omega_I);		//adding proportional term
  Vector_Add(Omega_Vector, Omega, Omega_P);		//adding Integrator term
  
	float Update_Matrix[3][3];
  Update_Matrix[0][0]= 0;
  Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
  Update_Matrix[0][2]= G_Dt*Omega_Vector[1];// y
  Update_Matrix[1][0]= G_Dt*Omega_Vector[2];// z
  Update_Matrix[1][1]= 0;
  Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
  Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
  Update_Matrix[2][1]= G_Dt*Omega_Vector[0];// x
  Update_Matrix[2][2]= 0;

	float Temporary_Matrix[3][3];
  Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c
  for(int x=0; x<3; x++)															//adding Integrator term
		Vector_Add(DCM_Matrix[x], DCM_Matrix[x], Temporary_Matrix[x]);
}
//==============================================================================
void Normalize(void)	//overload
{
  float error;
  float temporary[3][3];
  error= -Vector_Dot_Product(DCM_Matrix[0],DCM_Matrix[1])*.5;	//eq.19
  Vector_Scale(temporary[0], DCM_Matrix[1], error);						//eq.19
  Vector_Scale(temporary[1], DCM_Matrix[0], error);						//eq.19
  
  Vector_Add(temporary[0], temporary[0], DCM_Matrix[0]);				//eq.19
  Vector_Add(temporary[1], temporary[1], DCM_Matrix[1]);				//eq.19
  
  Vector_Cross_Product(temporary[2],temporary[0],temporary[1]); // c= a x b //eq.20
  
  float renorm;
  renorm= .5 *(3 - Vector_Dot_Product(temporary[0],temporary[0])); //eq.21
  Vector_Scale(DCM_Matrix[0], temporary[0], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(temporary[1],temporary[1])); //eq.21
  Vector_Scale(DCM_Matrix[1], temporary[1], renorm);
  
  renorm= .5 *(3 - Vector_Dot_Product(temporary[2],temporary[2])); //eq.21
  Vector_Scale(DCM_Matrix[2], temporary[2], renorm);
}

//==============================================================================
void Compass_Heading()
{
#if defined Kp_YAW
	#if defined Ki_YAW
	float	pitch;
	float	roll;
  float mag_x;
  float mag_y;
  float cos_roll;
  float sin_roll;
  float cos_pitch;
  float sin_pitch;
  
	pitch     = -asin(DCM_Matrix[2][0]);
	roll      = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
  sin_roll  = sin(roll);
  cos_roll  = cos(roll);
  sin_pitch = sin(pitch);
  cos_pitch = cos(pitch);

  // Tilt compensated magnetic field X
  mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
  // Tilt compensated magnetic field Y
  mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
  // Magnetic Heading
  MAG_Heading = atan2(-mag_y, mag_x);
	#endif
#endif
}
//==============================================================================
//Compensation the Roll, Pitch and Yaw drift.
void Drift_correction(void)
{
  //*****Roll and Pitch***************
	float Accel_weight = Weight(accel);

  Vector_Cross_Product(errorRollPitch,accel,DCM_Matrix[2]); //adjust the ground of reference
  Vector_Scale(Omega_P,errorRollPitch,Kp_ROLLPITCH*Accel_weight);
  
  static float Scaled_Omega_I[3];
  Vector_Scale(Scaled_Omega_I,errorRollPitch,Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
#if defined Kp_YAW
	#if defined Ki_YAW
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
  float mag_heading_x = cos(MAG_Heading);
  float mag_heading_y = sin(MAG_Heading);

  float errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,DCM_Matrix[2],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  float Scaled_Omega_P[3];
  Vector_Scale(Scaled_Omega_P,errorYaw,Kp_YAW);	//.01 proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);		//Adding  Proportional.
  
  Vector_Scale(Scaled_Omega_I,errorYaw,Ki_YAW);	//.00001 Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);		//adding integrator to the Omega_I
	#endif
#endif
}
//==============================================================================

