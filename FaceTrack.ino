/* This file is part of the Razor AHRS Firmware */
//==============================================//

typedef struct
{
  int16_t  Begin  ;   // 2  Debut
  uint16_t Cpt ;      // 2  Compteur trame or Code info or error
  float    gyro[3];   // 12 [Y, P, R]    gyro
  float    acc[3];    // 12 [x, y, z]    Acc
  int16_t  End ;      // 2  Fin
} type_hat;

type_hat hat;

typedef struct
// Center
{
	float Matrix[3][3];
	float ZeroYaw = 0;
} tCenter;
tCenter Center;

float _Yaw    = 0;

#define centerYaw				0
#define centerYawPitch		1
#define centerYPR				2
//================================================
void FixCenter()
{
	float  M[3][3];
	float M0[3][3];
	float MM[3][3];
	Center.ZeroYaw = _Yaw + Center.ZeroYaw;
	Matrix_Multiply33(DCM_Matrix, Calibr.D.orient_matrix, MM); //a*b=c
  //----------
  if (Calibr.D.center_mode == centerYawPitch)							//Center Yaw,Pitch only
	{
    float half_roll = 0.5f * atan2(MM[2][1], MM[2][2]);	// half of roll;
    //---------- 
		IdentityMatrix(M);
    Matrix_Multiply33(M, Calibr.D.orient_matrix, MM);
    //---------- QuaternionFromAngleAxis
    float qw = cos(half_roll);
    float qx = sin(half_roll);
    //---------- QuaternionToMatrix
    float xx = qx * qx;
    float xw = qx * qw;
    IdentityMatrix(M);
    M[1][1] = 1.0f - 2.0f * xx;
    M[2][1] =      - 2.0f * xw;
    M[1][2] =        2.0f * xw;
    M[2][2] = 1.0f - 2.0f * xx;
    //----------
    Matrix_Multiply(MM, M, M0);
    Matrix_Multiply(DCM_Matrix, M0, MM);
  };  //if
  //----------

  //----------- Center_Matrix = MM' --------------
  for(int x=0; x<3; x++) for(int y=0; y<3; y++) Center.Matrix[x][y] = MM[y][x];
}
//================================================
void ResetCenterMatrix()
{
	IdentityMatrix(Center.Matrix);
	Center.ZeroYaw = 0;
}
//================================================
void FT_Setup()
{
  hat.Begin=0xAAAA;
  hat.Cpt=0;
  hat.End=0x5555;
}
//================================================
void FT_Data()
{
	float OutMat[3][3];
	float Temporary_Matrix[3][3];

	if (Calibr.D.center_mode == centerYaw)	// Center only on Yaw
	{
		Matrix_Multiply33(DCM_Matrix, Calibr.D.orient_matrix, OutMat); //a*b=c
		_Yaw = TO_DEG(atan2(OutMat[1][0], OutMat[0][0])) - Center.ZeroYaw;
		if      (_Yaw > +180) _Yaw = _Yaw - 360;
		else if (_Yaw < -180) _Yaw = _Yaw + 360;
	}
	else
	{																					// Center Yaw, Pitch, maybe Roll
		Matrix_Multiply(Center.Matrix,DCM_Matrix,Temporary_Matrix);
		Matrix_Multiply33(Temporary_Matrix, Calibr.D.orient_matrix, OutMat); //a*b=c
		_Yaw = TO_DEG(atan2(OutMat[1][0], OutMat[0][0]));
	}
	
  hat.gyro[0] = _Yaw;
  hat.gyro[1] = TO_DEG(-asin(OutMat[2][0]));									//_Pitch;
  hat.gyro[2] = TO_DEG(atan2(OutMat[2][1], OutMat[2][2]));		//_Roll;

  Serial.write((byte*)&hat,30);
  hat.Cpt++;  if (hat.Cpt>999) { hat.Cpt=0; };
}
//==============================================================================
