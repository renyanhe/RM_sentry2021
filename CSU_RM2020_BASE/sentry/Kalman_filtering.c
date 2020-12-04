#include "Kalman_filtering.h"

_KF _KF_Mat;

float Zero41[4] = {0, 0, 0, 0};
float Zero44[16] = {0};

float A_data[16]={1, 0, 0.001, 0,
                  0, 1, 0, 0.001,
                  0, 0, 1, 0,
                  0, 0, 0, 1};

float H_data[16]={1, 0, 0, 0,
                  0, 1, 0, 0,
                  0, 0, 1, 0,
                  0, 0, 0, 1};

float Q_data[16]={0.01, 0, 0, 0,
                  0, 0.01, 0, 0,
                  0, 0, 0.01, 0,
                  0, 0, 0, 0.01};

float R_data[16];

void Matrix_Init()
{
   Mat_Init(&_KF_Mat.X,4,1,(float *)Zero41);
	 Mat_Init(&_KF_Mat._X,4,1,(float *)Zero41);
	 Mat_Init(&_KF_Mat.Z,4,1,(float *)Zero41);
	
   Mat_Init(&_KF_Mat.A,4,4,(float *)A_data); //×´Ì¬×ªÒÆ¾ØÕó
	 Mat_Init(&_KF_Mat.AT,4,4,(float *)A_data);
	 Mat_Trans(&_KF_Mat.A, &_KF_Mat.AT);
	
	 Mat_Init(&_KF_Mat.H,4,4,(float *)H_data); //¹Û²â×ªÒÆ¾ØÕó
	 Mat_Init(&_KF_Mat.HT,4,4,(float *)H_data);
	 Mat_Trans(&_KF_Mat.H, &_KF_Mat.HT);
	
	 Mat_Init(&_KF_Mat.Q,4,4,(float *)Q_data);
	 Mat_Init(&_KF_Mat.R,4,4,(float *)R_data);
	 
	 Mat_Init(&_KF_Mat.P,4,4,(float *)Zero44);
	 Mat_Init(&_KF_Mat._P,4,4,(float *)Zero44);
	
	 Mat_Init(&_KF_Mat.K,4,4,(float *)Zero44);
}

void KF_Calc(_KF *F,float signal1, float signal2,float signal3, float signal4)
{
	 float TEMP_data[16] = {0};
   float TEMP_data41[4] = {0};
   mat TEMP,TEMP41;

   Mat_Init(&TEMP,4,4,(float *)TEMP_data);
   Mat_Init(&TEMP41,4,1,(float *)TEMP_data41);
	 
	 F->Z.pData[0] = signal1;
   F->Z.pData[1] = signal2;
   F->Z.pData[2] = signal3;
   F->Z.pData[3] = signal4;	 
	 
     //1. xhat'(k)= A xhat(k-1)
   Mat_Mult(&F->A, &F->X, &F->_X);

     //2. P'(k) = A P(k-1) AT + Q
   Mat_Mult(&F->A, &F->P, &F->_P);
   Mat_Mult(&F->P, &F->AT, &TEMP);
   Mat_Add(&TEMP, &F->Q, &F->_P);

     //3. K(k) = P'(k) HT / (H P'(k) HT + R)
   Mat_Mult(&F->H, &F->_P, &F->K);
   Mat_Mult(&F->K, &F->HT, &TEMP);
   Mat_Add(&TEMP, &F->R, &F->K);
   Mat_Inv(&F->K, &F->_P);
   Mat_Mult(&F->_P, &F->HT, &TEMP);
   Mat_Mult(&TEMP, &F->_P, &F->K);

     //4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
   Mat_Mult(&F->H, &F->_X, &TEMP41);
   Mat_Sub(&F->Z, &TEMP41, &F->X);
   Mat_Mult(&F->K, &F->X, &TEMP41);
   Mat_Add(&F->_X, &TEMP41, &F->X);

     //5. P(k) = (1-K(k)H)P'(k)
   Mat_Mult(&F->K, &F->H, &F->P);
   Mat_Sub(&F->Q, &F->P, &TEMP);
   Mat_Mult(&TEMP, &F->_P, &F->P);
}



