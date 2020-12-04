#include "headfile.h"

#define Mat         arm_matrix_instance_f32 
#define Mat_Init    arm_mat_init_f32
#define Mat_Add     arm_mat_add_f32
#define Mat_Sub     arm_mat_sub_f32
#define Mat_Mult    arm_mat_mult_f32
#define Mat_Trans   arm_mat_trans_f32
#define Mat_Inv     arm_mat_inverse_f32

typedef struct _KF
{
  Mat X,_X,A,AT,Z,H,HT,Q,R,P,_P,K; 
} _KF,*_p_KF;

