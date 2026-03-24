#ifndef YIS130_H
#define YIS130_H
#include <stdint.h>
#define MPU_CAN   hcan1   


typedef struct
{
    float acc[3];
		float acc_cali[3];
		double vel[3];
	  float gyro[3];         
      
	  
	  float PITCH;
	  float YAW; 
	  float ROLL;
	  float PITCH_ANGLE;
	  float YAW_ANGLE; 
	  float ROLL_ANGLE;
	  float PITCH_ANGLE_BEG;
	  float YAW_ANGLE_BEG;
	  float ROLL_ANGLE_BEG;
	  float PITCH_ANGLE_Del;
	  float YAW_ANGLE_Del;
	  float ROLL_ANGLE_Del;



		float quat[4];
	  float ACCX_CALI;
		float ACCY_CALI;
		float ACCZ_CALI;
	float ACCX_FILTER;
	float ACCY_FILTER;
	  int cali ;
	
	 // DATA FROM ENCODER
	 float REAL_X;
	 float REAL_Y;
	 float REAL_YAW;
	 float REAL_YAW_SET;
	 float REAL_YAW_MARK;
	 float X_tt;
	 float Y_tt;
	
} MPU_DATA;




void CAN_CMD_ENCODER();


void can_filter_init(void);

void get_mpu_measure(MPU_DATA *ptr,uint8_t *data ) ;

extern MPU_DATA mpu_data[4];
extern float output_vector_data[3];

void VECTOR_CONVERT();

void SelfCalibration();

void DATARELOAD(uint8_t * arr);

extern float ACCX,ACCY,ACCZ; // 锟斤拷叩锟叫Ｗ硷拷锟斤拷锟斤拷要锟较筹拷锟斤拷锟杰诧拷锟斤拷
void arm_fir_f32_lp(void);

#endif
		AS5->total_angle += diff;
		AS5->delta_dis = diff;
	
	} else if (diff < 0) {
		AS5->total_angle += diff;
		AS5->delta_dis = diff;
	}	
	AS5->last_angle = AS5->angle;


}

	// cc direction & circle
//	int diff = AS5->angle - AS5->last_angle;
//	if (diff > 10000) {
//		AS5->cirle--;
//		AS5->total_angle = AS5->angle + AS5->cirle * 16384;
//		
//	} else if (diff < -10000) {
//		AS5->cirle++;
//		AS5->total_angle = AS5->angle + AS5->cirle * 16384;
//		
//	}else if (diff > 0) {
//		AS5->total_angle += diff;
//	
//	} else if (diff < 0) {
//		AS5->total_angle += diff;
//	
//		
//	}


	

