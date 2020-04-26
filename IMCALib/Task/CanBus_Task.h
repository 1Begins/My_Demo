#ifndef __CANBUS_TASK
#define __CANBUS_TASK


#include "mytype.h"
#include "can.h"
#include "pid.h"


#define FILTER_BUF_LEN		5

/*CAN2 7���������*/
enum
{
	/*CAN2��IDǰ��λΪ���̵��������λΪyaw��pluck*/
	CAN2_CHASSIS_MOTOR_ID = 4,
	CAN2_GIMBAL_MOTOR_SD  = 5,
};


/*����CAN���ͻ��ǽ��յ�ID*/
typedef enum
{
	//can2���ID
	CAN_3508Moto1_ID  		 = 0x201,
	CAN_3508Moto2_ID  		 = 0x202,
	CAN_3508Moto3_ID  		 = 0x203,
	CAN_3508Moto4_ID 		   = 0x204,
	CAN_YAW_Motor_ID   		 = 0x205,
	CAN_PLUCK_Motor_ID     = 0x206,

	//can1���ID	
	CAN_PUSHROD_Motor_ID   = 0x205,
	CAN_PITCH_Motor_ID     = 0x206,	
	CAN_FRICTION1_Motor_ID = 0x207,
	CAN_FRICTION2_Motor_ID = 0x208,
	
}CAN_Message_ID;


/*���յ�����̨����Ĳ����ṹ��*/
typedef struct{
	
	uint16_t	angle_bias;
//	float  		ecd_angle_bias; 	//��ʼ�Ƕ�
	int16_t	 	speed_rpm;				//���ٶ�
	float  	  real_current;
	int16_t  	given_current;
	uint8_t  	hall;
	uint16_t 	angle;			//abs angle range:[0,8191]
	uint16_t 	last_angle;	  	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8				buf_idx;
	u16				angle_buf[FILTER_BUF_LEN];
	u16				fited_angle;
	u32				msg_cnt;
//	float			ecd_angle;		//ת���ĽǶ�
//	float 		angular_speed;	//���ٶ�
	
}moto_measure_t;


extern moto_measure_t  Chassis_Motor_Encoder[];  //���̵�������ṹ��<can2>
extern moto_measure_t  Gimbal_Motor_Encoder[];	 //��̨��������ṹ��<can1>



/*CAN�����������ú�CAN�Ŀ���*/
void CANFilterInit(void);
/*��õ���Ļ�е�Ƕ�*/
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*����3508���ͨ��CAN����������Ϣ*/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[]);
/*���͵������Ϣ��CAN�����ϣ��˺�������һ·CAN��ǰ4������Ŀ���*/
void set_moto_current1(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*���͵������Ϣ��CAN�����ϣ��˺�������һ·CAN�ĺ�4������Ŀ���*/
void set_moto_current2(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4);
/*��ع����Ƕȿ���*/
void position_send_out(CAN_HandleTypeDef *hcan, int32_t angleControl);

int16_t GetEncoderRelativeAngle(void);
int16_t GetEncoderOffsetRelativeAngle(void);

#endif

