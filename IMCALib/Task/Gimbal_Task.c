#include "Gimbal_Task.h"
#include "RC_Task.h"
#include "pid.h"
#include "CanBus_Task.h"
#include "PrintfInfo.h"
#include "Vision_interact.h"
#include "Jscope_Task.h"
#include "Limit_Switch.h"
#include "user_lib.h"
#include "mytype.h"
#include "IMU_Task.h"


uint8_t Gimbal_Init_Flag;  //��̨��ʼ����ʶ

pid_t Gimbal_Motor_pid_Spd[6] = {0};  //��̨���PID�ṹ��
pid_t Gimbal_Motor_pid_Pos[6] = {0};

gimbal_t PluckData;
gimbal_t GimbalData[2];  //��̨�ṹ��

/*����PID����*/
pid_t Gimbal_Pid_Vision_Pos[2];
pid_t Gimbal_Pid_Vision_Spd[2];

gimbal_control_t gimbal_control;
gimbal_behaviour_e gimbal_behaviour;

int32_t friction_speed_set = 0;
int32_t PluckUP_angel_set = 0;


float angular_speed1,angular_speed2;	

void Gimbal_PID_Init()
{	
	/*�ṹ����������*/
	memset(Gimbal_Motor_pid_Spd, 0, sizeof(pid_t)*6);
	memset(Gimbal_Motor_pid_Pos, 0, sizeof(pid_t)*6);

	//Ħ����3508���PID��ʼ��
	PID_struct_init(&Gimbal_Motor_pid_Spd[FRICTION1_CAN1_207], POSITION_PID, 6000, 2000, 8.3f, 0.002f, 0.0f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[FRICTION2_CAN1_208], POSITION_PID, 6000, 2000, 8.3f, 0.002f, 0.0f);

	//3508�������PID��ʼ��		3508����PLUCK���PID��ʼ��,����PID������Ŀ��ת��С��150rpm
	PID_struct_init(&Gimbal_Motor_pid_Pos[PLUCK_CAN2_206], POSITION_PID, 100, 2000, 0.6f, 0.0f, 1.0f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[PLUCK_CAN2_206], POSITION_PID, 16000, 10000, 175.0f, 0.8f, 0.0f);
	
	//�Ƹ�2006���PID��ʼ��
	PID_struct_init(&Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205], POSITION_PID, 500, 10, 0.22f, 0.0f, 0.1f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205], POSITION_PID, 10000, 2000, 120.0f, 0.05f, 1.0f);
	
	//YAW����PID��ʼ��  
	PID_struct_init(&Gimbal_Motor_pid_Pos[YAW_CAN2_205], POSITION_PID, 100, 2000, 0.6f, 0.0f, 1.0f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[YAW_CAN2_205], POSITION_PID, 5000, 3000, 175.0f, 0.8f, 0.0f);//16000 10000
	
	//PITCH����PID��ʼ��
	PID_struct_init(&Gimbal_Motor_pid_Pos[PITCH_CAN1_206], POSITION_PID, 7, 20, 0.015f, 0.0f, 0.2f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[PITCH_CAN1_206], POSITION_PID, 20000, 15000, 13.2f, 7.5f, 7.0f);
	
	/*Yaw���λ�û��ٶȻ�����λ�û���ʼ��PID����*/
	PID_struct_init(&Gimbal_Pid_Vision_Pos[0], POSITION_PID, 100, 2000, 12.0f, 0.0f, 0.0f);		//��ʼ��λ�û�PID����
	PID_struct_init(&Gimbal_Pid_Vision_Spd[0], POSITION_PID, 15000, 5000, 175.0f, 0.8f, 0.0f); 		//��ʼ���ٶȻ�PID���� 
	
	
}

void GimbalControl(void)
{
	
	/*��̨״̬�л���־λ(��ֹ����ģʽ�л�ң��ģʽ����)*/
	static bool gimbal_mode_flag = 0;  
	
	switch(remote_control.switch_left)
	{
		/*��̨ң��ģʽ*/
		case Switch_Up:		
		case Switch_Middle:
		{
			if( gimbal_mode_flag == 1 )
			{
				gimbal_mode_flag = 0;
				GimbalData[YAW].angle_set = Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle/19.2f;		//����ģʽ�л���ң��ģʽ�����yaw�趨ֵλ��ǰֵ
			}
			
			gimbal_behaviour = GIMBAL_Manaul_control;
			GimbalRC();			//yaw��pitch�����ݸ���
			break;
		}
		
		/*��̨����ģʽ����ʱֻ��pitch��������*/
		case Switch_Down:
		{
			gimbal_mode_flag = 1;
			gimbal_behaviour = GIMBAL_Auto_aiming;
      GimbalAuto();		//yaw��pitch�����ݸ���
			break;	
		}
		
	}	
	
	/*������ݸ��£�����ģʽ���Զ�����Ħ���֣�*/
	ShootControlSetValue();		
	GimbalPidCalc();
	GimbalPidOutput();
}

/*��̨��ʼ��������*/
void GimbalInit(void)
{
    static int32_t relative_angle;
    
    if(Gimbal_Motor_Encoder[YAW_CAN2_205].msg_cnt > 60)
    {
        sin_ramp_state.sin_ramp_switch = 1;
        sin_ramp_state.compare_value_up = 0;
        sin_ramp_state.compare_value_dowm = 0;
        
        GimbalData[YAW].offset_relative_angle = GetEncoderOffsetRelativeAngle(); /*�õ��ϵ�ʱ����������Ի�е�Ƕ�ֵ*/
        
    }
    
		/*
		���庯��ʱ��δ��offset_relative_angle��ֵ����sin����Ĭ��000������ʱ�����������״̬����ʼ����
		60ms��ֵ100���Ҹ���offset_relative_angle�ϵ�ʱ����������Ի�е�Ƕ�ֵ������б�¼��㣬����
		*/
    relative_angle = SinRampVariation_3(GimbalData[YAW].offset_relative_angle, 3, 3);
    
    pid_calc(&Gimbal_Motor_pid_Pos[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle, relative_angle /*Gimbal[YAW].offset_relative_angle*/);  //λ�û�
    pid_calc(&Gimbal_Motor_pid_Spd[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm, Gimbal_Motor_pid_Pos[YAW_CAN2_205].pos_out);  //�ٶȻ�
    //Gimbal[YAW].can_send = Gimbal_Motor_Pid_Spd[YAW].pos_out; //��ֵ��CAN����
    
    if((Gimbal_Motor_Encoder[YAW_CAN2_205].angle  <= (CHASSIS_YAW_MID_FRONT + FAULT_TOLERANT_OFFSET_ANGLE))&&(Gimbal_Motor_Encoder[YAW_CAN2_205].angle  >= (CHASSIS_YAW_MID_FRONT - FAULT_TOLERANT_OFFSET_ANGLE))) /*��̨yaw��ת������ǰ��������*/
    {
        Gimbal_Init_Flag = TRUE;
        IMU_Yaw.offset_relative_angle = IMU_Yaw.total_angle; //�õ��ϵ�ʱ����������ڵ���ǰ�������ĵ���ԽǶ�
        //IMU_Yaw.offset_relative_angle = rm_imu_data.yaw_total_angle; //�ٷ�������
        sin_ramp_state.sin_ramp_switch = 0;
    }
    else
    {
        Gimbal_Init_Flag = FALSE;
    }
    
}


void GimbalAuto(void)
{
	static float yaw_angle_err/*,pitch_angle_err*/;//YAW,pitch���Ӿ��Ƕ�ƫ���Ԥ����
	//�����Ӿ�Ŀ��ֵ
	//gimbal_control.gimbal_pitch_motor.target_val =0;
  gimbal_control.gimbal_yaw_motor.target_val =0;
	//����Ŀ����ԽǶ�ֵ����
	Vision_Yaw_Error(&(gimbal_control.gimbal_yaw_motor.Vision_Data));
	//Vision_Pitch_Error(&(gimbal_control.gimbal_pitch_motor.Vision_Data));
	if(Vision_UpDate()==TRUE)
	{
		Vision_UpDate_Clean();//�����Ӿ����ݸ��±�־λ
		if(VisonRecvData.model==1)//���ʶ��װ�װ壬�͸�������
		{
			 //pitch_angle_err=gimbal_control.gimbal_pitch_motor.Vision_Data;
			 yaw_angle_err=gimbal_control.gimbal_yaw_motor.Vision_Data;
				
			 //gimbal_control.gimbal_pitch_motor.Vision_Angle_Error=pitch_angle_err;
			 gimbal_control.gimbal_yaw_motor.Vision_Angle_Error=yaw_angle_err;
				
			 ANGLE_LIMIT(gimbal_control.gimbal_yaw_motor.Vision_Angle_Error, 5 , -5 );
		}
		else
		{
			gimbal_control.gimbal_yaw_motor.Vision_Angle_Error = 0;
		}
	}
	
	/*��̨Pitch�����ݸ���*/
	GimbalData[PITCH].angle_set -= remote_control.ch4 * PITCH_ROTATE_INC_FACT;
	ANGLE_LIMIT( GimbalData[PITCH].angle_set ,1024 ,-1024 ) ;
	
	Jscope_vision_yaw();//Jlink��ӡ����Ƕ�����
	
}


void GimbalRC()
{

	GimbalDataUpdate();
	//ShootDataUpdate();
	//GimbalPidOutput();
}


void GimbalDataUpdate()
{ 
	
	//yaw��Ƕ����� �Ƕ�����
	GimbalData[YAW].angle_set -= RcDeadlineLimit(remote_control.ch3, 5) * YAW_ROTATE_INC_FACT;
	//ANGLE_LIMIT( GimbalData[YAW].angle_set , 3072 ,-3072 ) ;
	
	//pitch��Ƕ����� �Ƕ�����	��ҡ��
	GimbalData[PITCH].angle_set -= RcDeadlineLimit(remote_control.ch4, 5) * PITCH_ROTATE_INC_FACT;
	ANGLE_LIMIT( GimbalData[PITCH].angle_set ,1024 ,-1024 ) ;
	
}

void ShootControlSetValue()
{
	static bool left_flag = 0;
  static uint8_t frictiongear_on_off;
	
  //static bool right_flag = 0;
    if(remote_control.ch5 == -660)
    {
        frictiongear_on_off = 1;
    }
    else if(remote_control.ch5 == 660)
    {
        frictiongear_on_off = 0;
    }
    
    if(frictiongear_on_off == 1)
    {
        friction_speed_set = 6000;
    }
    else if(frictiongear_on_off == 0)
    {
        friction_speed_set = 0;
    }
		
	//	Ħ�����ٶ��趨
//	if(remote_control.switch_left == Switch_Up)
//			friction_speed_set = 7000;
//	else if(remote_control.switch_left == Switch_Middle)
//			friction_speed_set = 0;
//	else if(remote_control.switch_left == Switch_Down)
//			friction_speed_set = 0;

		
	//�ϲ�����תһ��<90��>
	if(remote_control.switch_left == Switch_Up && left_flag==1)	{		
			PluckUP_angel_set += 2048; 
			left_flag=0;			
	}
	else if(remote_control.switch_left == Switch_Middle && left_flag==0) 	{
			left_flag = 1; 
	}
	
	/*Jscope��ӡʹ��*/
	angular_speed1 = Gimbal_Motor_Encoder[FRICTION1_CAN1_207].speed_rpm;
	angular_speed2 = Gimbal_Motor_Encoder[FRICTION2_CAN1_208].speed_rpm;

	
  //�����ֲ����Ƕȼ��Ƹ��趨
//	if(remote_control.switch_right == Switch_Up && right_flag==1)	{
//			PluckData.Pluck_angle_set += 1382; right_flag=0;			//360/5*19.2<һ��>
//	}
//	else if(remote_control.switch_right == Switch_Down && right_flag==1) {
//			PluckData.Pluck_angle_set += 8192; right_flag=0;    
//	}
//	else if(remote_control.switch_right == Switch_Middle && right_flag==0) 	{
//			right_flag = 1;
//	}

}



/*������⺯����ÿ���һ�β���������*/
void lock_bullet_detection()
{
	static int lock_flag = 0;						//���ڿ��������־λ����ֹ�ظ�if���
	
	//200ms�󲦵��̴�ʱλ�����趨λ�����50����2�� �����ֻ���
	if( PluckData.angle_set - Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2 > 50 && PluckData.lock_time_count == 4 ) 
	{
		PluckData.angle_staging = PluckData.angle_set;																				//�Ƕ��ݴ�
		PluckData.angle_set = Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2 - 113.5;				//�����ֻ���5��		22.7*5
		lock_flag = 1;
	}
	
	if( PluckData.lock_time_count == 8 && lock_flag == 1 )														//�����ڿ�����100ms��ת������ǰ�Ƕ�
	{
		PluckData.angle_set = PluckData.angle_staging;
		PluckData.lock_flag = 0;																									//�رտ������
		lock_flag = 0;
	}
}


void GimbalPidCalc()
{
	
	//Ħ����PID  3508���ٱ�19.2:1
	pid_calc(&Gimbal_Motor_pid_Spd[FRICTION1_CAN1_207], Gimbal_Motor_Encoder[FRICTION1_CAN1_207].speed_rpm, friction_speed_set);
	pid_calc(&Gimbal_Motor_pid_Spd[FRICTION2_CAN1_208], Gimbal_Motor_Encoder[FRICTION2_CAN1_208].speed_rpm, -friction_speed_set);
		
	//pluck�ᴮ��PID ��������3508���ٱ�19.2:1
	pid_calc(&Gimbal_Motor_pid_Pos[PLUCK_CAN2_206], Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2f, PluckData.angle_set );								/*λ�û�PID����*/
	pid_calc(&Gimbal_Motor_pid_Spd[PLUCK_CAN2_206], Gimbal_Motor_Encoder[PLUCK_CAN2_206].speed_rpm/19.2f,	  Gimbal_Motor_pid_Pos[PLUCK_CAN2_206].pos_out); 	/*�ٶȻ�PID����*/ 
	
	//�ϲ�������PID    2006���ٱ�36:1
	pid_calc(&Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205], Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].total_angle/36, PluckUP_angel_set );													/*λ�û�PID����*/
	pid_calc(&Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205], Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].speed_rpm/36,	 Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205].pos_out); 	/*�ٶȻ�PID����*/ 
	
	//pitch�ᴮ��PID
	pid_calc(&Gimbal_Motor_pid_Pos[PITCH_CAN1_206], Gimbal_Motor_Encoder[PITCH_CAN1_206].total_angle, GimbalData[PITCH].angle_set ); 													/*λ�û�PID����*/
	pid_calc(&Gimbal_Motor_pid_Spd[PITCH_CAN1_206], Gimbal_Motor_Encoder[PITCH_CAN1_206].speed_rpm, 	Gimbal_Motor_pid_Pos[PITCH_CAN1_206].pos_out);		/*�ٶȻ�PID����*/ 
	
	//�жϴ�ʱΪ����ģʽ����ң��ģʽ
	if(gimbal_behaviour == GIMBAL_Auto_aiming)
	{	
		pid_calc(&Gimbal_Pid_Vision_Pos[0], gimbal_control.gimbal_yaw_motor.Vision_Angle_Error, gimbal_control.gimbal_yaw_motor.target_val);
		pid_calc(&Gimbal_Pid_Vision_Spd[0], Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm/19.2, Gimbal_Pid_Vision_Pos[0].pos_out);
		
		//pid_calc(&Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208], gimbal_control.gimbal_pitch_motor.Vision_Angle_Error, gimbal_control.gimbal_pitch_motor.target_val);	
		//pid_calc(&Gimbal_Pid_Vision_Spd[GIMBAL_PITCH_208], Gimbal_Motor[GIMBAL_PITCH_208].speed_rpm, Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208].pos_out);
		
	}
	
	else if(gimbal_behaviour == GIMBAL_Manaul_control)
	{

		//yaw�ᴮ��PID
		pid_calc(&Gimbal_Motor_pid_Pos[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle/19.2f, GimbalData[YAW].angle_set ); 													/*λ�û�PID����*/
		pid_calc(&Gimbal_Motor_pid_Spd[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm/19.2f, 	Gimbal_Motor_pid_Pos[YAW_CAN2_205].pos_out); 	/*�ٶȻ�PID����*/ 
				
		//GimbalPidOutput();
	}
	
}

void GimbalPidOutput()
{
	//��̨���PID���
	set_moto_current2(&hcan1, Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205].pos_out, Gimbal_Motor_pid_Spd[PITCH_CAN1_206].pos_out, 
														Gimbal_Motor_pid_Spd[FRICTION1_CAN1_207].pos_out, Gimbal_Motor_pid_Spd[FRICTION2_CAN1_208].pos_out);
	
	//��̨ģʽ�ж�
	if(gimbal_behaviour == GIMBAL_Auto_aiming){
		set_moto_current2(&hcan2, Gimbal_Pid_Vision_Spd[0].pos_out, Gimbal_Motor_pid_Spd[PLUCK_CAN2_206].pos_out, 0, 0);
	}
	else if(gimbal_behaviour == GIMBAL_Manaul_control){
		set_moto_current2(&hcan2, Gimbal_Motor_pid_Spd[YAW_CAN2_205].pos_out, Gimbal_Motor_pid_Spd[PLUCK_CAN2_206].pos_out, 0, 0);
	}
	
}


