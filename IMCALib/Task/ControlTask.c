#include "ControlTask.h"
#include "RC_Task.h"
#include "pid.h"
#include "Gimbal_Task.h"
#include "Chassis_Task.h"
#include "Jscope_Task.h"
#include "CanBus_Task.h"
#include "user_lib.h"
#include "IOI2C.h"
#include "IMU_Task.h"
#include "Limit_Switch.h"
#include "Judge_interact.h"
#include "Ano_Dt.h"

uint8_t Campaign_Ctrl;
uint8_t usmart_count = 0;						//usmartɨ��100ms����


//�����������
void AllTask(void)
{
	/*ʧ�ر���*/
    if(Campaign_Ctrl == 1)			//1ms����
    {
			
        Campaign_Ctrl=0;
		
				usmart_count++;
			
				GimbalControl();
				ChassisControl();

				/*ԭΪ2ms����<��δ��2ms��Ƶ>*/
				GimbalData[0].relative_angle = GetEncoderRelativeAngle(); /*�õ���̨����̵���ԽǶ�*/
				GimbalData[0].relative_angle_rad = GimbalData[0].relative_angle*Motor_Ecd_To_Rad;//����������Ի�е�Ƕ�ֵת��Ϊ�Ƕȣ���λ��rad����������ϵת���ļ���

				GetImuAngle(&IMU_Yaw);  //��ȡ����������(2ms)

				/*��ع���can���Ͳ���*/
				//position_send_out(&hcan1, 100);  

				/*Jscope���κ�������*/
				//Jscope_pitch();		
				//Jscope_yaw();
				Jscope_pluck();
				Jscope_pushrod();
				Jscope_vision_yaw();
				
    }		
    if(Data_Send_ANO_DT)/*�������ݵ���������վ������Ϊ2ms*/
    {
      
        Data_Send_ANO_DT = 0;
//        ANO_DT_DataUpdate(); /*�������ݵ���������վ������4��������*/
        //SwDataWaveUpdate(); /*����1�������ݵ�ɽ��๦�ܵ���������ʾ����*/
        GimbalData[YAW].relative_angle = GetEncoderRelativeAngle(); /*�õ���̨����̵���ԽǶ�*/
        GimbalData[YAW].relative_angle_rad = GimbalData[0].relative_angle*Motor_Ecd_To_Rad;//����������Ի�е�Ƕ�ֵת��Ϊ�Ƕȣ���λ��rad����������ϵת���ļ���
        
        //�����ǲ���
        GetImuAngle(&IMU_Yaw);  //��ȡ����������
        
//        PrintfInfo();  //���ڴ�ӡ�����Ϣ
//        printf("\n\r IMU total angle and angle is : %.3f %.3f \n\r", IMU_Yaw.total_angle, IMU_Yaw.angle);
//        printf("\n\r %.3f %.3f \n\r", IMU_Yaw.total_angle, Gimbal[YAW].set_angle_imu);
//        printf("\n\r %d \n\r", Gimbal_Motor[PITCH].angle);
//        printf("\n\r %f , %f \n\r", IMU_Yaw.angle, IMU_Yaw.speed_gyro);
//        printf("\n\r Robomaster! \n\r");
//        printf("\r\n yaw����ٶȣ�%f \r\n", rm_imu_data.gyro_fp32[2]);
//        printf("\r\n %d \r\n", Gimbal_Motor[YAW].angle);
//        printf("\n\r %d \n\r", remote_control.ch5);
     
    }
		
		if(PluckData.pluck_time_count == 10)			//500ms����һ��(������̬��Ƶ��δʹ���Զ�����)
		{
			PluckData.pluck_time_count = 0;
			
			if(PluckData.pluck_flag == 1){
				//PluckData.angle_set += 1638.4;			//���Ʋ����Ƕ�
				PluckData.lock_flag = 1;							//��������־λ
				PluckData.lock_time_count = 0;				//�������������
			}
		}
		
		if(usmart_count == 100)			//100ms����
		{ 
			KEY0_StateRead();//100msɨ��һ�ΰ���
			
			//usmart���ι������ݴ�ӡ
			//printf("λ�û� P:%f I:%f D:%f\n",pid_yaw_P.p,pid_yaw_P.i,pid_yaw_P.d);
			ShowJudgeMeassge();		//����ϵͳ���ݸ���
			usmart_count = 0;
		}
 }	


void PID_Init()
{
	Gimbal_PID_Init();
	Chassis_PID_Init();
}


/*************************���Բ�����ֵ******************************
@belief ���ߵ���ʱ��λ���������Ĳ������Ѿ�����1000�ģ�����Ҫ����1000
*******************************************************************/
void pid_reset_yaw_speed(uint32_t kp, uint32_t ki, uint32_t kd)
{
	Gimbal_Motor_pid_Spd[YAW_CAN2_205].p = kp/1000.0f;
	Gimbal_Motor_pid_Spd[YAW_CAN2_205].i = kp/1000.0f;
	Gimbal_Motor_pid_Spd[YAW_CAN2_205].d = kp/1000.0f;
}

void pid_reset_yaw_position(uint32_t kp, uint32_t ki, uint32_t kd)
{
	Gimbal_Motor_pid_Pos[YAW_CAN2_205].p = kp/1000.0f;
	Gimbal_Motor_pid_Pos[YAW_CAN2_205].i = ki/1000.0f;
	Gimbal_Motor_pid_Pos[YAW_CAN2_205].d = kd/1000.0f; 
}

void pid_reset_pitch_speed(uint32_t kp, uint32_t ki, uint32_t kd)
{
	Gimbal_Motor_pid_Spd[PITCH_CAN1_206].p = kp/1000.0f;
	Gimbal_Motor_pid_Spd[PITCH_CAN1_206].i = kp/1000.0f;
	Gimbal_Motor_pid_Spd[PITCH_CAN1_206].d = kp/1000.0f;
}

void pid_reset_pitch_position(uint32_t kp, uint32_t ki, uint32_t kd)
{
	Gimbal_Motor_pid_Pos[PITCH_CAN1_206].p = kp/1000.0f;
	Gimbal_Motor_pid_Pos[PITCH_CAN1_206].i = ki/1000.0f;
	Gimbal_Motor_pid_Pos[PITCH_CAN1_206].d = kd/1000.0f; 
}

void pid_reset_pushrod_speed(uint32_t kp, uint32_t ki, uint32_t kd)
{
	Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205].p = kp/1000.0f;
	Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205].i = kp/1000.0f;
	Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205].d = kp/1000.0f;
}

void pid_reset_pushrod_position(uint32_t kp, uint32_t ki, uint32_t kd)
{
	Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205].p = kp/1000.0f;
	Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205].i = ki/1000.0f;
	Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205].d = kd/1000.0f; 
}
