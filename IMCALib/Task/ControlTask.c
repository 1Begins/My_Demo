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
uint8_t usmart_count = 0;						//usmart扫描100ms计数


//电机控制任务
void AllTask(void)
{
	/*失控保护*/
    if(Campaign_Ctrl == 1)			//1ms任务
    {
			
        Campaign_Ctrl=0;
		
				usmart_count++;
			
				GimbalControl();
				ChassisControl();

				/*原为2ms任务<尚未进2ms分频>*/
				GimbalData[0].relative_angle = GetEncoderRelativeAngle(); /*得到云台与底盘的相对角度*/
				GimbalData[0].relative_angle_rad = GimbalData[0].relative_angle*Motor_Ecd_To_Rad;//将编码器相对机械角度值转化为角度，单位：rad，用于坐标系转换的计算

				GetImuAngle(&IMU_Yaw);  //读取陀螺仪数据(2ms)

				/*光毓电机can发送测试*/
				//position_send_out(&hcan1, 100);  

				/*Jscope波形函数更新*/
				//Jscope_pitch();		
				//Jscope_yaw();
				Jscope_pluck();
				Jscope_pushrod();
				Jscope_vision_yaw();
				
    }		
    if(Data_Send_ANO_DT)/*发送数据到匿名地面站，周期为2ms*/
    {
      
        Data_Send_ANO_DT = 0;
//        ANO_DT_DataUpdate(); /*发送数据到匿名地面站，串口4发送数据*/
        //SwDataWaveUpdate(); /*串口1发送数据到山外多功能调试助手显示波形*/
        GimbalData[YAW].relative_angle = GetEncoderRelativeAngle(); /*得到云台与底盘的相对角度*/
        GimbalData[YAW].relative_angle_rad = GimbalData[0].relative_angle*Motor_Ecd_To_Rad;//将编码器相对机械角度值转化为角度，单位：rad，用于坐标系转换的计算
        
        //陀螺仪测试
        GetImuAngle(&IMU_Yaw);  //读取陀螺仪数据
        
//        PrintfInfo();  //串口打印相关信息
//        printf("\n\r IMU total angle and angle is : %.3f %.3f \n\r", IMU_Yaw.total_angle, IMU_Yaw.angle);
//        printf("\n\r %.3f %.3f \n\r", IMU_Yaw.total_angle, Gimbal[YAW].set_angle_imu);
//        printf("\n\r %d \n\r", Gimbal_Motor[PITCH].angle);
//        printf("\n\r %f , %f \n\r", IMU_Yaw.angle, IMU_Yaw.speed_gyro);
//        printf("\n\r Robomaster! \n\r");
//        printf("\r\n yaw轴角速度：%f \r\n", rm_imu_data.gyro_fp32[2]);
//        printf("\r\n %d \r\n", Gimbal_Motor[YAW].angle);
//        printf("\n\r %d \n\r", remote_control.ch5);
     
    }
		
		if(PluckData.pluck_time_count == 10)			//500ms拨弹一次(初级形态视频尚未使用自动拨弹)
		{
			PluckData.pluck_time_count = 0;
			
			if(PluckData.pluck_flag == 1){
				//PluckData.angle_set += 1638.4;			//控制拨弹角度
				PluckData.lock_flag = 1;							//卡弹检测标志位
				PluckData.lock_time_count = 0;				//卡弹检测计数清空
			}
		}
		
		if(usmart_count == 100)			//100ms任务
		{ 
			KEY0_StateRead();//100ms扫描一次按键
			
			//usmart调参工具数据打印
			//printf("位置环 P:%f I:%f D:%f\n",pid_yaw_P.p,pid_yaw_P.i,pid_yaw_P.d);
			ShowJudgeMeassge();		//裁判系统数据更新
			usmart_count = 0;
		}
 }	


void PID_Init()
{
	Gimbal_PID_Init();
	Chassis_PID_Init();
}


/*************************调试参数赋值******************************
@belief 在线调参时上位机发过来的参数是已经乘以1000的，这里要除以1000
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
