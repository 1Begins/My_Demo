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


uint8_t Gimbal_Init_Flag;  //云台初始化标识

pid_t Gimbal_Motor_pid_Spd[6] = {0};  //云台电机PID结构体
pid_t Gimbal_Motor_pid_Pos[6] = {0};

gimbal_t PluckData;
gimbal_t GimbalData[2];  //云台结构体

/*自瞄PID参数*/
pid_t Gimbal_Pid_Vision_Pos[2];
pid_t Gimbal_Pid_Vision_Spd[2];

gimbal_control_t gimbal_control;
gimbal_behaviour_e gimbal_behaviour;

int32_t friction_speed_set = 0;
int32_t PluckUP_angel_set = 0;


float angular_speed1,angular_speed2;	

void Gimbal_PID_Init()
{	
	/*结构体数组清零*/
	memset(Gimbal_Motor_pid_Spd, 0, sizeof(pid_t)*6);
	memset(Gimbal_Motor_pid_Pos, 0, sizeof(pid_t)*6);

	//摩擦轮3508电机PID初始化
	PID_struct_init(&Gimbal_Motor_pid_Spd[FRICTION1_CAN1_207], POSITION_PID, 6000, 2000, 8.3f, 0.002f, 0.0f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[FRICTION2_CAN1_208], POSITION_PID, 6000, 2000, 8.3f, 0.002f, 0.0f);

	//3508拨弹电机PID初始化		3508拨弹PLUCK电机PID初始化,此套PID适用于目标转速小于150rpm
	PID_struct_init(&Gimbal_Motor_pid_Pos[PLUCK_CAN2_206], POSITION_PID, 100, 2000, 0.6f, 0.0f, 1.0f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[PLUCK_CAN2_206], POSITION_PID, 16000, 10000, 175.0f, 0.8f, 0.0f);
	
	//推杆2006电机PID初始化
	PID_struct_init(&Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205], POSITION_PID, 500, 10, 0.22f, 0.0f, 0.1f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205], POSITION_PID, 10000, 2000, 120.0f, 0.05f, 1.0f);
	
	//YAW轴电机PID初始化  
	PID_struct_init(&Gimbal_Motor_pid_Pos[YAW_CAN2_205], POSITION_PID, 100, 2000, 0.6f, 0.0f, 1.0f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[YAW_CAN2_205], POSITION_PID, 5000, 3000, 175.0f, 0.8f, 0.0f);//16000 10000
	
	//PITCH轴电机PID初始化
	PID_struct_init(&Gimbal_Motor_pid_Pos[PITCH_CAN1_206], POSITION_PID, 7, 20, 0.015f, 0.0f, 0.2f);
	PID_struct_init(&Gimbal_Motor_pid_Spd[PITCH_CAN1_206], POSITION_PID, 20000, 15000, 13.2f, 7.5f, 7.0f);
	
	/*Yaw电机位置环速度环自瞄位置环初始化PID参数*/
	PID_struct_init(&Gimbal_Pid_Vision_Pos[0], POSITION_PID, 100, 2000, 12.0f, 0.0f, 0.0f);		//初始化位置环PID参数
	PID_struct_init(&Gimbal_Pid_Vision_Spd[0], POSITION_PID, 15000, 5000, 175.0f, 0.8f, 0.0f); 		//初始化速度环PID参数 
	
	
}

void GimbalControl(void)
{
	
	/*云台状态切换标志位(防止自瞄模式切换遥控模式抖动)*/
	static bool gimbal_mode_flag = 0;  
	
	switch(remote_control.switch_left)
	{
		/*云台遥控模式*/
		case Switch_Up:		
		case Switch_Middle:
		{
			if( gimbal_mode_flag == 1 )
			{
				gimbal_mode_flag = 0;
				GimbalData[YAW].angle_set = Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle/19.2f;		//自瞄模式切换回遥控模式后更新yaw设定值位当前值
			}
			
			gimbal_behaviour = GIMBAL_Manaul_control;
			GimbalRC();			//yaw，pitch轴数据更新
			break;
		}
		
		/*云台自瞄模式，暂时只对pitch轴做自瞄*/
		case Switch_Down:
		{
			gimbal_mode_flag = 1;
			gimbal_behaviour = GIMBAL_Auto_aiming;
      GimbalAuto();		//yaw，pitch轴数据更新
			break;	
		}
		
	}	
	
	/*射击数据更新（自瞄模式下自动开启摩擦轮）*/
	ShootControlSetValue();		
	GimbalPidCalc();
	GimbalPidOutput();
}

/*云台初始化，居中*/
void GimbalInit(void)
{
    static int32_t relative_angle;
    
    if(Gimbal_Motor_Encoder[YAW_CAN2_205].msg_cnt > 60)
    {
        sin_ramp_state.sin_ramp_switch = 1;
        sin_ramp_state.compare_value_up = 0;
        sin_ramp_state.compare_value_dowm = 0;
        
        GimbalData[YAW].offset_relative_angle = GetEncoderOffsetRelativeAngle(); /*得到上电时编码器的相对机械角度值*/
        
    }
    
		/*
		定义函数时，未对offset_relative_angle赋值，且sin函数默认000，即此时电机处于锁紧状态（初始化）
		60ms后赋值100，且赋予offset_relative_angle上电时编码器的相对机械角度值，进入斜坡计算，归中
		*/
    relative_angle = SinRampVariation_3(GimbalData[YAW].offset_relative_angle, 3, 3);
    
    pid_calc(&Gimbal_Motor_pid_Pos[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle, relative_angle /*Gimbal[YAW].offset_relative_angle*/);  //位置环
    pid_calc(&Gimbal_Motor_pid_Spd[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm, Gimbal_Motor_pid_Pos[YAW_CAN2_205].pos_out);  //速度环
    //Gimbal[YAW].can_send = Gimbal_Motor_Pid_Spd[YAW].pos_out; //赋值给CAN发送
    
    if((Gimbal_Motor_Encoder[YAW_CAN2_205].angle  <= (CHASSIS_YAW_MID_FRONT + FAULT_TOLERANT_OFFSET_ANGLE))&&(Gimbal_Motor_Encoder[YAW_CAN2_205].angle  >= (CHASSIS_YAW_MID_FRONT - FAULT_TOLERANT_OFFSET_ANGLE))) /*云台yaw轴转到底盘前面正中央*/
    {
        Gimbal_Init_Flag = TRUE;
        IMU_Yaw.offset_relative_angle = IMU_Yaw.total_angle; //得到上电时陀螺仪相对于底盘前面正中心的相对角度
        //IMU_Yaw.offset_relative_angle = rm_imu_data.yaw_total_angle; //官方陀螺仪
        sin_ramp_state.sin_ramp_switch = 0;
    }
    else
    {
        Gimbal_Init_Flag = FALSE;
    }
    
}


void GimbalAuto(void)
{
	static float yaw_angle_err/*,pitch_angle_err*/;//YAW,pitch轴视觉角度偏差，供预测用
	//设置视觉目标值
	//gimbal_control.gimbal_pitch_motor.target_val =0;
  gimbal_control.gimbal_yaw_motor.target_val =0;
	//自瞄目标相对角度值更新
	Vision_Yaw_Error(&(gimbal_control.gimbal_yaw_motor.Vision_Data));
	//Vision_Pitch_Error(&(gimbal_control.gimbal_pitch_motor.Vision_Data));
	if(Vision_UpDate()==TRUE)
	{
		Vision_UpDate_Clean();//清零视觉数据更新标志位
		if(VisonRecvData.model==1)//如果识别到装甲板，就更新数据
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
	
	/*云台Pitch轴数据更新*/
	GimbalData[PITCH].angle_set -= remote_control.ch4 * PITCH_ROTATE_INC_FACT;
	ANGLE_LIMIT( GimbalData[PITCH].angle_set ,1024 ,-1024 ) ;
	
	Jscope_vision_yaw();//Jlink打印自瞄角度数据
	
}


void GimbalRC()
{

	GimbalDataUpdate();
	//ShootDataUpdate();
	//GimbalPidOutput();
}


void GimbalDataUpdate()
{ 
	
	//yaw轴角度修正 角度限制
	GimbalData[YAW].angle_set -= RcDeadlineLimit(remote_control.ch3, 5) * YAW_ROTATE_INC_FACT;
	//ANGLE_LIMIT( GimbalData[YAW].angle_set , 3072 ,-3072 ) ;
	
	//pitch轴角度修正 角度限制	右摇杆
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
		
	//	摩擦轮速度设定
//	if(remote_control.switch_left == Switch_Up)
//			friction_speed_set = 7000;
//	else if(remote_control.switch_left == Switch_Middle)
//			friction_speed_set = 0;
//	else if(remote_control.switch_left == Switch_Down)
//			friction_speed_set = 0;

		
	//上拨弹轮转一格<90°>
	if(remote_control.switch_left == Switch_Up && left_flag==1)	{		
			PluckUP_angel_set += 2048; 
			left_flag=0;			
	}
	else if(remote_control.switch_left == Switch_Middle && left_flag==0) 	{
			left_flag = 1; 
	}
	
	/*Jscope打印使用*/
	angular_speed1 = Gimbal_Motor_Encoder[FRICTION1_CAN1_207].speed_rpm;
	angular_speed2 = Gimbal_Motor_Encoder[FRICTION2_CAN1_208].speed_rpm;

	
  //拨弹轮拨弹角度及推杆设定
//	if(remote_control.switch_right == Switch_Up && right_flag==1)	{
//			PluckData.Pluck_angle_set += 1382; right_flag=0;			//360/5*19.2<一格>
//	}
//	else if(remote_control.switch_right == Switch_Down && right_flag==1) {
//			PluckData.Pluck_angle_set += 8192; right_flag=0;    
//	}
//	else if(remote_control.switch_right == Switch_Middle && right_flag==0) 	{
//			right_flag = 1;
//	}

}



/*卡弹检测函数，每完成一次拨弹进入检测*/
void lock_bullet_detection()
{
	static int lock_flag = 0;						//存在卡弹情况标志位，防止重复if语句
	
	//200ms后拨弹盘此时位置与设定位置相差50，即2° 拨弹轮回退
	if( PluckData.angle_set - Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2 > 50 && PluckData.lock_time_count == 4 ) 
	{
		PluckData.angle_staging = PluckData.angle_set;																				//角度暂存
		PluckData.angle_set = Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2 - 113.5;				//拨弹轮回退5°		22.7*5
		lock_flag = 1;
	}
	
	if( PluckData.lock_time_count == 8 && lock_flag == 1 )														//若存在卡弹，100ms后转到卡弹前角度
	{
		PluckData.angle_set = PluckData.angle_staging;
		PluckData.lock_flag = 0;																									//关闭卡弹检测
		lock_flag = 0;
	}
}


void GimbalPidCalc()
{
	
	//摩擦轮PID  3508减速比19.2:1
	pid_calc(&Gimbal_Motor_pid_Spd[FRICTION1_CAN1_207], Gimbal_Motor_Encoder[FRICTION1_CAN1_207].speed_rpm, friction_speed_set);
	pid_calc(&Gimbal_Motor_pid_Spd[FRICTION2_CAN1_208], Gimbal_Motor_Encoder[FRICTION2_CAN1_208].speed_rpm, -friction_speed_set);
		
	//pluck轴串级PID （拨弹）3508减速比19.2:1
	pid_calc(&Gimbal_Motor_pid_Pos[PLUCK_CAN2_206], Gimbal_Motor_Encoder[PLUCK_CAN2_206].total_angle/19.2f, PluckData.angle_set );								/*位置环PID计算*/
	pid_calc(&Gimbal_Motor_pid_Spd[PLUCK_CAN2_206], Gimbal_Motor_Encoder[PLUCK_CAN2_206].speed_rpm/19.2f,	  Gimbal_Motor_pid_Pos[PLUCK_CAN2_206].pos_out); 	/*速度环PID计算*/ 
	
	//上拨弹串级PID    2006减速比36:1
	pid_calc(&Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205], Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].total_angle/36, PluckUP_angel_set );													/*位置环PID计算*/
	pid_calc(&Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205], Gimbal_Motor_Encoder[PLUCKUP_CAN1_205].speed_rpm/36,	 Gimbal_Motor_pid_Pos[PLUCKUP_CAN1_205].pos_out); 	/*速度环PID计算*/ 
	
	//pitch轴串级PID
	pid_calc(&Gimbal_Motor_pid_Pos[PITCH_CAN1_206], Gimbal_Motor_Encoder[PITCH_CAN1_206].total_angle, GimbalData[PITCH].angle_set ); 													/*位置环PID计算*/
	pid_calc(&Gimbal_Motor_pid_Spd[PITCH_CAN1_206], Gimbal_Motor_Encoder[PITCH_CAN1_206].speed_rpm, 	Gimbal_Motor_pid_Pos[PITCH_CAN1_206].pos_out);		/*速度环PID计算*/ 
	
	//判断此时为自瞄模式还是遥控模式
	if(gimbal_behaviour == GIMBAL_Auto_aiming)
	{	
		pid_calc(&Gimbal_Pid_Vision_Pos[0], gimbal_control.gimbal_yaw_motor.Vision_Angle_Error, gimbal_control.gimbal_yaw_motor.target_val);
		pid_calc(&Gimbal_Pid_Vision_Spd[0], Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm/19.2, Gimbal_Pid_Vision_Pos[0].pos_out);
		
		//pid_calc(&Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208], gimbal_control.gimbal_pitch_motor.Vision_Angle_Error, gimbal_control.gimbal_pitch_motor.target_val);	
		//pid_calc(&Gimbal_Pid_Vision_Spd[GIMBAL_PITCH_208], Gimbal_Motor[GIMBAL_PITCH_208].speed_rpm, Gimbal_Pid_Vision_Pos[GIMBAL_PITCH_208].pos_out);
		
	}
	
	else if(gimbal_behaviour == GIMBAL_Manaul_control)
	{

		//yaw轴串级PID
		pid_calc(&Gimbal_Motor_pid_Pos[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].total_angle/19.2f, GimbalData[YAW].angle_set ); 													/*位置环PID计算*/
		pid_calc(&Gimbal_Motor_pid_Spd[YAW_CAN2_205], Gimbal_Motor_Encoder[YAW_CAN2_205].speed_rpm/19.2f, 	Gimbal_Motor_pid_Pos[YAW_CAN2_205].pos_out); 	/*速度环PID计算*/ 
				
		//GimbalPidOutput();
	}
	
}

void GimbalPidOutput()
{
	//云台电机PID输出
	set_moto_current2(&hcan1, Gimbal_Motor_pid_Spd[PLUCKUP_CAN1_205].pos_out, Gimbal_Motor_pid_Spd[PITCH_CAN1_206].pos_out, 
														Gimbal_Motor_pid_Spd[FRICTION1_CAN1_207].pos_out, Gimbal_Motor_pid_Spd[FRICTION2_CAN1_208].pos_out);
	
	//云台模式判断
	if(gimbal_behaviour == GIMBAL_Auto_aiming){
		set_moto_current2(&hcan2, Gimbal_Pid_Vision_Spd[0].pos_out, Gimbal_Motor_pid_Spd[PLUCK_CAN2_206].pos_out, 0, 0);
	}
	else if(gimbal_behaviour == GIMBAL_Manaul_control){
		set_moto_current2(&hcan2, Gimbal_Motor_pid_Spd[YAW_CAN2_205].pos_out, Gimbal_Motor_pid_Spd[PLUCK_CAN2_206].pos_out, 0, 0);
	}
	
}


