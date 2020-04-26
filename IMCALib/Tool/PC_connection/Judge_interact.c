#include "Driver_Judge.h"
#include "Judge_interact.h"
#include "Vision_interact.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"

/*--------裁判系统读取V1.0--------*/
/*-----------------------------------------------------------------------
//当前裁判系统测试了底盘功率和射频，枪口热量等的读取，裁判系统发送暂时还没有做。
//串口3DMA通讯
//存在问题
//裁判系统在一次数据发送中，会包含多帧的数据。但在当前缓数组没溢出的情况下
//下一次数据发送的时候会接收到数据首位不为0XA5的帧,使校验失败。导致一定程度的掉包
//但不影响前一帧已过校验的读取数据准度。
*--------------------------------------------------------------------*/

/*Jscope调试参数*/
float  Jscope_Chassis_Heat;//底盘功率
float  Jscope_Fire_Speed;  //开火射速
uint8_t  Jscope_Fire_Feq;  //射频
int16_t  Jscope_Fire_Heat; //17mm枪口热量
int16_t  Jscope_Fire_Num;  //发弹数


/*****************裁判系统数据定义**********************/
ext_game_state_t       				    Game_State;					      //0x0001
ext_game_result_t            		  Game_Result;				      //0x0002
ext_game_robot_survivors_t        Game_Robot_Survivors;     //0x0003
ext_event_data_t        			    Event_Data;					      //0x0101
ext_supply_projectile_action_t		Supply_Projectile_Action;	//0x0102
ext_supply_projectile_booking_t		Supply_Projectile_Booking;//0x0103
ext_game_robot_state_t			  	  Game_Robot_Stat;				  //0x0201
ext_power_heat_data_t		  		    Power_Heat_Data;				  //0x0202
ext_game_robot_pos_t				      Game_Robot_Pos;				    //0x0203
ext_buff_musk_t						        Buff_Musk;					      //0x0204
aerial_robot_energy_t			        Aerial_Robot_Energy;		  //0x0205
ext_robot_hurt_t					        Robot_Hurt;					      //0x0206
ext_shoot_data_t					        Shoot_Data;					      //0x0207

stoJudgeRecvData_t  Judge_Recv_Data;//裁判系统校验标志位接收结构体

uint8_t Jud_Tx_Buffer[SE_BUFFER_SIZE] ={0};//发送缓冲数组
uint8_t Jud_Rx_Buffer[JUD_RC_BUFFER_SIZE] ={0};//接收缓冲数组

/*-----------------------------------------------------------------------
\@brief 裁判系统数据读取
\@param 无
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void JudgeRead(uint8_t *RecvUsartData)
{
	uint8_t Judge_length =0;//帧数据长度

	
	//校验标志位赋值(用于串口打印调试)
	
	Judge_Recv_Data.crc8 =(RecvUsartData[4]);
	Judge_Recv_Data.data_lenth=(RecvUsartData[2]<<8|RecvUsartData[1]);
	Judge_Recv_Data.cmdid=(RecvUsartData[6]<<8|RecvUsartData[5]);

	//判断帧头首字节是否为0XA5
	if(RecvUsartData[0] == JUDGE_FRMAE_HEADER)
	{
		//CRC8帧头校验
		if(Verify_CRC8_Check_Sum(RecvUsartData, JUDGE_LEN_HAEDER) ==TRUE)
		{
			//计算一帧数据的长度，用作CRC16校验
			Judge_length = RecvUsartData[DATA_LENGTH] + JUDGE_LEN_HAEDER + JUDGE_LEN_CMDID + JUDGE_LEN_TAIL;
			
			Judge_Recv_Data.crc16 =(RecvUsartData[Judge_length-1]<<8|RecvUsartData[Judge_length-2]);
			//CRC16帧尾校验
			if(Verify_CRC16_Check_Sum(RecvUsartData, Judge_length) ==TRUE)
		 {
				switch(Judge_Recv_Data.cmdid)
				{
					case ID_game_state:        			//0x0001
						memcpy(&Game_State, (RecvUsartData + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&Game_Result, (RecvUsartData + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&Game_Robot_Survivors, (RecvUsartData + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&Event_Data, (RecvUsartData + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&Supply_Projectile_Action, (RecvUsartData + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_supply_projectile_booking:  //0x0103
						memcpy(&Supply_Projectile_Booking, (RecvUsartData + DATA), LEN_supply_projectile_booking);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&Game_Robot_Stat, (RecvUsartData + DATA), LEN_game_robot_state);;
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&Power_Heat_Data, (RecvUsartData + DATA), LEN_power_heat_data);

					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&Game_Robot_Pos, (RecvUsartData + DATA), LEN_game_robot_pos);
					  
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&Buff_Musk, (RecvUsartData + DATA), LEN_buff_musk);
					break;
					
					case ID_aerial_robot_energy:      	//0x0205
						memcpy(&Aerial_Robot_Energy, (RecvUsartData + DATA), LEN_aerial_robot_energy);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&Robot_Hurt, (RecvUsartData + DATA), LEN_robot_hurt);
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&Shoot_Data, (RecvUsartData + DATA), LEN_shoot_data);
            FireNum();//记录发弹量
					break;
				}

				Judge_Recv_Data.next_frame_header =*(RecvUsartData+JUDGE_LEN_HAEDER+JUDGE_LEN_CMDID+JUDGE_LEN_TAIL+Judge_Recv_Data.data_lenth);
         /*****************打印当前校验帧数距（测试用）**********************/
//				printf("cmdid=%x\r\n",JudgeRecvData.cmdid);
//				printf("data_lenth=%d\r\n",JudgeRecvData.data_lenth);
//				printf("crc8=%x\r\n",JudgeRecvData.crc8);
//				printf("tail_crc16=%x\r\n",JudgeRecvData.crc16);
//				printf("next_frame_header=%x\r\n",JudgeRecvData.next_frame_header);
				/************************************************************/
				//裁判系统在一次数据发送会包含多帧数据，若当前帧数据的后一位为帧头标志位则进行下一次接收
				if(*(RecvUsartData+JUDGE_LEN_HAEDER+JUDGE_LEN_CMDID+JUDGE_LEN_TAIL+Judge_Recv_Data.data_lenth)==0XA5)
				{
					JudgeRead(RecvUsartData+JUDGE_LEN_HAEDER+JUDGE_LEN_CMDID+JUDGE_LEN_TAIL+Judge_Recv_Data.data_lenth);
				}

			}
	    else
			{
			 printf("Vreify_crc16_error!\r\n");
			}	
    }
		else
		{
			printf("Vreify_crc8_error!\r\n");
		}	
  } 
	else
	{
		printf("Vreify_header_error!\r\n");
	}	
	
}	



/*-----------------------------------------------------------------------
\@brief 串口6DMA初始化
\@param 无
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void HAL_UART6_Receive_IT_IDLE()
{
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);//开启空闲中断
  HAL_UART_Receive_DMA(&huart6,Jud_Rx_Buffer,RC_BUFFER_SIZE);//开启DMA接收
}
/**********************裁判系统读取函数**************************************/
//底盘输出功率 单位W 瓦 50HZ
float RoboChassisHeat(void)
{
	return Power_Heat_Data.chassis_power;
}
//枪口热量每秒冷却值 10HZ
uint16_t FireCoolingRate(void)
{
	return Game_Robot_Stat.shooter_heat0_cooling_limit;
}
//枪口热量冷却上限 10HZ
uint16_t FireCoolingLimit(void)
{
	return Game_Robot_Stat.shooter_heat0_cooling_limit;
}
//机器人剩余血量 10HZ
uint16_t RoboHP(void)
{
	return Game_Robot_Stat.remain_HP;
}
//机器人射速 
float FireSpeed(void)
{
	return Shoot_Data.bullet_speed;
}
//17mm枪口热量
uint16_t FireHeat17(void)
{
	return Power_Heat_Data.shooter_heat0;
}

//射频
uint8_t FireFeq(void)
{
	return Shoot_Data.bullet_freq;
}
/*-----------------------------------------------------------------------
\@brief 发弹量计数
\@param 无
\@return uint16_t 发弹数
\@reference 
*--------------------------------------------------------------------*/
//统计发弹量
float Shoot_Speed[2]={0};
uint16_t ShootNum=0;
uint16_t FireNum(void)
{
	
	
	Shoot_Speed[NOW_t]=Shoot_Data.bullet_speed;
	if(Shoot_Speed[NOW_t]!=Shoot_Speed[LAST_t])//发射速度不相等就相当于发射了一颗弹丸
	{
		ShootNum++;
		Shoot_Speed[LAST_t]=Shoot_Speed[NOW_t];
	}
	return ShootNum;
}
/*-----------------------------------------------------------------------
\@brief JSCOOP参数打印
\@param 无
\@return 
\@reference  用来打印裁判系统读取的功率，射频等参数
*--------------------------------------------------------------------*/
void ShowJudgeMeassge(void)
{
	Jscope_Chassis_Heat=RoboChassisHeat();
	Jscope_Fire_Speed=FireSpeed();
	Jscope_Fire_Feq=FireFeq();
	Jscope_Fire_Heat=FireHeat17();
	Jscope_Fire_Num=FireNum();
}

