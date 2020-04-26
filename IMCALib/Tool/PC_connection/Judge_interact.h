#ifndef _Judge_INTERACT_H
#define _Judge_INTERACT_H

#include "stm32f4xx_hal.h"


/*--------裁判系统协议--------*/
//关于官方裁判系统的一些注意
//裁判系统发送的每一帧数据不定长，但帧头长度固定，帧尾CRC16整包校验时需要统计出每帧的数据长度，
//要注意的是CRC8校验为一字节，CRC16为两字节。CRC8做头校验，CRC16做整包的数据校验
//-------------header----------------//-cmd_id-//--data------//---tail-----//
//--SOF-----datalenth---seq---CRC8--//--------//------------//---CRC16----//
//--0XA5-----2byte---1byte----1byte//--2byte-//---nbyte-----//---2byte----//
/*--注意--*/
//视觉发送下来的16位像素偏差数据高低八位反转，要进行一次转换。

#define    SE_BUFFER_SIZE 20  //裁判系统发送数据缓冲字节
#define    JUD_RC_BUFFER_SIZE 200 //裁判系统接收数据缓冲字节
//起始字节， 为0XA5
#define    JUDGE_FRMAE_HEADER 0XA5

//协议帧长度
#define    JUDGE_LEN_HAEDER 5 //帧头长
#define    JUDGE_LEN_CMDID 2 //命令码长度 0xXXXX
#define    JUDGE_LEN_TAIL  2 //帧尾长度
//整帧偏移量
typedef enum 
{
	FRAME_HEADER         = 0,//头偏移位置
	CMD_ID               = 5,//命令码偏移位置
	DATA                 = 7,//数据端偏移位置
	
}JudgeFrameOffset;
//帧头偏移位置
typedef enum
{
	SOF        =0, //起始位
	DATA_LENGTH=1,//帧数据长度（即裁判系统单次发送的数据长度）
	SEQ        =3,//包序号
	CRC8       =4,//CRC8校验
}FrameHeaderOffset;	
/***************命令码ID(2019裁判系统)********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  2    比赛机器人存活数据   		1Hz发送  
	ID: 0x0101  Byte:  4    场地事件数据   				 事件改变后发送
	ID: 0x0102  Byte:  3    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			  50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/
//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       			    = 0x0001,//比赛状态数据
	ID_game_result 	   				    = 0x0002,//比赛结果数据
	ID_game_robot_survivors       = 0x0003,//比赛机器人存活数据
	ID_event_data  					      = 0x0101 ,//场地事件数据 
	ID_supply_projectile_action   = 0x0102,//场地补给站动作标识数据
	ID_supply_projectile_booking 	= 0x0103,//场地补给站预约子弹数据
	ID_game_robot_state    			  = 0x0201,//机器人状态数据
	ID_power_heat_data    			  = 0x0202,//实时功率热量数据
	ID_game_robot_pos        		  = 0x0203,//机器人位置数据
	ID_buff_musk					        = 0x0204,//机器人增益数据
	ID_aerial_robot_energy			  = 0x0205,//空中机器人能量状态数据
	ID_robot_hurt					        = 0x0206,//伤害状态数据
	ID_shoot_data					        = 0x0207,//实时射击数据

} CmdID;

//命令码数据段长度
typedef enum
{	
	LEN_game_state       				=  3,	//0x0001
	LEN_game_result       				=  1,	//0x0002
	LEN_game_robot_survivors       		=  2,	//0x0003
	LEN_event_data  					=  4,	//0x0101
	LEN_supply_projectile_action        =  3,	//0x0102
	LEN_supply_projectile_booking		=  2,	//0x0103
	LEN_game_robot_state    			= 15,	//0x0201
	LEN_power_heat_data   				= 14,	//0x0202
	LEN_game_robot_pos        			= 16,	//0x0203
	LEN_buff_musk        				=  1,	//0x0204
	LEN_aerial_robot_energy        		=  3,	//0x0205
	LEN_robot_hurt        				=  1,	//0x0206
	LEN_shoot_data       				=  6,	//0x0207	
}JudgeDataLength;


//裁判系统标志位接收(用于校验和调试)
typedef struct
{
	//帧头
	uint8_t sof;//首地址
	uint16_t data_lenth;//数据帧的data长度
	uint8_t seq; //帧序号
	uint8_t crc8;//CRC8校验位
	//命令码存放
	uint16_t cmdid;//命令码
	//下一帧数据
	uint8_t  next_frame_header;//下一帧数据头
	//数据尾
	uint16_t crc16;//CRC16校验位
}stoJudgeRecvData_t;

/***************裁判系统接收数据(2019裁判系统)********************/

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    比赛机器人存活数据 */
typedef __packed struct 
{ 
	uint16_t robot_legion;
} ext_game_robot_survivors_t; 


/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
} ext_supply_projectile_action_t; 


/* ID: 0X0103  Byte:  2    场地补给站预约子弹数据 */
typedef __packed struct 
{ 
	uint8_t supply_projectile_id;    
	uint8_t supply_num;  
} ext_supply_projectile_booking_t; 


/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct 
{ 
	uint8_t robot_id;                           //机器人ID，可用来校验发送
	uint8_t robot_level;                       //1一级，2二级，3三级
	uint16_t remain_HP;                       //机器人剩余血量
	uint16_t max_HP;                         //机器人满血量
	uint16_t shooter_heat0_cooling_rate;     //机器人 17mm 子弹热量冷却速度 单位 /s
	uint16_t shooter_heat0_cooling_limit;   // 机器人 17mm 子弹热量上限
	uint16_t shooter_heat1_cooling_rate;    
	uint16_t shooter_heat1_cooling_limit;   
	uint8_t mains_power_gimbal_output : 1;  //裁判系统电源模块输出标志位 1为有输出
	uint8_t mains_power_chassis_output : 1;  
	uint8_t mains_power_shooter_output : 1; 
} ext_game_robot_state_t; 


/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   //底盘输出电压
	uint16_t chassis_current;//地盘输出电流    
	float chassis_power;   //瞬时功率 
	uint16_t chassis_power_buffer;//60焦耳缓冲能量
	uint16_t shooter_heat0;//17mm枪管热量
	uint16_t shooter_heat1;//42mm枪管热量  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct  //10HZ
{   
	float x;              //单位m
	float y;              //单位m
	float z;              //单位m
	float yaw;            //枪口位置 单位度
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t power_rune_buff; 
} ext_buff_musk_t; 


/* ID: 0x0205  Byte:  3    空中机器人能量状态数据 */
typedef __packed struct 
{ 
	uint8_t energy_point;
	uint8_t attack_time; 
} aerial_robot_energy_t; 


/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef __packed struct 
{ 
	uint8_t armor_id : 4;  //bit0-3 产生伤害装甲ID
	uint8_t hurt_type : 4; //bit4-7 伤害类型 0x0装甲伤害 0X1模块掉线伤害 0x2超枪口热量扣血 0x3 超底盘功率扣血
} ext_robot_hurt_t; 


/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;  //射击子弹类型1：17mm弹丸 2；42mm弹丸 
	uint8_t bullet_freq;  //子弹射频 HZ 
	float bullet_speed;   //子弹射速 m/s
} ext_shoot_data_t; 

/*开火信息结构体（未用）*/
typedef struct
{
	uint8_t bullet_type;
	uint8_t fire_frequence;
	uint16_t tiny_shooter_heat;
	uint16_t thick_shooter_heat;
	float   bullet_speed;	
}sto_fire_info_t;



extern uint8_t Jud_Rx_Buffer[JUD_RC_BUFFER_SIZE];//接收缓冲数组
extern uint8_t Judge_Buffer[JUD_RC_BUFFER_SIZE];//视觉接收数据
/*--------裁判数据读取--------*/
void JudgeRead(uint8_t *RecvUsartData);//裁判系统接收
void HAL_UART6_Receive_IT_IDLE(void);//串口3空闲中断开启
/*--------裁判数据调用--------*/
float RoboChassisHeat(void);//功率读取
uint16_t RoboHP(void);//机器人血量读取
float FireSpeed(void);//射速
uint16_t FireHeat17(void);//17mm热量
uint8_t FireFeq(void);//射频
uint16_t FireNum(void);//统计发弹量
uint16_t FireCoolingLimit(void);//枪口热量冷却上限
uint16_t FireCoolingRate(void);//枪口热量每秒冷却值
/*--------参数打印--------*/
void ShowJudgeMeassge(void);//JSCOOP参数打印
#endif
