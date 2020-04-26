#ifndef _VISION_INTERACT_H
#define _VISION_INTERACT_H
#include "stm32f4xx_hal.h"
#include "MYtype.h"

/*--------上位机协议--------*/
//关于官方裁判系统和视觉通讯的通讯协议的一些注意
//现在视觉的通讯协议为首地址为0XA5。SEQ帧序号为2，因为暂时为定长发送，所以无数据长度段
//要注意的是CRC8校验为一字节，CRC16为两字节。CRC8做头校验，CRC16做整包的数据校验
//---header-------------------------------------//---data----------------//------tail--------//
//--首地址---帧序号---模式选择-----CRC8校验位--//---两个浮点数位数据--//---CRC16校验位----//
//--0XA5-----2--------占一字节----占一字节----//---占八字节------------//----占两字节------//
/*--注意--*/
//写入 CRC8帧头校验码 Append_CRC8_Check_Sum(param1,param2)
//param1 为需要写入的数组，param2为CRC8写入后的数据长度。即为加上数据0XA5和SEQ帧序号位之后的数据长度为三，crc8校验码写在[2]里
//写入 CRC16整包校验码 Append_CRC16_Check_Sum(param1,param2)
//param1 为需要写入的数组，param2为写入后的整个数据长度。同理
//视觉发送下来的16位像素偏差数据高低八位反转，要进行一次转换。
#define    PROTOCOL_TEST  2

#define    NOW_t            1
#define    LAST_t           0

#define    SE_BUFFER_SIZE 20  //视觉发送数据缓冲字节
#define    RC_BUFFER_SIZE 100 //视觉接收数据缓冲字节
//起始字节， 为0XA5
#define    VIOSN_SOF 0XA5
//帧序号
#define    VISON_SEQ 2

//协议帧长度
#define    VISON_LEN_HAEDER 4 //帧头长
#define    VISON_LEN_PACKED 14//数据包长度
#define    VISON_SEND_LENGTH 14//视觉发送数据长度
//视觉模式发送
#define    VISION_BLUE     (0X01)
#define    VISION_RED      (0X02)
//视觉模式切换
#define    VISION_MANUAL       (0X00)
#define    VISION_AUTO         (0x01)
#define    VISION_HBUFF        (0X02)
#define    VISION_LBUFF        (0X03)
//反馈模式选择
//#define    FeedBack_px     1  //像素偏差反馈
//#define    FeedBack_angle  2  //角度偏差反馈
/*--------云台模式枚举--------*/
typedef enum
{
  FeedBack_px, 
  FeedBack_angle,
} vision_feedback_e;

//下位机接收结构体
typedef __packed struct
{
	//帧头
	uint8_t SOF;//首地址
	uint8_t seq; //帧序号
	uint8_t model;//模式
	uint8_t crc8;//CRC8校验位
	
	//数据
	float visionYawData;//视觉反馈YAW轴数据
	float visionPitchData;//视觉反馈PITCH轴数据
	//uint8_t VisionTrackFlag;//视觉是否识别到目标
	
	//数据尾
	uint16_t crc16;//CRC16校验位
}stoVisonRecvData_t;

//下位机发送结构体,帧头和数据段帧尾校验分开，因为在储存数据简单暂时不需要字节对齐
typedef  struct//只用作发送
{
  //帧头
	uint8_t SOF;//首地址
	uint8_t robotColor; //装甲板颜色识别 0x01 红装甲板 0x02 蓝色装甲板
	uint8_t model;//模式切换 0x01 自瞄， 0x02小符 0x03大符
	uint8_t crc8;//CRC8校验位
}traVisonHeader_t;
typedef  struct
{
	//自瞄数据
	float YawAngle;//YAW轴像素偏差
	float PitchAngle;//PITCH轴像素偏差
  
		//数据尾
	uint16_t crc16;//CRC16校验位
	
}traVisonSendData_t;



//格式转换联合体"联合体成员变量共享内存"
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FloatTrans;

//尽量减少无意义的全局变量


extern stoVisonRecvData_t  VisonRecvData;//视觉接收结构体




extern uint8_t Rx_Buffer[RC_BUFFER_SIZE];//接收缓冲数组
extern uint8_t Visoin_Buffer[RC_BUFFER_SIZE];//视觉接收数据

extern float  YawLast_T;
extern float  YawPreSpeed;
extern uint32_t count_time;//帧计数
extern uint32_t time_ping;//TIM3计数值100us
extern uint8_t SEND_FALG;//数据发送标志

/*--------视觉数据读发函数--------*/
void sendVisionData(uint8_t ModleID,uint8_t Cmd);//发送视觉数据
void vision_connect(uint8_t *RecvUsartData);//接收视觉数据
/*--------视觉像素偏差更新函数--------*/
float Vision_Yaw_Error(float *Yaw_Error);
void Vision_Pitch_Error(float *Pitch_Error);
/*--------数据类型操作函数--------*/
void Float_to_Byte(float *target,unsigned char *buf,unsigned char beg);
void rev_shrort_data(void);
/*--------像素帧计数--------*/
uint16_t ping_count(void);//
/*--------串口DMA配置--------*/
void HAL_UART3_Receive_IT_IDLE(void);//串口1DMA启动
/*-------视觉功能函数--------*/
bool_t Vision_UpDate(void);
void Vision_UpDate_Clean(void);
#if  PROTOCOL_TEST==1

extern PC_interact_Rc_data_set PC_interact_Rc_data;//接收3组浮点数
extern u8 usart1_Recv_End_flag;//接收完成标志位
extern uint8_t seq;//帧序号

typedef struct//发送数据结构体
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}PC_interact_data_set;

typedef struct//接收数据结构体
{
	float data1;
	float data2;
	float data3;
	uint8_t masks;
}PC_interact_Rc_data_set;

void PC_interact_data_send_test(float Data_a,float Data_b, float Data_c);
void PC_interact_Rc_data_test(void);
void PC_interact_Rc_data_Replay(void);//串口1接收重启

#endif


#endif
