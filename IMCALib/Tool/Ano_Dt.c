/**********************************************************************************************************************
 *@brief   该文件是用于MCU和匿名地面站的通信，通过无线串口将数据发送到匿名地面站上进行波形显示。波形显示最好还是用
 *         J-Scope软件比较好，但是必须是有线的，当需要查看车运动时的数据波形时可以通过无线串口发送数据到匿名地面站。
 *
 *
**********************************************************************************************************************/

#include "Ano_Dt.h"
#include "CanBus_Task.h"
#include "Chassis_Task.h"
#include "ControlTask.h"
#include "usart.h"
#include "IMU_Task.h"
#include "Gimbal_Task.h"


uint8_t Data_Send_ANO_DT;
uint8_t data_to_send[50];	//发送数据缓存
uint8_t DMA_aTxBuffer[] = {10};       // 串口DMA发送缓冲区


/*测试用*/
void Data_Send_User(u16 data1,u8 data2,s16 data3)
{
    u8 _cnt=0,sum = 0,i;

	data_to_send[_cnt++]=0xAA;           //帧头AAAA
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1;           //功能字
	data_to_send[_cnt++]=0;              //数据长度

	data_to_send[_cnt++]=BYTE1(data1);
	data_to_send[_cnt++]=BYTE0(data1);
	
	data_to_send[_cnt++]=data2;
	
	data_to_send[_cnt++]=BYTE1(data3);
	data_to_send[_cnt++]=BYTE0(data3);

	data_to_send[3] = _cnt-4;            //数据长度赋值

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];            //数据校验求解
		
	data_to_send[_cnt++] = sum;          //数据校验赋值
	
	ANO_DT_Send_Data(data_to_send, _cnt);//帧发送到上位机
    
}



void DataSendS16(float data1,float data2)
{
    u8 _cnt=0,sum = 0,i;

    data_to_send[_cnt++]=0xAA;           //帧头AAAA
    data_to_send[_cnt++]=0xAA;
    data_to_send[_cnt++]=0xF1;           //功能字
    data_to_send[_cnt++]=0;              //数据长度

    data_to_send[_cnt++]=BYTE3(data1);
    data_to_send[_cnt++]=BYTE2(data1);
    data_to_send[_cnt++]=BYTE1(data1);
    data_to_send[_cnt++]=BYTE0(data1);

    data_to_send[_cnt++]=BYTE3(data2);
    data_to_send[_cnt++]=BYTE2(data2);
    data_to_send[_cnt++]=BYTE1(data2);
    data_to_send[_cnt++]=BYTE0(data2);


    data_to_send[3] = _cnt-4;            //数据长度赋值

    for(i=0;i<_cnt;i++)
    sum += data_to_send[i];            //数据校验求解

    data_to_send[_cnt++] = sum;          //数据校验赋值

    ANO_DT_Send_Data(data_to_send, _cnt);//帧发送到上位机
    
}


/*通过串口发送数据到匿名地面站*/
void ANO_DT_Send_Data(uint8_t *data, uint8_t len)
{
    
    uint8_t i;
	
	for(i=0;i<len;i++)
	{
        
        HAL_UART_Transmit(&huart4,data+i,1,10);
        
//		while(USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);//发送完成中断，用于检查串口UART1是否发送完成
//        while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC) == RESET);
        
	}
    
}


/*更新发送到匿名地面站的数据*/
void ANO_DT_DataUpdate(void)
{
    
//    DataSendFloat(Chassis.upper_right_motor_201, Chassis_Motor[UPPER_RIGHT_201].speed_rpm/19.2, 0, 0);
    /*底盘四个电机对应的转速*/
    
//    DataSendFloat(Chassis_Motor[FRON_RIGH_201].speed_rpm/19.2, Chassis_Motor[FRON_LEFT_202].speed_rpm/19.2, \
//                  Chassis_Motor[REAR_LEFT_203].speed_rpm/19.2, Chassis_Motor[REAR_RIGH_204].speed_rpm/19.2);
    
//    DataSendFloat(Gimbal[YAW].set_angle_imu, IMU_Yaw.total_angle, 0, 0);
    
    DataSendInt16(-GimbalData[0].relative_angle, 0, 0, 0);
    
}


/*发送四个float型数据到匿名地面站*/
void DataSendFloat(float data1, float data2, float data3, float data4)
{
    
    u8 _cnt=0,sum = 0,i;
	float _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
	
	_temp = data1;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//发送数据到匿名上位机
    
}


/*发送四个uint8_t型数据到匿名地面站*/
void DataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	uint8_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
    data_to_send[_cnt++]=_temp;
    _temp = data2;
    data_to_send[_cnt++]=_temp;
    _temp = data3;
    data_to_send[_cnt++]=_temp;
    _temp = data4;
    data_to_send[_cnt++]=_temp;
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//发送数据到匿名上位机
    
}


/*发送四个int16_t型数据到匿名地面站*/
void DataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	int16_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//发送数据到匿名上位机
    
}


/*发送四个uint16_t型数据到匿名地面站*/
void DataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	uint16_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//发送数据到匿名上位机
    
}


/*发送四个int32_t型数据到匿名地面站*/
void DataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4)
{
    
    u8 _cnt=0,sum = 0,i;
	int32_t _temp;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF1; 
	data_to_send[_cnt++]=0;
    
    _temp = data1;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data2;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data3;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = data4;
	data_to_send[_cnt++]=BYTE3(_temp);
	data_to_send[_cnt++]=BYTE2(_temp);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
    
    data_to_send[3] = _cnt-4;

	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
		
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);//发送数据到匿名上位机
    
}

