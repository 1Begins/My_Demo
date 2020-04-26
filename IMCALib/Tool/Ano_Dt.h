#ifndef __ANO_DT_
#define __ANO_DT_

#include "usart.h"
#include "mytype.h"

extern uint8_t Data_Send_ANO_DT;


//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

/*********************************************************测试用*****************************************************************/
void Data_Send_User(u16 data1,u8 data2,s16 data3);
void DataSendS16(float data1,float data2);
/**********************************************************END*******************************************************************/

/*通过串口发送数据到匿名地面站*/
void ANO_DT_Send_Data(uint8_t *data, uint8_t len);
/*更新发送到匿名地面站的数据*/
void ANO_DT_DataUpdate(void);

/*************************************************用户自定义数据发送函数*********************************************************/
/*发送四个float型数据到匿名地面站*/
void DataSendFloat(float data1, float data2, float data3, float data4);
/*发送四个uint8_t型数据到匿名地面站*/
void DataSendUint8(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4);
/*发送四个int16_t型数据到匿名地面站*/
void DataSendInt16(int16_t data1, int16_t data2, int16_t data3, int16_t data4);
/*发送四个uint16_t型数据到匿名地面站*/
void DataSendUint16(uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4);
/*发送四个int32_t型数据到匿名地面站*/
void DataSendInt32(int32_t data1, int32_t data2, int32_t data3, int32_t data4);
/********************************************************END********************************************************************/


#endif

