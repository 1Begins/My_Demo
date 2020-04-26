#ifndef __APPINERACTION_
#define __APPINERACTION_

#include "mytype.h"
#include "usmart.h"


#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define RXBUFFERSIZE   1 //缓存大小
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;      //接收状态标记	
extern u8 aRxBuffer[RXBUFFERSIZE];  //HAL库USART接收Buffer


/*USMART调试组件初始化*/
void APPInteractionInit(void);
/*超时处理，重启串口1接收中断*/
void Reset_Usart1_Receive_IT(void);
/*USMART扫描，100ms执行一次，在TIM7_IRQHandler中被调用*/
void UsmartScan(void);


#endif

