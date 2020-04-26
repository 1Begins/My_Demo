#include "Driver_Judge.h"
#include "Vision_interact.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"

stoVisonRecvData_t  VisonRecvData;//视觉接收结构体

traVisonHeader_t VisonHeader;//帧头

traVisonSendData_t VisonSendData;//发送数据

FloatTrans BTF;//字节转浮点数

uint8_t Tx_Buffer[SE_BUFFER_SIZE] ={0};//发送缓冲数组
uint8_t Rx_Buffer[RC_BUFFER_SIZE] ={0};//接收缓冲数组
uint8_t Visoin_Buffer[RC_BUFFER_SIZE]={0};//视觉接收缓冲数组




uint16_t USART_RX_STA = 0;  //接收状态标记

uint32_t Vision_ping[2]={0};//间隔计数存储数组
uint32_t count_time;//帧计数
uint32_t time_ping;//TIM3计数值100us

uint8_t SEND_FALG=1;//视觉数据发送标志位
uint8_t vision_update;//视觉更新标志位

/*-----------------------------------------------------------------------
\@brief <接收视觉数据>
\@param param：视觉接收缓冲数组
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void vision_connect(uint8_t *RecvUsartData)
{
	//判断帧头首字节是否为0XA5
	if(Rx_Buffer[0] == VIOSN_SOF)
	{
		//CRC8帧头校验
		if(Verify_CRC8_Check_Sum(RecvUsartData, VISON_LEN_HAEDER) ==TRUE)
		{
			//CRC16帧尾校验
			if(Verify_CRC16_Check_Sum(RecvUsartData, VISON_LEN_PACKED) ==TRUE)
			{
				  //拷贝接收数据到结构体中        
          memcpy(&VisonRecvData,RecvUsartData,VISON_LEN_PACKED);
				  //高低8为数据转换
		      rev_shrort_data();
				  //从高到低传
//          BTF.U[3] = Rx_Buffer[7];
//					BTF.U[2] = Rx_Buffer[6];
//					BTF.U[1] = Rx_Buffer[5];
//					BTF.U[0] = Rx_Buffer[4];
//          PC_interact_Rc_data.data1 =BTF.F;
//				    Rc_time_flag=!Rc_time_flag;
				    count_time=ping_count();//简单帧技术
			}
			else
			{
			 printf("Vreify_crc16_error!");
			}
		}
		else
		{
			printf("Vreify_crc8_error!");
		}
	}
	else
	{
		printf("Vreify_header_error!");
	}	
}
/*-----------------------------------------------------------------------
\@brief <发送数据给视觉>
\@param 无
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void sendVisionData()
{
	
	VisonHeader.SOF=VIOSN_SOF;
	VisonHeader.seq=VISON_SEQ;
	//写入帧头数据
	memcpy(Tx_Buffer,&VisonHeader,VISON_LEN_HAEDER);
	//写入CRC8校验码
	Append_CRC8_Check_Sum(Tx_Buffer, VISON_LEN_HAEDER);//用APPEND函数校验为加到数值中不要用Get_CRC8函数
	
	VisonSendData.visonYawData=235;
	VisonSendData.visonPitchData=-233;
	//写入数据
	memcpy(Tx_Buffer + VISON_LEN_HAEDER,&VisonSendData,VISON_LEN_PACKED);
	//写入CRC16校验码
	Append_CRC16_Check_Sum(Tx_Buffer,VISON_LEN_PACKED);
	
	HAL_UART_Transmit_DMA(&huart1,Tx_Buffer,VISON_LEN_PACKED);
  //数据清零
	HAL_Delay(1);//不加延时清零会使发送异常，大概是因为DMA发送有时间延时
	memset(Tx_Buffer,0,11);
	
}
/*-----------------------------------------------------------------------
\@brief 将四字节浮点型数据。转换为一字节送到数组中储存
\@param param1:目标浮点数，param2：存储数组，param3：开始存储的位数
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void Float_to_Byte(float *target,unsigned char *buf,unsigned char beg)
{
	  unsigned char *point;
    point = (unsigned char*)target;
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}
/*-----------------------------------------------------------------------
\@brief 用于读取视觉的16位像素偏差数据高低位转换
\@param param：无
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void rev_shrort_data(void)
{
	int16_t transfer_data;
	
	transfer_data=VisonRecvData.visonYawData >> 8;//右移得到高8位
	
	VisonRecvData.visonYawData= VisonRecvData.visonYawData <<8;//左移低8位赋值给高八位
	
	VisonRecvData.visonYawData = VisonRecvData.visonYawData | transfer_data;//进行一次或运算

  
}

/*-----------------------------------------------------------------------
\@brief 简单写的视觉帧计数
\@param 无
\@return 无
\@reference 
*--------------------------------------------------------------------*/
uint16_t ping_count()
{
	uint16_t ping;
	
	Vision_ping[NOW_t]=time_ping/10;
  ping=Vision_ping[NOW_t]-Vision_ping[LAST_t];
	Vision_ping[LAST_t]=Vision_ping[NOW_t];
	
	return ping;
}
/*-----------------------------------------------------------------------
\@brief 视觉云台YAW像素更新
\@param 偏差指针
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void Vision_Yaw_Error(int16_t *Yaw_Error)
{
	if(VisonRecvData.visonYawData !=0)
	{
		*Yaw_Error=VisonRecvData.visonYawData;
	}
	else
	{
		*Yaw_Error=0;
	}
}
/*-----------------------------------------------------------------------
\@brief 串口1DMA初始化
\@param 无
\@return 无
\@reference 
*--------------------------------------------------------------------*/
void HAL_UART1_Receive_IT_IDLE()
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//开启空闲中断
  HAL_UART_Receive_DMA(&huart1,Rx_Buffer,RC_BUFFER_SIZE);//开启DMA接收
}


/*--------上赛季旧版本裁判系统读取--------*/
/*-----------------------------------------------------------------------
//上赛季的裁判系统读取采用的是数组位赋值的方法对数据进行每一位的更新
   能够对每一位的数据进行操作。但因为用了大量标志位和全局变量，对于后续的
   协议修改不友好。
//但因为其可以对每一位进行操作所以可以用作协议修改时候的测试。
*--------------------------------------------------------------------*/

//测试
#if  PROTOCOL_TEST==1
PC_interact_data_set PC_interact_data;//发送数据结构体

PC_interact_Rc_data_set PC_interact_Rc_data;//接收数据结构体

uint8_t usart1_Recv_End_flag = 0;//串口接收完成标志位
uint16_t usart1_Rx_len = 0;//串口接收数据长度


//PC交互发送协议测试
void PC_interact_Txdata_set(void)
{
     
		Tx_Buffer[0] = 0xA5;
		Tx_Buffer[1] = 2;
    Append_CRC8_Check_Sum(Tx_Buffer, 3);//用APPEND函数校验为加到数值中不要用Get_CRC8函数
//	  Tx_Buffer[3] = 0x12;
//		Tx_Buffer[4] = 0x34;
//		Tx_Buffer[5] = 0x56;
//		Tx_Buffer[6] = 0x78;
	  Float_to_Byte(&PC_interact_data.data1,Tx_Buffer,3);
    Append_CRC16_Check_Sum(Tx_Buffer, 9);
}

void PC_interact_data_send_test(float Data_a,float Data_b, float Data_c)
{
  PC_interact_data.data1=Data_a;
//	PC_interact_data.data2=Data_b;
//	PC_interact_data.data3=Data_c;
	PC_interact_Txdata_set();
	HAL_UART_Transmit_DMA(&huart1,Tx_Buffer,9);
	
}

void PC_interact_Rc_data_test(void)
{
  uint32_t tmp_flag = 0;
	uint32_t temp;

    
  tmp_flag=__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE);

	if((tmp_flag != RESET))//检测到DMA线路空闲
	{
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//通过读取SR,DR寄存器去清零。
		temp=__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//读取DMA剩余传输量
    HAL_UART_DMAStop(&huart1);//关闭DMA接收 
    if(temp==RC_BUFFER_SIZE-28)//检查是否溢出
      { 
        
        Verify_crc16=Verify_CRC16_Check_Sum(Rx_Buffer,28);
        if (Rx_Buffer[5] == 01 &&Rx_Buffer[6] == 03 && Verify_CRC16_Check_Sum(Rx_Buffer, 28))//自定义数据校验&& Verify_CRC16_Check_Sum(Rx_Buffer, 28)
        {
            
           /*加上"&& Verify_CRC16_Check_Sum(Rx_Buffer, 28)"校验返回值后不能进入判断读数，因为在USART1_IRQHandler中断处理函数中串口打印出的Verify_temp
            值为0显示解算失败。*/
             
            //从高到低传
             
          BTF.U[3] = Rx_Buffer[16];
					BTF.U[2] = Rx_Buffer[15];
					BTF.U[1] = Rx_Buffer[14];
					BTF.U[0] = Rx_Buffer[13];
          PC_interact_Rc_data.data1 =BTF.F;
          BTF.U[3] = Rx_Buffer[20];
					BTF.U[2] = Rx_Buffer[19];
					BTF.U[1] = Rx_Buffer[18];
					BTF.U[0] = Rx_Buffer[17];
					PC_interact_Rc_data .data2  = BTF.F;
					BTF.U[3] = Rx_Buffer[24];
					BTF.U[2] = Rx_Buffer[23];
					BTF.U[1] = Rx_Buffer[22];
					BTF.U[0] = Rx_Buffer[21];
					PC_interact_Rc_data .data3  = BTF.F;
          PC_interact_Rc_data .masks = Rx_Buffer[25];
        
        }
//			memset(Rx_Buffer,0,100);
        usart1_Rx_len =RC_BUFFER_SIZE-temp;//接收数据长度
        usart1_Recv_End_flag=1;
        Rc_time_flag=!Rc_time_flag;
			  count_time=ping_count();
      }        
	}
}
void PC_interact_Rc_data_Replay(void)
{
    if(usart1_Recv_End_flag ==1)
	{
		    
		
        usart1_Rx_len=0;
        usart1_Recv_End_flag=0;
        HAL_UART_Receive_DMA(&huart1,Rx_Buffer,RC_BUFFER_SIZE);//重启DMA接收
		
	}
}
#endif
