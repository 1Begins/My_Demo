#include "Driver_Judge.h"
#include "Vision_interact.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"

stoVisonRecvData_t  VisonRecvData;//�Ӿ����սṹ��

traVisonHeader_t VisonHeader;//֡ͷ

traVisonSendData_t VisonSendData;//��������

FloatTrans BTF;//�ֽ�ת������

uint8_t Tx_Buffer[SE_BUFFER_SIZE] ={0};//���ͻ�������
uint8_t Rx_Buffer[RC_BUFFER_SIZE] ={0};//���ջ�������
uint8_t Visoin_Buffer[RC_BUFFER_SIZE]={0};//�Ӿ����ջ�������




uint16_t USART_RX_STA = 0;  //����״̬���

uint32_t Vision_ping[2]={0};//��������洢����
uint32_t count_time;//֡����
uint32_t time_ping;//TIM3����ֵ100us

uint8_t SEND_FALG=1;//�Ӿ����ݷ��ͱ�־λ
uint8_t vision_update;//�Ӿ����±�־λ

/*-----------------------------------------------------------------------
\@brief <�����Ӿ�����>
\@param param���Ӿ����ջ�������
\@return ��
\@reference 
*--------------------------------------------------------------------*/
void vision_connect(uint8_t *RecvUsartData)
{
	//�ж�֡ͷ���ֽ��Ƿ�Ϊ0XA5
	if(Rx_Buffer[0] == VIOSN_SOF)
	{
		//CRC8֡ͷУ��
		if(Verify_CRC8_Check_Sum(RecvUsartData, VISON_LEN_HAEDER) ==TRUE)
		{
			//CRC16֡βУ��
			if(Verify_CRC16_Check_Sum(RecvUsartData, VISON_LEN_PACKED) ==TRUE)
			{
				  //�����������ݵ��ṹ����        
          memcpy(&VisonRecvData,RecvUsartData,VISON_LEN_PACKED);
				  //�ߵ�8Ϊ����ת��
		      rev_shrort_data();
				  //�Ӹߵ��ʹ�
//          BTF.U[3] = Rx_Buffer[7];
//					BTF.U[2] = Rx_Buffer[6];
//					BTF.U[1] = Rx_Buffer[5];
//					BTF.U[0] = Rx_Buffer[4];
//          PC_interact_Rc_data.data1 =BTF.F;
//				    Rc_time_flag=!Rc_time_flag;
				    count_time=ping_count();//��֡����
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
\@brief <�������ݸ��Ӿ�>
\@param ��
\@return ��
\@reference 
*--------------------------------------------------------------------*/
void sendVisionData()
{
	
	VisonHeader.SOF=VIOSN_SOF;
	VisonHeader.seq=VISON_SEQ;
	//д��֡ͷ����
	memcpy(Tx_Buffer,&VisonHeader,VISON_LEN_HAEDER);
	//д��CRC8У����
	Append_CRC8_Check_Sum(Tx_Buffer, VISON_LEN_HAEDER);//��APPEND����У��Ϊ�ӵ���ֵ�в�Ҫ��Get_CRC8����
	
	VisonSendData.visonYawData=235;
	VisonSendData.visonPitchData=-233;
	//д������
	memcpy(Tx_Buffer + VISON_LEN_HAEDER,&VisonSendData,VISON_LEN_PACKED);
	//д��CRC16У����
	Append_CRC16_Check_Sum(Tx_Buffer,VISON_LEN_PACKED);
	
	HAL_UART_Transmit_DMA(&huart1,Tx_Buffer,VISON_LEN_PACKED);
  //��������
	HAL_Delay(1);//������ʱ�����ʹ�����쳣���������ΪDMA������ʱ����ʱ
	memset(Tx_Buffer,0,11);
	
}
/*-----------------------------------------------------------------------
\@brief �����ֽڸ��������ݡ�ת��Ϊһ�ֽ��͵������д���
\@param param1:Ŀ�긡������param2���洢���飬param3����ʼ�洢��λ��
\@return ��
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
\@brief ���ڶ�ȡ�Ӿ���16λ����ƫ�����ݸߵ�λת��
\@param param����
\@return ��
\@reference 
*--------------------------------------------------------------------*/
void rev_shrort_data(void)
{
	int16_t transfer_data;
	
	transfer_data=VisonRecvData.visonYawData >> 8;//���Ƶõ���8λ
	
	VisonRecvData.visonYawData= VisonRecvData.visonYawData <<8;//���Ƶ�8λ��ֵ���߰�λ
	
	VisonRecvData.visonYawData = VisonRecvData.visonYawData | transfer_data;//����һ�λ�����

  
}

/*-----------------------------------------------------------------------
\@brief ��д���Ӿ�֡����
\@param ��
\@return ��
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
\@brief �Ӿ���̨YAW���ظ���
\@param ƫ��ָ��
\@return ��
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
\@brief ����1DMA��ʼ��
\@param ��
\@return ��
\@reference 
*--------------------------------------------------------------------*/
void HAL_UART1_Receive_IT_IDLE()
{
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);//���������ж�
  HAL_UART_Receive_DMA(&huart1,Rx_Buffer,RC_BUFFER_SIZE);//����DMA����
}


/*--------�������ɰ汾����ϵͳ��ȡ--------*/
/*-----------------------------------------------------------------------
//�������Ĳ���ϵͳ��ȡ���õ�������λ��ֵ�ķ��������ݽ���ÿһλ�ĸ���
   �ܹ���ÿһλ�����ݽ��в���������Ϊ���˴�����־λ��ȫ�ֱ��������ں�����
   Э���޸Ĳ��Ѻá�
//����Ϊ����Զ�ÿһλ���в������Կ�������Э���޸�ʱ��Ĳ��ԡ�
*--------------------------------------------------------------------*/

//����
#if  PROTOCOL_TEST==1
PC_interact_data_set PC_interact_data;//�������ݽṹ��

PC_interact_Rc_data_set PC_interact_Rc_data;//�������ݽṹ��

uint8_t usart1_Recv_End_flag = 0;//���ڽ�����ɱ�־λ
uint16_t usart1_Rx_len = 0;//���ڽ������ݳ���


//PC��������Э�����
void PC_interact_Txdata_set(void)
{
     
		Tx_Buffer[0] = 0xA5;
		Tx_Buffer[1] = 2;
    Append_CRC8_Check_Sum(Tx_Buffer, 3);//��APPEND����У��Ϊ�ӵ���ֵ�в�Ҫ��Get_CRC8����
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

	if((tmp_flag != RESET))//��⵽DMA��·����
	{
		
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);//ͨ����ȡSR,DR�Ĵ���ȥ���㡣
		temp=__HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//��ȡDMAʣ�ഫ����
    HAL_UART_DMAStop(&huart1);//�ر�DMA���� 
    if(temp==RC_BUFFER_SIZE-28)//����Ƿ����
      { 
        
        Verify_crc16=Verify_CRC16_Check_Sum(Rx_Buffer,28);
        if (Rx_Buffer[5] == 01 &&Rx_Buffer[6] == 03 && Verify_CRC16_Check_Sum(Rx_Buffer, 28))//�Զ�������У��&& Verify_CRC16_Check_Sum(Rx_Buffer, 28)
        {
            
           /*����"&& Verify_CRC16_Check_Sum(Rx_Buffer, 28)"У�鷵��ֵ���ܽ����ж϶�������Ϊ��USART1_IRQHandler�жϴ������д��ڴ�ӡ����Verify_temp
            ֵΪ0��ʾ����ʧ�ܡ�*/
             
            //�Ӹߵ��ʹ�
             
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
        usart1_Rx_len =RC_BUFFER_SIZE-temp;//�������ݳ���
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
        HAL_UART_Receive_DMA(&huart1,Rx_Buffer,RC_BUFFER_SIZE);//����DMA����
		
	}
}
#endif
