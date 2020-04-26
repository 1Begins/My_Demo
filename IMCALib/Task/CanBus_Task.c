/**********************************************************************************************************************
 * @file  CanBus_Task.c
 * @brief CAN1�˲��������ú�CAN�Ŀ�������CAN�����Ͻ��ձ��ĺͷ��ͱ��ĵ�CAN������
 *
**********************************************************************************************************************/

#include "CanBus_Task.h"
#include "Gimbal_Task.h"


moto_measure_t  Chassis_Motor_Encoder[4];  //���̵�������ṹ��<can2>
moto_measure_t  Gimbal_Motor_Encoder[6];	 //��̨��������ṹ��<can1>

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;


/**********************************************************************************************************************
  * @Func	 can_filter_init
  * @Brief   CAN1��CAN2�˲��������úͿ���CAN
  * @Param	 CAN_HandleTypeDef* _hcan
  * @Retval	 None
**********************************************************************************************************************/
void CANFilterInit(void)
{

	CAN_FilterTypeDef		CAN_FilterConfigStructure;

	/*filter config for can1*/
	CAN_FilterConfigStructure.FilterBank = 0;                      // filter 0
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;  // mask mode
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;            // set mask 0 to receive all can id
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0
	CAN_FilterConfigStructure.FilterActivation = ENABLE;           // enable can filter
	CAN_FilterConfigStructure.SlaveStartFilterBank = 14;           // only meaningful in dual can mode
	
	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterConfigStructure);       // init can filter
	HAL_CAN_Start(&hcan1);                                         // start can1
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);  // enable can1 rx interrupt
	
	/*filter config for can2*/ 
	/*can1(0-13)��can2(14-27)�ֱ�õ�һ���filter*/
	CAN_FilterConfigStructure.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &CAN_FilterConfigStructure);
	HAL_CAN_Start(&hcan2); 
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);

}



/**********************************************************************************************************************
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
 *********************************************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
		
	CAN_RxHeaderTypeDef   rx_header;
	uint8_t               rx_data[8];

	//igonre can1 or can2
	HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data); /*recive can data*/
		
	switch(rx_header.StdId)
    {
			//���̵��ID�ѱ�����
			case CAN_3508Moto1_ID       :
			case CAN_3508Moto2_ID     	:  	
			case CAN_3508Moto3_ID     	:
			case CAN_3508Moto4_ID     	:
			case CAN_PLUCK_Motor_ID     :
			case CAN_PUSHROD_Motor_ID   :
			case CAN_FRICTION1_Motor_ID :  
			case CAN_FRICTION2_Motor_ID :
					{
						static u8 i;
						
						if(_hcan->Instance == CAN1){
							i = rx_header.StdId - CAN_PUSHROD_Motor_ID;			
							can1_count++;
							Gimbal_Motor_Encoder[i].msg_cnt++ <= 50 ? GetMotorOffset(&Gimbal_Motor_Encoder[i], rx_data) : GetMotorMeasure(&Gimbal_Motor_Encoder[i], rx_data);  /*��ȡ���̵��������Ϣ*/
						}
						
						if(_hcan->Instance == CAN2){
							i = rx_header.StdId - CAN_3508Moto1_ID;	
							can2_count++;
							if( i < 4 )  //����201��202��203��204
								Chassis_Motor_Encoder[i].msg_cnt++ <= 50 ? GetMotorOffset(&Chassis_Motor_Encoder[i], rx_data) : GetMotorMeasure(&Chassis_Motor_Encoder[i], rx_data);  /*��ȡ���̵��������Ϣ*/
							else if( i < 6 )   //��̨205��206
								Gimbal_Motor_Encoder[i].msg_cnt++ <= 50 ? GetMotorOffset(&Gimbal_Motor_Encoder[i], rx_data) : GetMotorMeasure(&Gimbal_Motor_Encoder[i], rx_data);  /*��ȡ���̵��������Ϣ*/

						}
						break;
					}
		}
		
	if(can1_count == 500)  /*����ָʾCAN1ͨ���Ƿ�����*/
	{
		can1_count = 0;
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);  
	}
	if(can2_count == 500)  /*����ָʾCAN2ͨ���Ƿ�����*/
	{
		can2_count = 0;
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);  
	}

}


/**********************************************************************************************************************
 * @brief    ����3508���ͨ��CAN����������Ϣ��2006���Ҳ���ã�����2006���û���¶�ֵ����
 * @param	 moto_measure_t *ptr����������ṹ��
 *           uint8_t can_rx_data[]��CAN�������ݻ�����
 * @retval	 None
**********************************************************************************************************************/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[])
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]);
	ptr->speed_rpm  = (int16_t)(can_rx_data[2]<<8 | can_rx_data[3]);
	ptr->real_current = (can_rx_data[4]<<8 | can_rx_data[5])*5.f/16384.f;
	ptr->hall = can_rx_data[6];
    
	/*����������㴦��*/
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
    
    /*�õ�ת�����ܽǶȣ�����Ƕ���������ϵ�ʱ�ĽǶ�*/
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle; 
    
}

/*this function should be called after system+can init */
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[])        
{
	ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]) ;
	ptr->offset_angle = ptr->angle;
}

/**********************************************************************************************************************
 * @Brief   ��ȡyaw��������ڳ�ǰ���е�ı����������ֵ��ת��Ϊ�Ƕȶ�ӦΪ-180�� ~ 180�ȣ�0�ȶ�Ӧ���ǵ���ǰ���������
 * @Param   None
 * @Retval  ��̨yaw������ڵ���ǰ��ĵ����е�Ƕ�ֵ����ΧΪ-4096 ~ 4096�����Ϊ�����ұ�Ϊ��
 * @Other   ��ΪҪ������������Ի�е�Ƕ�ֵ��PID���㣬������������Ի�е�Ƕ�ֵ������Ҫ���ݵ����ת�����趨
**********************************************************************************************************************/
int16_t GetEncoderRelativeAngle(void)
{
    
    int16_t relative_angle = 0, relative_angle_out = 0;
    int8_t direction_flag = 0;
    
    relative_angle = Gimbal_Motor_Encoder[0].angle - CHASSIS_YAW_MID_FRONT; /*ֱ������*/
	
    if((CHASSIS_YAW_MID_FRONT < Gimbal_Motor_Encoder[0].angle) &&
			 (Gimbal_Motor_Encoder[0].angle < CHASSIS_YAW_MID_BACK)		) /*�����ж�*/
    {
        direction_flag = -1;
    }
    else
    {
        direction_flag = 1;
    }
    
    if(relative_angle < 0) /*ȡ����*/
    {
        relative_angle = -relative_angle;
    }
    
    if(relative_angle > 8192/2)
    {
        relative_angle_out = 8192 - relative_angle;
    }
    else
    {
        relative_angle_out = relative_angle;
    }
    
    return relative_angle_out*direction_flag;
    
}

/*�õ��ϵ�ʱ�ı���������ڵ���ǰ���ĵĻ�е�Ƕ�ֵ*/
int16_t GetEncoderOffsetRelativeAngle(void)
{
    
    int16_t relative_angle = 0, relative_angle_out = 0;
    int8_t direction_flag = 0;
    
    relative_angle = Gimbal_Motor_Encoder[YAW_CAN2_205].offset_angle - CHASSIS_YAW_MID_FRONT; /*ֱ������*/
    if((CHASSIS_YAW_MID_FRONT < Gimbal_Motor_Encoder[YAW_CAN2_205].offset_angle)&&(Gimbal_Motor_Encoder[YAW_CAN2_205].offset_angle < CHASSIS_YAW_MID_BACK)) /*�����ж�*/
    {
        direction_flag = -1;
    }
    else
    {
        direction_flag = 1;
    }
    
    if(relative_angle < 0) /*ȡ����*/
    {
        relative_angle = -relative_angle;
    }
    
    if(relative_angle > 8192/2)
    {
        relative_angle_out = 8192 - relative_angle;
    }
    else
    {
        relative_angle_out = relative_angle;
    }
    
    return relative_angle_out*direction_flag;
    
}


void position_send_out(CAN_HandleTypeDef *hcan, int32_t angleControl)
{
    
	int16_t pos_1,pos_2;
	pos_1 = angleControl>>16;
	pos_2 = angleControl;
	
	CAN_TxHeaderTypeDef   tx_header;
	uint8_t               tx_data[8];

	tx_header.StdId = 0x141;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = 0xA3;
	tx_data[1] = 0x00;
	tx_data[2] = 0x00;
	tx_data[3] = 0x00;
	tx_data[4] = pos_1>>8;
	tx_data[5] = pos_1;
	tx_data[6] = pos_2>>8;
	tx_data[7] = pos_2;
	
//	tx_data[4] = *(uint8_t *)(&angleControl);
//	tx_data[5] = *(uint8_t *)((&angleControl)+1);
//	tx_data[6] = *(uint8_t *)((&angleControl)+2);.
//	tx_data[7] = *(uint8_t *)((&angleControl)+3);
	
   HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}	

/**********************************************************************************************************************
 * @brief  send motor control message through can bus
 * @param  CAN_HandleTypeDef *hcan             CAN handle Structure
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  �����ĸ�����ĵ���ֵ
 * @retval None
 * @others ��ʶ��0x200��Ӧǰ�ĸ�ID�ĵ������ʶ��0x1FF��Ӧ���ĸ�ID�ĵ����һ·CAN�����ԽӰ˸������
 *         �˺����Ƕ�Ӧǰ4��ID�ĵ��
**********************************************************************************************************************/
void set_moto_current1(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    
	CAN_TxHeaderTypeDef   tx_header;
	uint8_t               tx_data[8];

	tx_header.StdId = 0x200;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
	tx_data[2] = (iq2 >> 8);
	tx_data[3] = iq2;
	tx_data[4] = (iq3 >> 8);
	tx_data[5] = iq3;
	tx_data[6] = (iq4 >> 8);
	tx_data[7] = iq4;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}	


/**********************************************************************************************************************
 * @brief  send motor control message through can bus
 * @param  CAN_HandleTypeDef *hcan             CAN handle Structure
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  �����ĸ�����ĵ���ֵ
 * @retval None
 * @others ��ʶ��0x200��Ӧǰ�ĸ�ID�ĵ������ʶ��0x1FF��Ӧ���ĸ�ID�ĵ����һ·CAN�����ԽӰ˸������
 *         �˺����Ƕ�Ӧ��4��ID�ĵ��,��0x205��ʼ�ĵ��ID
**********************************************************************************************************************/
void set_moto_current2(CAN_HandleTypeDef *hcan,s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{
    
    CAN_TxHeaderTypeDef   tx_header;
    uint8_t               tx_data[8];

	tx_header.StdId = 0x1FF;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 0x08;
	tx_data[0] = (iq1 >> 8);
	tx_data[1] = iq1;
	tx_data[2] = (iq2 >> 8);
	tx_data[3] = iq2;
	tx_data[4] = (iq3 >> 8);
	tx_data[5] = iq3;
	tx_data[6] = (iq4 >> 8);
	tx_data[7] = iq4;
	
    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}	
