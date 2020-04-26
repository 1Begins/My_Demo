/**********************************************************************************************************************
 * @file  CanBus_Task.c
 * @brief CAN1滤波器的配置和CAN的开启，从CAN总线上接收报文和发送报文到CAN总线上
 *
**********************************************************************************************************************/

#include "CanBus_Task.h"
#include "Gimbal_Task.h"


moto_measure_t  Chassis_Motor_Encoder[4];  //底盘电机参数结构体<can2>
moto_measure_t  Gimbal_Motor_Encoder[6];	 //云台电机参数结构体<can1>

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;


/**********************************************************************************************************************
  * @Func	 can_filter_init
  * @Brief   CAN1和CAN2滤波器的配置和开启CAN
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
	/*can1(0-13)和can2(14-27)分别得到一半的filter*/
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
			//底盘电机ID已被包含
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
							Gimbal_Motor_Encoder[i].msg_cnt++ <= 50 ? GetMotorOffset(&Gimbal_Motor_Encoder[i], rx_data) : GetMotorMeasure(&Gimbal_Motor_Encoder[i], rx_data);  /*读取底盘电机参数信息*/
						}
						
						if(_hcan->Instance == CAN2){
							i = rx_header.StdId - CAN_3508Moto1_ID;	
							can2_count++;
							if( i < 4 )  //底盘201·202·203·204
								Chassis_Motor_Encoder[i].msg_cnt++ <= 50 ? GetMotorOffset(&Chassis_Motor_Encoder[i], rx_data) : GetMotorMeasure(&Chassis_Motor_Encoder[i], rx_data);  /*读取底盘电机参数信息*/
							else if( i < 6 )   //云台205·206
								Gimbal_Motor_Encoder[i].msg_cnt++ <= 50 ? GetMotorOffset(&Gimbal_Motor_Encoder[i], rx_data) : GetMotorMeasure(&Gimbal_Motor_Encoder[i], rx_data);  /*读取底盘电机参数信息*/

						}
						break;
					}
		}
		
	if(can1_count == 500)  /*用于指示CAN1通信是否正常*/
	{
		can1_count = 0;
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);  
	}
	if(can2_count == 500)  /*用于指示CAN2通信是否正常*/
	{
		can2_count = 0;
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);  
	}

}


/**********************************************************************************************************************
 * @brief    接收3508电机通过CAN发过来的信息，2006电机也适用，不过2006电机没有温度值返来
 * @param	 moto_measure_t *ptr：电机参数结构体
 *           uint8_t can_rx_data[]：CAN接收数据缓存区
 * @retval	 None
**********************************************************************************************************************/
void GetMotorMeasure(moto_measure_t *ptr, uint8_t can_rx_data[])
{

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]);
	ptr->speed_rpm  = (int16_t)(can_rx_data[2]<<8 | can_rx_data[3]);
	ptr->real_current = (can_rx_data[4]<<8 | can_rx_data[5])*5.f/16384.f;
	ptr->hall = can_rx_data[6];
    
	/*编码器过零点处理*/
	if(ptr->angle - ptr->last_angle > 4096)
		ptr->round_cnt --;
	else if (ptr->angle - ptr->last_angle < -4096)
		ptr->round_cnt ++;
    
    /*得到转过的总角度，这个角度是相对于上电时的角度*/
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle; 
    
}

/*this function should be called after system+can init */
void GetMotorOffset(moto_measure_t *ptr, uint8_t can_rx_data[])        
{
	ptr->angle = (uint16_t)(can_rx_data[0]<<8 | can_rx_data[1]) ;
	ptr->offset_angle = ptr->angle;
}

/**********************************************************************************************************************
 * @Brief   获取yaw轴电机相对于车前面中点的编码器的相对值，转化为角度对应为-180度 ~ 180度，0度对应的是底盘前面的正中央
 * @Param   None
 * @Retval  云台yaw轴相对于底盘前面的电机机械角度值，范围为-4096 ~ 4096，左边为正，右边为负
 * @Other   因为要拿这个电机的相对机械角度值作PID计算，则这个电机的相对机械角度值的正负要根据电机的转向来设定
**********************************************************************************************************************/
int16_t GetEncoderRelativeAngle(void)
{
    
    int16_t relative_angle = 0, relative_angle_out = 0;
    int8_t direction_flag = 0;
    
    relative_angle = Gimbal_Motor_Encoder[0].angle - CHASSIS_YAW_MID_FRONT; /*直接作差*/
	
    if((CHASSIS_YAW_MID_FRONT < Gimbal_Motor_Encoder[0].angle) &&
			 (Gimbal_Motor_Encoder[0].angle < CHASSIS_YAW_MID_BACK)		) /*方向判断*/
    {
        direction_flag = -1;
    }
    else
    {
        direction_flag = 1;
    }
    
    if(relative_angle < 0) /*取正数*/
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

/*得到上电时的编码器相对于底盘前中心的机械角度值*/
int16_t GetEncoderOffsetRelativeAngle(void)
{
    
    int16_t relative_angle = 0, relative_angle_out = 0;
    int8_t direction_flag = 0;
    
    relative_angle = Gimbal_Motor_Encoder[YAW_CAN2_205].offset_angle - CHASSIS_YAW_MID_FRONT; /*直接作差*/
    if((CHASSIS_YAW_MID_FRONT < Gimbal_Motor_Encoder[YAW_CAN2_205].offset_angle)&&(Gimbal_Motor_Encoder[YAW_CAN2_205].offset_angle < CHASSIS_YAW_MID_BACK)) /*方向判断*/
    {
        direction_flag = -1;
    }
    else
    {
        direction_flag = 1;
    }
    
    if(relative_angle < 0) /*取正数*/
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
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  设置四个电机的电流值
 * @retval None
 * @others 标识符0x200对应前四个ID的电调，标识符0x1FF对应后四个ID的电调，一路CAN最多可以接八个电机，
 *         此函数是对应前4个ID的电调
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
 *         s16 iq1, s16 iq2, s16 iq3, s16 iq4  设置四个电机的电流值
 * @retval None
 * @others 标识符0x200对应前四个ID的电调，标识符0x1FF对应后四个ID的电调，一路CAN最多可以接八个电机，
 *         此函数是对应后4个ID的电调,即0x205开始的电机ID
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
