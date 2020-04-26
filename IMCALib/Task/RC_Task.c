/**********************************************************************************************************************
 * @file   RC_Task.c
 * @brief  DR16遥控接收数据的解析和用串口2的DMA接收遥控器发过来的数据，使用时将本文件的头文件添加
 *         到stm32f4xx_it.c中，在USART2_IRQHandler()函数中添加HAL_UART_IDLE_IRQHandler(&huart2)
 *         串口中断空闲函数，并把USART2_IRQHandler()函数中原来的HAL_UART_IRQHandler(&huart2)注释掉
**********************************************************************************************************************/

#include "RC_Task.h"


RC_Type remote_control;
uint8_t UART_Buffer[100];


/**********************************************************************************************************************
  * @Func	 void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief   DR16接收机协议解码程序
  * @Param	 RC_Type* rc　  存储遥控器数据的结构体
  *          uint8_t* buff　用于解码的缓存　　
  * @Retval	 None  
 *********************************************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
//	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
	
	rc->ch5 = ((int16_t)buff[16] | ((int16_t)buff[17] << 8)) & 0x07FF;
  rc->ch5 -= 1024;
}


/**********************************************************************************************************************
  * @brief  配置串口没有DMA中断的接收,串口使用了空闲中断，所以这里没有必要再开启DMA中断，由于HAL库中
  *			所带的UART_Receive_DMA函数默认开启了DMA中断，这里要自己实现一个没有中断的DMA接收函数。	
  * @param  UART_HandleTypeDef *huart   UART handle Structure
  *         uint8_t *pData              接收缓存区
  *         uint16_t Size               数据大小
  * @retval HAL status
**********************************************************************************************************************/
HAL_StatusTypeDef UART_Receive_DMA_NoIT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
 uint32_t *tmp;
  
  /* Check that a Rx process is not already ongoing */
  if(huart->RxState == HAL_UART_STATE_READY) 
  {
    if((pData == NULL) || (Size == 0U))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pData;
    huart->RxXferSize = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    /* Set the DMA abort callback */
    huart->hdmarx->XferAbortCallback = NULL;

    /* Enable the DMA channel */
    tmp = (uint32_t*)&pData;
    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, *(uint32_t*)tmp, Size);

    /* Clear the Overrun flag just before enabling the DMA Rx request: can be mandatory for the second transfer */
    __HAL_UART_CLEAR_OREFLAG(huart);

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART Parity Error Interrupt */
    SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    /* Enable the DMA transfer for the receiver request by setting the DMAR bit 
    in the UART CR3 register */
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


/**********************************************************************************************************************
 * @brief  启动串口空闲中断,在初始化时调用
 * @param  UART_HandleTypeDef *huart   UART handle Structure
 *         uint8_t *pData              接收缓存区
 *         uint16_t Size               数据大小
 * @retval HAL status
**********************************************************************************************************************/
HAL_StatusTypeDef HAL_UART_Receive_IT_IDLE(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
	
		__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  /*开启串口空闲中断*/
	
		return UART_Receive_DMA_NoIT(huart,pData,Size);  /*启动没有DMA中断的DMA接收函数*/

}


/**********************************************************************************************************************
 * @brief   串口空闲中断，在USART2中断中调用
 * @param   UART_HandleTypeDef *huart    UART handle Structure
 *
 * @retval  None
**********************************************************************************************************************/
void HAL_UART_IDLE_IRQHandler(UART_HandleTypeDef *huart)
{
		
		uint32_t DMA_FLAGS,tmp;
	
			
	if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
			
		tmp = huart->Instance->SR;
		tmp = huart->Instance->DR;
		tmp++;  
		DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);
		
		//失能DMA
	    __HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx,DMA_FLAGS);
		
		huart->hdmarx->Instance->NDTR = huart->RxXferSize;
		__HAL_DMA_ENABLE(huart->hdmarx);
		
	}
	Callback_RC_Handle(&remote_control,huart->pRxBuffPtr);  /*遥控接收器接收数据解析*/
	
}

//遥控器死区限制，因为遥控器的拨杆在中位的时候，不一定是发送1024过来
int16_t RcDeadlineLimit(int16_t input, uint8_t dealine)
{
    int16_t output;
    if((input > dealine)||(input < -dealine))
    {
        output = input;
    }
    else
    {
        output = 0;
    }
    return output;

}

