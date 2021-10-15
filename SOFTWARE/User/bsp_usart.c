///**
//  ****************************(C) COPYRIGHT 2020 HCRT****************************
//  * @file       bsp_usart.c/h
//  * @brief      �����ж����ú��жϴ�����
//  * @note
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.0     2020.2.1    		 YLT             V1.0 done
//	*  V1.0.1     2020.3.16    		 YLT               update
//  @verbatim
//  ==============================================================================
//	* 2020.3.16(update):������ϸ��ע�� �Ż������߼� ���ж���ں���ȫ����ӵ�
//	* stm32f4xx_it.c ���� �����ж�+DMA �������ݵ�����˵��
//	*
//	* �����жϽ��� :
//	*						�����ж�													�����ж�
//	*	������ 	USARTx_IRQHandler								USARTx_IRQHandler
//	*	�ص����� 	HAL_UART_RxCpltCallback 				HAL��û���ṩ
//	*	USART ״̬�Ĵ����е�λ UART_FLAG_RXNE 		UART_FLAG_IDLE
//	*	�������� ���һ֡���ݵĽ���֮�󴥷�һ��		���ڽ�����һ֡���ݺ��ֹ���һ��
//  *																						�ֽڵ�ʱ��û�н��յ��κ�����
//	*	���ڿ����жϼ�ÿ�����ڽ�����һ֡���ݺ��ֹ���һ���ֽڵ�ʱ��û�н��յ��κ�������
//	* ����һ���жϣ��жϴ�����ͬ��Ϊ USARTx_IRQHandler������ͨ�� USART ״̬�Ĵ���
//	* �е� UART_FLAG_IDLE �ж��Ƿ����˿����жϡ�
//	*
//	*	����DMA UART ������˵ ͨ��ʣ�����ݸ��� = �ܹ���Ҫ���յĸ��� - Ŀǰ�ѷ��ͳ������ݵĸ���
//	*	����DMA UART ������˵ ͨ��ʣ�����ݸ��� = �ܹ���Ҫ���յĸ��� - Ŀǰ�ѽ��յ������ݵĸ���
//	*
//	* �����жϵ����ݳ����ж� Ϊʲô USARTX_BUFLEN = USARTX_MAX_LEN - dma_stream->NDTR 
//	*	��DMA����ʱ�����������յ������ݵĸ���Ϊ USARTX_MAX_LEN ��NDTR�Ĵ���=USARTX_MAX_LEN
//  * ��ÿ��DMA������һ������ NDTR�Ĵ�����ֵ�ͻ��һ
//	*	��Ϊ���Ƕ�����������������Ϊ���ݳ��ȵ����� USARTX_MAX_LEN = USARTX_BUFLEN * 2
//	* ��˵�dma_stream->NDTR ��ֵ��USARTX_MAX_LEN ��Ϊ USARTX_BUFLENʱ ���������ж�����
//	*	ΪʲôҪ��������������Ϊ���ݳ��ȵ�������
//	* 1.������������ ��֤�������ܹ���ȫ�����յ� 2.Ϊ�˺��ж� д�ж�������ʱ��Ƚϼ��
//	*
//	*	2020.2.14(update):
//	* д�������д��ڵĿ����жϣ�����ʹ��
//	* ��Ҫ��stm32f4xx_it.c �������ں��� uart_receive_handler
//  ==============================================================================
//  @endverbatim
//  ****************************(C) COPYRIGHT 2020 HCRT****************************
//  */
//#include <stdio.h>
//#include <string.h>
//#include "bsp_usart.h"
//#include "bsp_led.h"
//#include "ctrl.h"

//uint8_t usart2_buf[USART2_BUFLEN];

//extern DriverType Driver[8];
//extern MotorType Motor[8];

///******** ���ڽ������鶨�� ********/

//char Usart2_RxBuff[USART2_RXBUFF_SIZE];
//__align(8) char Usart2_TxBuff[USART2_TXBUFF_SIZE];

///* ���ڿ����жϴ����� */
//void USART2_CMD_Hander(char *buffer, uint8_t bufferLen)
//{
//  static volatile int reValue0 = 0, reValue1 = 0;
//  static volatile _Bool reFlag0 = 1, reFlag1 = 1;
//  static volatile uint8_t commaNum = 0;

//  /* ��ȡָ��Ľ����� �ж�һ��ָ���Ƿ���� */
//  if ((buffer[bufferLen - 1] == '\n') || (buffer[bufferLen - 1] == '\r') || (buffer[bufferLen - 1] == ';'))
//  {
//    /* �ҳ� "," ���ڵ�λ�� */
//    for (int i = 0; i < (bufferLen); i++)
//    {
//      if (strncmp(&buffer[i], ",", 1) == 0)
//      {
//        commaNum = i;
//        break;
//      }
//    }

//    if (commaNum == bufferLen)
//    {
//      printf((char *)"MISSING ��,�� OR SEC_PARAM;\r\n");
//    }

//    /* �������ݴ����ж� */
//    if (bufferLen == 1) //����ĳ���Ϊ1�����ݲ���
//    {
//      bufferLen = 0;
//    }
//    else if (buffer[2] == '=') //��ֵ�������
//    {
//      /* ���е�һ�����ݵ������ж�ת�� */
//      if (buffer[3] == '-') //����
//      {
//        for (int i = 4; i < (commaNum); i++)
//          reValue0 = reValue0 * 10 + buffer[i] - '0';
//        reValue0 = -1 * reValue0;
//      }
//      else //�Ǹ���
//      {
//        for (int i = 3; i < (commaNum); i++)
//          reValue0 = reValue0 * 10 + buffer[i] - '0';
//      }
//      /* ���еڶ������ݵ������ж�ת�� */
//      if (buffer[commaNum + 1] == '-') //����
//      {
//        for (int i = (commaNum + 2); i < (bufferLen - 1); i++)
//          reValue1 = reValue1 * 10 + buffer[i] - '0';
//        reValue1 = -1 * reValue1;
//      }
//      else //�Ǹ���
//      {
//        for (int i = (commaNum + 1); i < (bufferLen - 1); i++)
//          reValue1 = reValue1 * 10 + buffer[i] - '0';
//      }

//      //�Աȵ�0λ�͵�1λ ѡ��ͬ��ָ��
//      if (strncmp(buffer, "MO", 2) == 0) //���ʹ������
//      {
//        (reValue0 == 1) ? (MotorOn(0)) : (MotorOff(0));
//        (reValue1 == 1) ? (MotorOn(1)) : (MotorOff(1));
//        printf((char *)"MO=%d,%d;\r\n", (int)(reValue0), (int)(reValue1));
//      }
//      if (strncmp(buffer, "JV", 2) == 0) //JV �ٶȻ� �ٶ�
//      {
//        Driver[0].velCtrl.desiredVel[CMD] = (float)reValue0 * 0.001f;
//        Driver[1].velCtrl.desiredVel[CMD] = (float)reValue1 * 0.001f;
//        printf((char *)"JV=%d,%d;\r\n", (int)(Driver[0].velCtrl.desiredVel[CMD] * 1000.0f),
//               (int)(Driver[0].velCtrl.desiredVel[CMD] * 1000.0f));
//      }
//      else if (strncmp(buffer, "AC", 2) == 0) //AC �ٶȻ� ���ٶ�
//      {
//        Driver[0].velCtrl.acc = (float)(reValue0) / 1000000.0f;
//        Driver[1].velCtrl.acc = (float)(reValue1) / 1000000.0f;
//        printf((char *)"AC=%d,%d;\r\n", (int)(Driver[0].velCtrl.acc * 1000000.0f),
//               (int)(Driver[1].velCtrl.acc * 1000000.0f));
//      }
//      else if (strncmp(buffer, "DC", 2) == 0) //DC �ٶȻ� ���ٶ�
//      {
//        Driver[0].velCtrl.dec = (float)(reValue0) / 1000000.0f;
//        Driver[1].velCtrl.dec = (float)(reValue1) / 1000000.0f;
//        printf((char *)"DC=%d,%d;\r\n", (int)(Driver[0].velCtrl.dec * 1000000.0f),
//               (int)(Driver[1].velCtrl.dec * 1000000.0f));
//      }
//      else if (strncmp(buffer, "SP", 2) == 0) //SP λ�û� �ٶ�
//      {
//        Driver[0].posCtrl.posVel = (float)reValue0 / 1000.0f;
//        Driver[1].posCtrl.posVel = (float)reValue1 / 1000.0f;
//        printf((char *)"SP=%d,%d;\r\n", (int)(Driver[0].posCtrl.posVel * 1000.0f),
//               (int)(Driver[0].posCtrl.posVel * 1000.0f));
//      }
//      else if (strncmp(buffer, "PA", 2) == 0) //PA λ�û� ����λ��
//      {
//        Driver[0].posCtrl.desiredPos = (float)reValue0;
//        Driver[1].posCtrl.desiredPos = (float)reValue1;
//        printf((char *)"PA=%d,%d;\r\n", (int)(Driver[0].posCtrl.desiredPos),
//               (int)(Driver[0].posCtrl.desiredPos));
//      }
//      else if (strncmp(buffer, "PR", 2) == 0) //PR λ�û� ���λ��
//      {
//        Driver[0].posCtrl.desiredPos = (float)reValue0 + Driver[0].posCtrl.actualPos;
//        Driver[1].posCtrl.desiredPos = (float)reValue1 + Driver[1].posCtrl.actualPos;
//        printf((char *)"PR=%d,%d;\r\n", (int)(reValue0), (int)(reValue1));
//      }
//      else
//      {
//        printf((char *)"INVALID WRITE CMD!!!\r\n");
//      }
//    }
//    else //��� buffer[2] != '=' ��Ϊ��ѯ���
//    {
//      if (strncmp(buffer, "VX", 2) == 0) //VX ��ѯ��ǰ�ٶ�
//      {
//        printf((char *)"VX%d,%d;\r\n", (int)(Driver[0].velCtrl.speed * 1000.0f),
//               (int)(Driver[1].velCtrl.speed * 1000.0f));
//      }
//      else if (strncmp(buffer, "PX", 2) == 0) //PX ��ѯ��ǰλ��
//      {
//        printf((char *)"PX%d,%d;\r\n", (int)(Driver[0].posCtrl.actualPos),
//               (int)(Driver[1].posCtrl.actualPos));
//      }
//      else if (strncmp(buffer, "IQ", 2) == 0) //IQ ��ѯ��ǰ����
//      {
//        printf((char *)"IQ invalid\r\n");
//      }
//      else
//      {
//        printf((char *)"INVALID READ CMD!!!\r\n"); //ָ���ʽ����
//      }
//    }
//    //��ճ�ʼ����
//    bufferLen = 0;
//    reValue1 = 0;
//    reFlag1 = 1;
//  }
//  else // ָ���޽�����
//  {
//    bufferLen = 0;
//    printf("NOT START WITH 'A'\r\n");
//  }
//}

///**
//  * @brief      ����DMA�������ݵĳ���
//  * @param[in]  hdma: DMA���ָ��
//	* @param[in]  DataLength: ���ݳ���
//  * @retval     none
//  */
//void dma_set_counter(DMA_HandleTypeDef *hdma, uint32_t DataLength)
//{
//  hdma->Instance->NDTR = DataLength;
//}

///**
//  * @brief	�ڽ��յ�һ֡����֮�����һ֡����ʱ��֮��������
//	*					���������˻ص�����,�˺�������������жϱ�־λ
//  * @param	huart: UART���ָ��
//  * @retval
//  */
//static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
//{
//  uint8_t DataLength = 0; //�������ݵĳ���

//  if (huart == &huart3)
//  {
//    /* ���㱾�ν��յ������ݵĳ��� */
//    DataLength = USART1_MAX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

//    /* ��������жϴ����� */
//    USART2_CMD_Hander(Usart2_RxBuff, DataLength);

//#ifdef UART_DEBUG
//    printf("data len is:%d\r\n", dataLen);
//    printf("data rx is:%s\r\n", Usart2_RxBuff);
//#endif // UART_DEBUG

//    /* ����DMA�������ݵĳ��� */
//    dma_set_counter(huart->hdmarx, USART2_MAX_LEN);
//  }
//}

///**
//  * @brief	�����ڷ����жϵ�ʱ����˺���
//  * @param	huart: UART���ָ��
//  * @retval	��stm32f4xx_it.c�����
//  */
//void uart_receive_handler(UART_HandleTypeDef *huart)
//{
//  /* __HAL_UART_GET_FLAG ���б�־λ�Ƿ񴥷� */
//  /* __HAL_UART_GET_IT_SOURCEG �����ж��Ƿ񴥷� */
//  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
//      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
//  {
//    /* ��������жϱ�־λ */
//    __HAL_UART_CLEAR_IDLEFLAG(huart);

//    /* �ص�DMA */
//    __HAL_DMA_DISABLE(huart->hdmarx);

//    /* ��������жϴ����� */
//    uart_rx_idle_callback(huart);

//    /* ����DMA���� */
//    __HAL_DMA_ENABLE(huart->hdmarx);
//  }
//}

///**
//  * @brief      ����ʹ��DMA����(�������жϽ���)
//  * @param[in]  huart: UART���ָ��
//  * @param[in]  pData: receive buff
//  * @param[in]  Size:  buff size
//  * @retval     set success or fail
//  */
//static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
//{
//  uint32_t tmp = 0;
//  tmp = huart->RxState;

//  /* �жϴ����Ƿ��Ѿ���ʼ����� */
//  if (tmp == HAL_UART_STATE_READY)
//  {
//    /* ����û�����������Ƿ���ȷ */
//    if ((pData == NULL) || (Size == 0))
//      return HAL_ERROR;

//    huart->pRxBuffPtr = pData;
//    huart->RxXferSize = Size;
//    huart->ErrorCode = HAL_UART_ERROR_NONE;

//    /* ʹ��DMAͨ�� */
//    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

//    /* ����DMA���� ��UART CR3 �Ĵ����е� DMARλ �ø� */
//    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

//    return HAL_OK;
//  }
//  else
//    return HAL_BUSY;
//}

///**
//  * @brief	�����жϳ�ʼ������
//  * @param	huart:UART���ָ��
//  * @retval	none
//  */
//void uart_receive_init(UART_HandleTypeDef *huart)
//{
//  if (huart == &huart3)
//  {
//    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
//    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
//    uart_receive_dma_no_it(&huart3, usart2_buf, USART2_MAX_LEN);
//  }
//}

//int fputc(int ch, FILE *stream)
//{
//  uint8_t temp = ch;
//  HAL_UART_Transmit(&huart3, &temp, 1, 2);
//  return temp;
//}

//int fgetc(FILE *stream)
//{
//  uint8_t ch;
//  HAL_UART_Receive(&huart3, &ch, 1, 0xffff);
//  return ch;
//}
