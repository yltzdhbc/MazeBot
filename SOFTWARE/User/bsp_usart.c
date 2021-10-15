///**
//  ****************************(C) COPYRIGHT 2020 HCRT****************************
//  * @file       bsp_usart.c/h
//  * @brief      空闲中断配置和中断处理函数
//  * @note
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.0     2020.2.1    		 YLT             V1.0 done
//	*  V1.0.1     2020.3.16    		 YLT               update
//  @verbatim
//  ==============================================================================
//	* 2020.3.16(update):增加详细的注释 优化程序逻辑 将中断入口函数全部添加到
//	* stm32f4xx_it.c 增加 空闲中断+DMA 接收数据的流程说明
//	*
//	* 空闲中断介绍 :
//	*						接收中断													空闲中断
//	*	处理函数 	USARTx_IRQHandler								USARTx_IRQHandler
//	*	回调函数 	HAL_UART_RxCpltCallback 				HAL库没有提供
//	*	USART 状态寄存器中的位 UART_FLAG_RXNE 		UART_FLAG_IDLE
//	*	触发条件 完成一帧数据的接收之后触发一次		串口接收完一帧数据后又过了一个
//  *																						字节的时间没有接收到任何数据
//	*	串口空闲中断即每当串口接收完一帧数据后又过了一个字节的时间没有接收到任何数据则
//	* 触发一次中断，中断处理函数同样为 USARTx_IRQHandler，可以通过 USART 状态寄存器
//	* 中的 UART_FLAG_IDLE 判断是否发生了空闲中断。
//	*
//	*	对于DMA UART 发送来说 通道剩余数据个数 = 总共需要接收的个数 - 目前已发送出的数据的个数
//	*	对于DMA UART 接收来说 通道剩余数据个数 = 总共需要接收的个数 - 目前已接收到的数据的个数
//	*
//	* 空闲中断的数据长度判断 为什么 USARTX_BUFLEN = USARTX_MAX_LEN - dma_stream->NDTR 
//	*	在DMA开启时填充的期望接收到的数据的个数为 USARTX_MAX_LEN 即NDTR寄存器=USARTX_MAX_LEN
//  * 而每当DMA搬运完一个数据 NDTR寄存器的值就会减一
//	*	因为我们定义了最大接收数设置为数据长度的两倍 USARTX_MAX_LEN = USARTX_BUFLEN * 2
//	* 因此当dma_stream->NDTR 的值由USARTX_MAX_LEN 变为 USARTX_BUFLEN时 就满足了判断条件
//	*	为什么要将最大接收数设置为数据长度的两倍？
//	* 1.留出接收余量 保证数据能能够被全部接收到 2.为了好判断 写判断条件的时候比较简洁
//	*
//	*	2020.2.14(update):
//	* 写完了所有串口的空闲中断，按需使用
//	* 需要在stm32f4xx_it.c 中添加入口函数 uart_receive_handler
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

///******** 串口接收数组定义 ********/

//char Usart2_RxBuff[USART2_RXBUFF_SIZE];
//__align(8) char Usart2_TxBuff[USART2_TXBUFF_SIZE];

///* 串口空闲中断处理函数 */
//void USART2_CMD_Hander(char *buffer, uint8_t bufferLen)
//{
//  static volatile int reValue0 = 0, reValue1 = 0;
//  static volatile _Bool reFlag0 = 1, reFlag1 = 1;
//  static volatile uint8_t commaNum = 0;

//  /* 读取指令的结束符 判断一条指令是否结束 */
//  if ((buffer[bufferLen - 1] == '\n') || (buffer[bufferLen - 1] == '\r') || (buffer[bufferLen - 1] == ';'))
//  {
//    /* 找出 "," 所在的位置 */
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
//      printf((char *)"MISSING “,” OR SEC_PARAM;\r\n");
//    }

//    /* 进入数据处理判断 */
//    if (bufferLen == 1) //数组的长度为1则数据不对
//    {
//      bufferLen = 0;
//    }
//    else if (buffer[2] == '=') //赋值命令语句
//    {
//      /* 进行第一个数据的正负判断转换 */
//      if (buffer[3] == '-') //负数
//      {
//        for (int i = 4; i < (commaNum); i++)
//          reValue0 = reValue0 * 10 + buffer[i] - '0';
//        reValue0 = -1 * reValue0;
//      }
//      else //非负数
//      {
//        for (int i = 3; i < (commaNum); i++)
//          reValue0 = reValue0 * 10 + buffer[i] - '0';
//      }
//      /* 进行第二个数据的正负判断转换 */
//      if (buffer[commaNum + 1] == '-') //负数
//      {
//        for (int i = (commaNum + 2); i < (bufferLen - 1); i++)
//          reValue1 = reValue1 * 10 + buffer[i] - '0';
//        reValue1 = -1 * reValue1;
//      }
//      else //非负数
//      {
//        for (int i = (commaNum + 1); i < (bufferLen - 1); i++)
//          reValue1 = reValue1 * 10 + buffer[i] - '0';
//      }

//      //对比第0位和第1位 选择不同的指令
//      if (strncmp(buffer, "MO", 2) == 0) //电机使能命令
//      {
//        (reValue0 == 1) ? (MotorOn(0)) : (MotorOff(0));
//        (reValue1 == 1) ? (MotorOn(1)) : (MotorOff(1));
//        printf((char *)"MO=%d,%d;\r\n", (int)(reValue0), (int)(reValue1));
//      }
//      if (strncmp(buffer, "JV", 2) == 0) //JV 速度环 速度
//      {
//        Driver[0].velCtrl.desiredVel[CMD] = (float)reValue0 * 0.001f;
//        Driver[1].velCtrl.desiredVel[CMD] = (float)reValue1 * 0.001f;
//        printf((char *)"JV=%d,%d;\r\n", (int)(Driver[0].velCtrl.desiredVel[CMD] * 1000.0f),
//               (int)(Driver[0].velCtrl.desiredVel[CMD] * 1000.0f));
//      }
//      else if (strncmp(buffer, "AC", 2) == 0) //AC 速度环 加速度
//      {
//        Driver[0].velCtrl.acc = (float)(reValue0) / 1000000.0f;
//        Driver[1].velCtrl.acc = (float)(reValue1) / 1000000.0f;
//        printf((char *)"AC=%d,%d;\r\n", (int)(Driver[0].velCtrl.acc * 1000000.0f),
//               (int)(Driver[1].velCtrl.acc * 1000000.0f));
//      }
//      else if (strncmp(buffer, "DC", 2) == 0) //DC 速度环 减速度
//      {
//        Driver[0].velCtrl.dec = (float)(reValue0) / 1000000.0f;
//        Driver[1].velCtrl.dec = (float)(reValue1) / 1000000.0f;
//        printf((char *)"DC=%d,%d;\r\n", (int)(Driver[0].velCtrl.dec * 1000000.0f),
//               (int)(Driver[1].velCtrl.dec * 1000000.0f));
//      }
//      else if (strncmp(buffer, "SP", 2) == 0) //SP 位置环 速度
//      {
//        Driver[0].posCtrl.posVel = (float)reValue0 / 1000.0f;
//        Driver[1].posCtrl.posVel = (float)reValue1 / 1000.0f;
//        printf((char *)"SP=%d,%d;\r\n", (int)(Driver[0].posCtrl.posVel * 1000.0f),
//               (int)(Driver[0].posCtrl.posVel * 1000.0f));
//      }
//      else if (strncmp(buffer, "PA", 2) == 0) //PA 位置环 绝对位置
//      {
//        Driver[0].posCtrl.desiredPos = (float)reValue0;
//        Driver[1].posCtrl.desiredPos = (float)reValue1;
//        printf((char *)"PA=%d,%d;\r\n", (int)(Driver[0].posCtrl.desiredPos),
//               (int)(Driver[0].posCtrl.desiredPos));
//      }
//      else if (strncmp(buffer, "PR", 2) == 0) //PR 位置环 相对位置
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
//    else //如果 buffer[2] != '=' 则为查询语句
//    {
//      if (strncmp(buffer, "VX", 2) == 0) //VX 查询当前速度
//      {
//        printf((char *)"VX%d,%d;\r\n", (int)(Driver[0].velCtrl.speed * 1000.0f),
//               (int)(Driver[1].velCtrl.speed * 1000.0f));
//      }
//      else if (strncmp(buffer, "PX", 2) == 0) //PX 查询当前位置
//      {
//        printf((char *)"PX%d,%d;\r\n", (int)(Driver[0].posCtrl.actualPos),
//               (int)(Driver[1].posCtrl.actualPos));
//      }
//      else if (strncmp(buffer, "IQ", 2) == 0) //IQ 查询当前电流
//      {
//        printf((char *)"IQ invalid\r\n");
//      }
//      else
//      {
//        printf((char *)"INVALID READ CMD!!!\r\n"); //指令格式错误
//      }
//    }
//    //清空初始变量
//    bufferLen = 0;
//    reValue1 = 0;
//    reFlag1 = 1;
//  }
//  else // 指令无结束符
//  {
//    bufferLen = 0;
//    printf("NOT START WITH 'A'\r\n");
//  }
//}

///**
//  * @brief      设置DMA接收数据的长度
//  * @param[in]  hdma: DMA入口指针
//	* @param[in]  DataLength: 数据长度
//  * @retval     none
//  */
//void dma_set_counter(DMA_HandleTypeDef *hdma, uint32_t DataLength)
//{
//  hdma->Instance->NDTR = DataLength;
//}

///**
//  * @brief	在接收到一帧数据之后空闲一帧数据时间之后无数据
//	*					再来则进入此回调函数,此函数会清除空闲中断标志位
//  * @param	huart: UART句柄指针
//  * @retval
//  */
//static void uart_rx_idle_callback(UART_HandleTypeDef *huart)
//{
//  uint8_t DataLength = 0; //接收数据的长度

//  if (huart == &huart3)
//  {
//    /* 计算本次接收到的数据的长度 */
//    DataLength = USART1_MAX_LEN - __HAL_DMA_GET_COUNTER(huart->hdmarx);

//    /* 进入空闲中断处理函数 */
//    USART2_CMD_Hander(Usart2_RxBuff, DataLength);

//#ifdef UART_DEBUG
//    printf("data len is:%d\r\n", dataLen);
//    printf("data rx is:%s\r\n", Usart2_RxBuff);
//#endif // UART_DEBUG

//    /* 设置DMA接收数据的长度 */
//    dma_set_counter(huart->hdmarx, USART2_MAX_LEN);
//  }
//}

///**
//  * @brief	当串口发生中断的时候进此函数
//  * @param	huart: UART句柄指针
//  * @retval	在stm32f4xx_it.c中添加
//  */
//void uart_receive_handler(UART_HandleTypeDef *huart)
//{
//  /* __HAL_UART_GET_FLAG 空闲标志位是否触发 */
//  /* __HAL_UART_GET_IT_SOURCEG 空闲中断是否触发 */
//  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
//      __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
//  {
//    /* 清除空闲中断标志位 */
//    __HAL_UART_CLEAR_IDLEFLAG(huart);

//    /* 关掉DMA */
//    __HAL_DMA_DISABLE(huart->hdmarx);

//    /* 进入空闲中断处理函数 */
//    uart_rx_idle_callback(huart);

//    /* 重启DMA传输 */
//    __HAL_DMA_ENABLE(huart->hdmarx);
//  }
//}

///**
//  * @brief      配置使能DMA接收(而不是中断接收)
//  * @param[in]  huart: UART句柄指针
//  * @param[in]  pData: receive buff
//  * @param[in]  Size:  buff size
//  * @retval     set success or fail
//  */
//static int uart_receive_dma_no_it(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
//{
//  uint32_t tmp = 0;
//  tmp = huart->RxState;

//  /* 判断串口是否已经初始化完成 */
//  if (tmp == HAL_UART_STATE_READY)
//  {
//    /* 检测用户输入的数据是否正确 */
//    if ((pData == NULL) || (Size == 0))
//      return HAL_ERROR;

//    huart->pRxBuffPtr = pData;
//    huart->RxXferSize = Size;
//    huart->ErrorCode = HAL_UART_ERROR_NONE;

//    /* 使能DMA通道 */
//    HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

//    /* 开启DMA传输 将UART CR3 寄存器中的 DMAR位 置高 */
//    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

//    return HAL_OK;
//  }
//  else
//    return HAL_BUSY;
//}

///**
//  * @brief	空闲中断初始化函数
//  * @param	huart:UART句柄指针
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
