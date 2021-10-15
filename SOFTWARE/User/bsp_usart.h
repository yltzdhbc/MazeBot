#ifndef BSP_USART_H
#define BSP_USART_H
//#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_uart.h"
// #include <stdio.h>

#include "usart.h"
//#include "dma.h"
#include "main.h"

#define UART_RX_DMA_SIZE (1024)

#define USART1_BUFLEN 28
#define USART1_MAX_LEN USART1_BUFLEN * 2

#define USART2_BUFLEN 28
#define USART2_MAX_LEN USART2_BUFLEN * 2

#define USART3_BUFLEN 28
#define USART3_MAX_LEN USART3_BUFLEN * 2

#define USART6_BUFLEN 28
#define USART6_MAX_LEN USART6_BUFLEN * 2

#define UART7_BUFLEN 28
#define UART7_MAX_LEN UART7_BUFLEN * 2

#define UART8_BUFLEN 28
#define UART8_MAX_LEN UART8_BUFLEN * 2

extern uint8_t usart1_buf[USART1_BUFLEN];
extern uint8_t usart2_buf[USART2_BUFLEN];
extern uint8_t usart3_buf[USART3_BUFLEN];
extern uint8_t usart6_buf[USART6_BUFLEN];
extern uint8_t uart7_buf[UART7_BUFLEN];
extern uint8_t uart8_buf[UART8_BUFLEN];

//ACTION定位模块数据联合体
typedef union _imu_data {
    uint8_t data[24];
    float ActVal[6];
} imudata_t;
extern imudata_t imudata;

#define USART1_TXBUFF_SIZE 256 //定义串口1 发送缓冲区大小 256字节
#define USART1_RXBUFF_SIZE 256 //定义串口2 接收缓冲区大小 256字节
extern char Usart1_RxBuff[USART1_RXBUFF_SIZE];

#define USART2_TXBUFF_SIZE 256 //定义串口1 发送缓冲区大小 256字节
#define USART2_RXBUFF_SIZE 256 //定义串口2 接收缓冲区大小 256字节
extern char Usart2_RxBuff[USART2_RXBUFF_SIZE];

void uart_receive_handler(UART_HandleTypeDef *huart);
void uart_receive_init(UART_HandleTypeDef *huart);
#define ABS(x) ((x > 0) ? (x) : (-x))

#endif
