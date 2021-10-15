#ifndef _WIFICONFIG_H
#define _WIFICONFIG_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define _WIFI_USART huart1 /* 收发AT使用的串口 */

#define _WIFI_RX_SIZE 512           /* 接收数据缓存大小 */
#define _WIFI_RX_FOR_DATA_SIZE 1024 /* +IPD数据缓存大小 */
#define _WIFI_TX_SIZE 256           /* 发送数缓存大小 */
#define _WIFI_TASK_SIZE 512         /* wifi任务堆栈 */

#define _WIFI_WAIT_TIME_LOW 1000 /* 接收timeout */
#define _WIFI_WAIT_TIME_MED 10000
#define _WIFI_WAIT_TIME_HIGH 25000
#define _WIFI_WAIT_TIME_VERYHIGH 60000

#define USER_DEBUG 1
#define SEND_DEBUG 1
#define RECV_DEBUG 1

#endif
