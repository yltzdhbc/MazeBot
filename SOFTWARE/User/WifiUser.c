/* Includes ------------------------------------------------------------------*/
#include "Wifi.h"

#include "usart.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  初始化wifi
  * @param  无 
  * @retval 无
  */
void Wifi_UserInit(void)
{
    /* 设置wifi工作模式 */
    Wifi_SetMode(WifiMode_Station);
		printf("*** wifi initial start *** \r\n");
    /* station模式连接ap */
    while (Wifi_Station_ConnectToAp("Hi_2301", "773673787", NULL) == false)
        ;
    printf("*** wifi initial finished *** \r\n");
	 
    LOG_USER_DEBUG("wifi initial finished\r\n");
}

/**
  * @brief  用户wifi线程
  * @param  无
  * @retval 无
  */
void Wifi_UserProcess(void)
{
    static uint8_t last = 0;

    if (strstr(Wifi.MyIP, "0.0.0.0") != NULL)
    {
        if (last != 1)
        {
        }
        last = 1;
    }
    else
    {
        if (last != 2)
        {
            /* 初始化远端ip地址 */
            strcpy(Wifi.MyGateWay, "192.168.199.185");
            Wifi_TcpIp_StartTcpConnection(0, Wifi.MyGateWay, 6666, 10);
        }
        last = 2;

        char ch[] = "hello";

        Wifi_TcpIp_SendDataTcp(0, sizeof(ch), (uint8_t *)ch);
    }
}

/**
  * @brief  获取UDP数据
  * @param  linkid : ？
  * @param  DataLen : 数据长度
  * @param  Data : 数据地址
  * @retval 无
  */
void Wifi_UserGetUdpData(uint8_t LinkId, uint16_t DataLen, uint8_t *Data)
{
    Wifi_TcpIp_SendDataUdp(LinkId, 2, (uint8_t *)"OK");
}

/**
  * @brief  获取TCP数据
  * @param  linkid : ？
  * @param  DataLen : 数据长度
  * @param  Data : 数据地址
  * @retval 无
  */
void Wifi_UserGetTcpData(uint8_t LinkId, uint16_t DataLen, uint8_t *Data)
{
    Wifi_TcpIp_SendDataTcp(LinkId, 2, (uint8_t *)"OK");
    printf("%s\r\n", Data);
}
