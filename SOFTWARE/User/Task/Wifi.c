/* Includes ------------------------------------------------------------------*/
#include "Wifi.h"
#include "WifiConfig.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId WifiTaskHandle;
osSemaphoreId WifiSemHandle;

Wifi_t Wifi;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/***********************************************************************************************************
***********************************************************************************************************/
/**
  * @brief  发送数据原始函数
  * @param  data : 要发送的数据
  * @param  len : 数据长度
  * @retval true : 成功; false : 失败
  */
bool Wifi_SendRaw(uint8_t *data, uint16_t len)
{
    if (len <= _WIFI_TX_SIZE)
    {
        memcpy(Wifi.TxBuffer, data, len);

        LOG_SEND_DEBUG("%s", Wifi.TxBuffer);

        if (HAL_UART_Transmit(&_WIFI_USART, data, len, 100) == HAL_OK)
            return true;
        else
            return false;
    }
    else
        return false;
}
/**
  * @brief  发送字符串
  * @param  无
  * @retval 无
  */
bool Wifi_SendString(char *data)
{
    return Wifi_SendRaw((uint8_t *)data, strlen(data));
}

/**
  * @brief  等待接收数据，并检测是否与参数相匹配
  * @param  data : 要发送的数据
  * @param  DelayMs : 等待的时间
  * @retval true : 成功; false : 失败
  */
bool Wifi_SendStringAndWait(char *data, uint16_t DelayMs)
{
    if (Wifi_SendRaw((uint8_t *)data, strlen(data)) == false)
        return false;

    osDelay(DelayMs);

    return true;
}

/**
  * @brief  等待接收数据，并检测是否与参数相匹配
  * @param  TimeOut_ms : 超时时间
  * @param  result : 匹配参数的数量
  * @param  CountOfParameter : 数据地址
  * @retval 是否有接收到匹配参数的数据
  */
bool Wifi_WaitForString(uint32_t TimeOut_ms, uint8_t *result, uint8_t CountOfParameter, ...)
{

    if (result == NULL)
        return false;

    if (CountOfParameter == 0)
        return false;

    *result = 0;

    LOG_RECV_DEBUG("%s", Wifi.RxBuffer);

    /* 提取可变参数 */
    va_list tag;
    va_start(tag, CountOfParameter);
    char *arg[CountOfParameter];
    for (uint8_t i = 0; i < CountOfParameter; i++)
        arg[i] = va_arg(tag, char *);
    va_end(tag);

    /* 检测是否接收到匹配参数 */
    for (uint32_t t = 0; t < TimeOut_ms; t += 50)
    {
        osDelay(50);
        for (uint8_t mx = 0; mx < CountOfParameter; mx++)
        {
            if (strstr((char *)Wifi.RxBuffer, arg[mx]) != NULL)
            {
                *result = mx + 1;
                return true;
            }
        }
    }

    /* 超时 */
    return false;
}

/**
  * @brief  提取指定位置的字符串
  * @param  result : 分割出的结果
  * @param  WantWhichOne : 位置（分割后的第几个字符串）
  * @param  SplitterChars : 分隔符
  * @retval true : 成功; false : 失败
  */
bool Wifi_ReturnString(char *result, uint8_t WantWhichOne, char *SplitterChars)
{
    if (result == NULL)
        return false;

    if (WantWhichOne == 0)
        return false;

    char *str = (char *)Wifi.RxBuffer;

    /* 分解字符串为一组字符串 */
    str = strtok(str, SplitterChars);
    if (str == NULL)
    {
        strcpy(result, "");
        return false;
    }
    while (str != NULL)
    {
        str = strtok(NULL, SplitterChars);
        if (str != NULL)
            WantWhichOne--;
        if (WantWhichOne == 0)
        {
            strcpy(result, str);
            return true;
        }
    }

    strcpy(result, "");

    return false;
}

/**
  * @brief  提取指定位置的字符串
  * @param  InputString : 输入的字符串
  * @param  SplitterChars : 分隔符
  * @param  CountOfParameter : 参数个数
  * @param  ... : 可变参数
  * @retval true : 成功; false : 失败
  */
bool Wifi_ReturnStrings(char *InputString, char *SplitterChars, uint8_t CountOfParameter, ...)
{
    if (CountOfParameter == 0)
        return false;

    /* 提取可变参数的地址，并存入arg中 */
    va_list tag;
    va_start(tag, CountOfParameter);
    char *arg[CountOfParameter];
    for (uint8_t i = 0; i < CountOfParameter; i++)
        arg[i] = va_arg(tag, char *);
    va_end(tag);

    /* 将分离出的字符串存入可变参数中 */
    char *str;
    str = strtok(InputString, SplitterChars);
    if (str == NULL)
        return false;
    uint8_t i = 0;
    while (str != NULL)
    {
        str = strtok(NULL, SplitterChars);
        if (str != NULL)
            CountOfParameter--;
        strcpy(arg[i], str);
        i++;
        if (CountOfParameter == 0)
        {
            return true;
        }
    }
    return false;
}

/**
  * @brief  提取整型数据
  * @param  result : 分割出的结果
  * @param  WantWhichOne : 位置（分割后的第几个字符串）
  * @param  SplitterChars : 分隔符
  * @retval true : 成功; false : 失败
  */
bool Wifi_ReturnInteger(int32_t *result, uint8_t WantWhichOne, char *SplitterChars)
{
    if ((char *)Wifi.RxBuffer == NULL)
        return false;

    /* 提取字符 */
    if (Wifi_ReturnString((char *)Wifi.RxBuffer, WantWhichOne, SplitterChars) == false)
        return false;

    /* 将字符转换成整型 */
    *result = atoi((char *)Wifi.RxBuffer);

    return true;
}

/**
  * @brief  提取浮点型数据
  * @param  result : 分割出的结果
  * @param  WantWhichOne : 位置（分割后的第几个字符串）
  * @param  SplitterChars : 分隔符
  * @retval true : 成功; false : 失败
  */
bool Wifi_ReturnFloat(float *result, uint8_t WantWhichOne, char *SplitterChars)
{
    if ((char *)Wifi.RxBuffer == NULL)
        return false;

    /* 提取字符 */
    if (Wifi_ReturnString((char *)Wifi.RxBuffer, WantWhichOne, SplitterChars) == false)
        return false;

    /* 将字符转换成浮点型 */
    *result = atof((char *)Wifi.RxBuffer);

    return true;
}

/**
  * @brief  移除字符
  * @param  str : 字符串地址
  * @param  garbage : 要移除的字符串
  * @retval 无
  */
void Wifi_RemoveChar(char *str, char garbage)
{
    char *src, *dst;
    for (src = dst = str; *src != '\0'; src++)
    {
        *dst = *src;
        if (*dst != garbage)
            dst++;
    }
    *dst = '\0';
}

/**
  * @brief  清理接收缓存
  * @param  无
  * @retval 无
  */
void Wifi_RxClear(void)
{
    memset(Wifi.RxBuffer, 0, _WIFI_RX_SIZE);
    /* 清理接收索引 */
    Wifi.RxIndex = 0;

    HAL_UART_Receive_IT(&_WIFI_USART, &Wifi.usartBuff, 1);
}

/**
  * @brief  清理发送缓存
  * @param  无
  * @retval 无
  */
void Wifi_TxClear(void)
{
    memset(Wifi.TxBuffer, 0, _WIFI_TX_SIZE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        Wifi_RxCallBack();
    }
}

/**
  * @brief  串口接收回调
  * @param  无
  * @retval 无
  */
void Wifi_RxCallBack(void)
{
    if (Wifi.RxIsData == false)
    {
				//printf("RxIndex %d \r\n",Wifi.RxIndex);
        /* 未接收到+IPD */
        Wifi.RxBuffer[Wifi.RxIndex] = Wifi.usartBuff;
        if (Wifi.RxIndex < _WIFI_RX_SIZE)
            Wifi.RxIndex++;
    }
    else
    {
        /* 已接收到+IPD */
				printf("Received IPD \r\n");
        /* 接收时间步超过50ms */
        if (HAL_GetTick() - Wifi.RxDataLastTime > 50)
            Wifi.RxIsData = false;

        /* 计算接收到IPD后的数据长度 */
        if (Wifi.RxDataLen == 0)
        {
            /* 接收到+IPD后的第一笔数据 */
            if (Wifi.TcpIpMultiConnection == false)
            {
                /* 单连接模式 */
                /* 读取+IPD指令 : 前的数据，并从中抽取出数据长度 */
                Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp] = Wifi.usartBuff;
                printf("Received Data Back1 ! \r\n");

                Wifi.RxIndexForDataTmp++;
                if (Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp - 1] == ':')
                {
                    Wifi.RxDataConnectionNumber = 0;
                    Wifi.RxDataLen = atoi((char *)&Wifi.RxBufferForDataTmp[1]);
                }
            }
            else
            {
                /* 多连接模式 */
                /* 读取+IPD指令 : 前的数据，并从中抽取出数据长度 */
                Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp] = Wifi.usartBuff;
                printf("Received Data Back2 ! \r\n");

                Wifi.RxIndexForDataTmp++;
                if (Wifi.RxBufferForDataTmp[2] == ',')
                {
                    /* 获取Link ID */
                    Wifi.RxDataConnectionNumber = Wifi.RxBufferForDataTmp[1] - 48;
                }
                /* 读取+IPD指令 : 前的数据，并从中抽取出数据长度 */
                if ((Wifi.RxIndexForDataTmp > 3) && (Wifi.RxBufferForDataTmp[Wifi.RxIndexForDataTmp - 1] == ':'))
                    Wifi.RxDataLen = atoi((char *)&Wifi.RxBufferForDataTmp[3]);
            }
        }
        else
        {
            /* 校验数据长度，并填充数据 */
            Wifi.RxBufferForData[Wifi.RxIndexForData] = Wifi.usartBuff;
            printf("Received Data Back3 ! \r\n");

            if (Wifi.RxIndexForData < _WIFI_RX_FOR_DATA_SIZE)
                Wifi.RxIndexForData++;
            if (Wifi.RxIndexForData >= Wifi.RxDataLen)
            {
                Wifi.RxIsData = false;
                Wifi.GotNewData = true;
            }
        }
    }

    /* 重新打开中断 */
    HAL_UART_Receive_IT(&_WIFI_USART, &Wifi.usartBuff, 1);
//		HAL_UART_Transmit(&huart3,&Wifi.usartBuff,1,1);
//    printf(" ");
		printf(" %s ",&Wifi.usartBuff);

    /* 检测是否接收到+IPD,ESP8266接收到网络数据时向串口发送+IPD数据 */
    if (Wifi.RxIndex > 4)
    {
        if ((Wifi.RxBuffer[Wifi.RxIndex - 4] == '+') && (Wifi.RxBuffer[Wifi.RxIndex - 3] == 'I') && (Wifi.RxBuffer[Wifi.RxIndex - 2] == 'P') && (Wifi.RxBuffer[Wifi.RxIndex - 1] == 'D'))
        {
            /* 接收到+IPD */
            /* 重新初始化接收缓存 */
            memset(Wifi.RxBufferForDataTmp, 0, sizeof(Wifi.RxBufferForDataTmp));
            Wifi.RxBuffer[Wifi.RxIndex - 4] = 0;
            Wifi.RxBuffer[Wifi.RxIndex - 3] = 0;
            Wifi.RxBuffer[Wifi.RxIndex - 2] = 0;
            Wifi.RxBuffer[Wifi.RxIndex - 1] = 0;
            Wifi.RxIndex -= 4;
            Wifi.RxIndexForData = 0;
            Wifi.RxIndexForDataTmp = 0;
            Wifi.RxIsData = true;
            Wifi.RxDataLen = 0;
            Wifi.RxDataLastTime = HAL_GetTick();
        }
    }
}


/***********************************************************************************************************
***********************************************************************************************************/
/**
  * @brief  Wifi任务
  * @param  argument : 可设参数
  * @retval true : 成功; false : 失败
  */
//void WifiTask(void const *argument)
//{
//    osDelay(3000);
//    Wifi_Init();
//    //printf("*** 1 *** \r\n");
//    //Wifi_Restart();

//    Wifi_SendStringAndWait("AT\r\n", 1000);

//    Wifi_Restart();
//	
//		uint8_t result;
//    Wifi_WaitForString(_WIFI_WAIT_TIME_MED, &result, 1, "ready");

//    //printf("*** 2 *** \r\n");
//    //Wifi_SetRfPower(82);
//    //printf("*** 3 *** \r\n");
//    // Wifi_TcpIp_GetMultiConnection();
//    // Wifi_TcpIp_Close(0);
//    // Wifi_TcpIp_Close(1);
//    // Wifi_TcpIp_Close(2);
//    // Wifi_TcpIp_Close(3);
//    // Wifi_TcpIp_Close(4);
//    // Wifi_TcpIp_SetMultiConnection(true);
//    // Wifi_GetMode();
//    // Wifi_Station_DhcpIsEnable();
//    Wifi_UserInit();

//    for (;;)
//    {
//        Wifi_GetMyIp();
//        if ((Wifi.Mode == WifiMode_SoftAp) || (Wifi.Mode == WifiMode_StationAndSoftAp))
//            Wifi_SoftAp_GetConnectedDevices();
//        Wifi_TcpIp_GetConnectionStatus();
//        Wifi_RxClear();

//        for (uint8_t i = 0; i < 100; i++)
//        {
//            if (Wifi.GotNewData == true)
//            {
//                Wifi.GotNewData = false;
//                for (uint8_t ii = 0; ii < 5; ii++)
//                {
//                    if ((strstr(Wifi.TcpIpConnections[ii].Type, "UDP") != NULL) && (Wifi.RxDataConnectionNumber == Wifi.TcpIpConnections[ii].LinkId))
//                        Wifi_UserGetUdpData(Wifi.RxDataConnectionNumber, Wifi.RxDataLen, Wifi.RxBufferForData);

//                    if ((strstr(Wifi.TcpIpConnections[ii].Type, "TCP") != NULL) && (Wifi.RxDataConnectionNumber == Wifi.TcpIpConnections[ii].LinkId))
//                        Wifi_UserGetTcpData(Wifi.RxDataConnectionNumber, Wifi.RxDataLen, Wifi.RxBufferForData);
//                }
//            }
//            osDelay(10);
//        }
//        Wifi_UserProcess();
//    }
//}

/**
  * @brief  wifi任务初始化
  * @param  Priority : 任务优先级
  * @retval true : 成功; false : 失败
  */
void Wifi_Init()
{
	    Wifi_RxClear();
    Wifi_TxClear();
    HAL_UART_Receive_IT(&_WIFI_USART, &Wifi.usartBuff, 1);
    osSemaphoreDef(WifiSemHandle);
    WifiSemHandle = osSemaphoreCreate(osSemaphore(WifiSemHandle), 1);
}

/**
  * @brief  重启模块
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_Restart(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+RST\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_MED, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  进入睡眠模式
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_DeepSleep(uint16_t DelayMs)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);
    uint8_t result;
    bool returnVal = false;
    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+GSLP=%d\r\n", DelayMs);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  恢复出厂设置
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_FactoryReset(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+RESTORE\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  通过 Wi-Fi 升级软件
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_Update(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIUPDATE\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(1000 * 60 * 5, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  设置 RF TX Power 上限
  * @param  Power_0_to_82 : Power参数值
  * @retval true : 成功; false : 失败
  */
bool Wifi_SetRfPower(uint8_t Power_0_to_82)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+RFPOWER=%d\r\n", Power_0_to_82);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_MED, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);
    osSemaphoreRelease(WifiSemHandle);
    return returnVal;
}

/**
  * @brief  设置wifi模式
  * @param  WifiMode_ : wifi模式
  * @retval true : 成功; false : 失败
  */
bool Wifi_SetMode(WifiMode_t WifiMode_)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送查询指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWMODE_CUR=%d\r\n", WifiMode_);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        /* 存储wifi模式 */
        Wifi.Mode = WifiMode_;
        returnVal = true;
    } while (0);
    osSemaphoreRelease(WifiSemHandle);
    return returnVal;
}

/**
  * @brief  获取wifi的模式
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_GetMode(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送查询指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWMODE_CUR?\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        /* 存储查询到的值 */
        if (Wifi_ReturnInteger((int32_t *)&result, 1, ":"))
            Wifi.Mode = (WifiMode_t)result;
        else
            Wifi.Mode = WifiMode_Error;
        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  获取本地IP
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_GetMyIp(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送查询本地IP的指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIFSR\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        /* 存储获取到的IP */
        sscanf((char *)Wifi.RxBuffer, "AT+CIFSR\r\r\n+CIFSR:APIP,\"%[^\"]", Wifi.MyIP);
        sscanf((char *)Wifi.RxBuffer, "AT+CIFSR\r\r\n+CIFSR:STAIP,\"%[^\"]", Wifi.MyIP);

        /* 查询并存储默认网关和子网掩码 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIPSTA?\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量等于2，则失败 */
        if (result == 2)
            break;

        char *str = strstr((char *)Wifi.RxBuffer, "gateway:");
        if (str == NULL)
            break;
        if (Wifi_ReturnStrings(str, "\"", 1, Wifi.MyGateWay) == false)
            break;

        returnVal = true;
    } while (0);
    osSemaphoreRelease(WifiSemHandle);
    return returnVal;
}

/**
  * @brief  wifi 临时连接AP
  * @param  SSID : wifi 名称
  * @param  Pass : wifi密码
  * @param  MAC : MAC地址
  * @retval true : 成功; false : 失败
  */
bool Wifi_Station_ConnectToAp(char *SSID, char *Pass, char *MAC)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送连接指令 */
        Wifi_RxClear();
        printf("*** MAC = %s *** \r\n", MAC);
        //        if (MAC == NULL)
        //            sprintf((char *)Wifi.TxBuffer, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", SSID, Pass);
        //        else
        //            sprintf((char *)Wifi.TxBuffer, "AT+CWJAP_CUR=\"%s\",\"%s\",\"%s\"\r\n", SSID, Pass, MAC);
        sprintf((char *)Wifi.TxBuffer, "AT+CWJAP_CUR=\"%s\",\"%s\"\r\n", SSID, Pass);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;
        //printf("*** 1 *** \r\n");
        /* 等待接收数据，未接收到下列数据表示发送失败 */
        //        if (Wifi_WaitForString(_WIFI_WAIT_TIME_MED, &result, 3, "\r\nOK\r\n", "\r\nERROR\r\n", "\r\nFAIL\r\n") == false)
        //            break;
        //
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_MED, &result, 1, "\r\nOK\r\n") == false)
            break;
        printf("*** 2 *** \r\n");
        /* 参数匹配数量超过1，则失败 */
        if (result > 1)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  wifi station模式断开连接
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_Station_Disconnect(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWQAP\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  设置DHCP服务
  * @param  Enable : 是否使能
  * @retval true : 成功; false : 失败
  */
bool Wifi_Station_DhcpEnable(bool Enable)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWDHCP_CUR=1,%d\r\n", Enable);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        /* 存储设置数据 */
        Wifi.StationDhcp = Enable;
        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  查询是否开启DHCP服务
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_Station_DhcpIsEnable(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送查询指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWDHCP_CUR?\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        /* 处理查询打的数据 */
        if (Wifi_ReturnInteger((int32_t *)&result, 1, ":") == false)
            break;

        switch (result)
        {
        case 0:
            Wifi.StationDhcp = false;
            Wifi.SoftApDhcp = false;
            break;
        case 1:
            Wifi.StationDhcp = false;
            Wifi.SoftApDhcp = true;
            break;
        case 2:
            Wifi.StationDhcp = true;
            Wifi.SoftApDhcp = false;
            break;
        case 3:
            Wifi.StationDhcp = true;
            Wifi.SoftApDhcp = true;
            break;
        }

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  设置 ESP8266 Station 的 IP 地址
  * @param  IP : IP地址
  * @param  GateWay : 网关
  * @param  NetMask : 子网掩码
  * @retval true : 成功; false : 失败
  */
bool Wifi_Station_SetIp(char *IP, char *GateWay, char *NetMask)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIPSTA_CUR=\"%s\",\"%s\",\"%s\"\r\n", IP, GateWay, NetMask);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        Wifi.StationDhcp = false;
        returnVal = true;

    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  查询wifi连接信息
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_SoftAp_GetConnectedDevices(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);
    uint8_t result;
    bool returnVal = false;
    do
    {
        /* 查询连接到 ESP8266 SoftAP 的 Station 信息 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWLIF\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        /* 处理并存储数据 */
        Wifi_RemoveChar((char *)Wifi.RxBuffer, '\r');
        Wifi_ReturnStrings((char *)Wifi.RxBuffer, "\n,", 10,
                           Wifi.SoftApConnectedDevicesIp[0],
                           Wifi.SoftApConnectedDevicesMac[0],
                           Wifi.SoftApConnectedDevicesIp[1],
                           Wifi.SoftApConnectedDevicesMac[1],
                           Wifi.SoftApConnectedDevicesIp[2],
                           Wifi.SoftApConnectedDevicesMac[2],
                           Wifi.SoftApConnectedDevicesIp[3],
                           Wifi.SoftApConnectedDevicesMac[3],
                           Wifi.SoftApConnectedDevicesIp[4],
                           Wifi.SoftApConnectedDevicesMac[4]);
        for (uint8_t i = 0; i < 6; i++)
        {
            if ((Wifi.SoftApConnectedDevicesIp[i][0] < '0') || (Wifi.SoftApConnectedDevicesIp[i][0] > '9'))
                Wifi.SoftApConnectedDevicesIp[i][0] = 0;
            if ((Wifi.SoftApConnectedDevicesMac[i][0] < '0') || (Wifi.SoftApConnectedDevicesMac[i][0] > '9'))
                Wifi.SoftApConnectedDevicesMac[i][0] = 0;
        }

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  配置wifi为ap模式
  * @param  SSID : wifi名称
  * @param  password : 密码
  * @param  channel : 通道号
  * @param  WifiEncryptionType : 加密方式
  * @param  MaxConnections_1_to_4 : 允许最大的station数量
  * @param  HiddenSSID : 广播ssid
  * @retval true : 成功; false : 失败
  */
bool Wifi_SoftAp_Create(char *SSID, char *password, uint8_t channel, WifiEncryptionType_t WifiEncryptionType, uint8_t MaxConnections_1_to_4, bool HiddenSSID)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CWSAP=\"%s\",\"%s\",%d,%d,%d,%d\r\n", SSID, password, channel, WifiEncryptionType, MaxConnections_1_to_4, HiddenSSID);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/***********************************************************************************************************
***********************************************************************************************************/
/**
  * @brief  查询⽹网络连接信息
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_GetConnectionStatus(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送查询指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIPSTATUS\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        /* 分离并存储连接状态 */
        char *str = strstr((char *)Wifi.RxBuffer, "\nSTATUS:");
        if (str == NULL)
            break;
        str = strchr(str, ':');
        str++;
        for (uint8_t i = 0; i < 5; i++)
            Wifi.TcpIpConnections[i].status = (WifiConnectionStatus_t)atoi(str);
        str = strstr((char *)Wifi.RxBuffer, "+CIPSTATUS:");
        for (uint8_t i = 0; i < 5; i++)
        {
            sscanf(str, "+CIPSTATUS:%d,\"%3s\",\"%[^\"]\",%d,%d,%d",
                   (int *)&Wifi.TcpIpConnections[i].LinkId,
                   Wifi.TcpIpConnections[i].Type,
                   Wifi.TcpIpConnections[i].RemoteIp,
                   (int *)&Wifi.TcpIpConnections[i].RemotePort,
                   (int *)&Wifi.TcpIpConnections[i].LocalPort,
                   (int *)&Wifi.TcpIpConnections[i].RunAsServer);
            str++;
            str = strstr(str, "+CIPSTATUS:");
            if (str == NULL)
                break;
        }
        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  ping
  * @param  PingTo : ip地址
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_Ping(char *PingTo)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+PING=\"%s\"\r\n", PingTo);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_MED, &result, 3, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        /* 存储ping的回答 */
        if (Wifi_ReturnInteger((int32_t *)&Wifi.TcpIpPingAnswer, 2, "+") == false)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}
/**
  * @brief  设置多连接
  * @param  EnableMultiConnections : 是否设置多连接
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_SetMultiConnection(bool EnableMultiConnections)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送设置指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIPMUX=%d\r\n", EnableMultiConnections);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        /* 设置的值 */
        Wifi.TcpIpMultiConnection = EnableMultiConnections;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  查询多链接
  * @param  无
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_GetMultiConnection(void)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送查询指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIPMUX?\r\n");
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量超过2，则失败 */
        if (result == 2)
            break;

        if (Wifi_ReturnInteger((int32_t *)&result, 1, ":") == false)
            break;

        /* 存储查询到值 */
        Wifi.TcpIpMultiConnection = (bool)result;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}
/**
  * @brief  建立TCP连接
  * @param  linkid : ？
  * @param  RemoteIp : IP地址
  * @param  RemotePort : 端口
  * @param  TimeOut : 超时时间
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_StartTcpConnection(uint8_t LinkId, char *RemoteIp, uint16_t RemotePort, uint16_t TimeOut)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        // /* 建立TCP连接 */
        // Wifi_RxClear();
        // sprintf((char *)Wifi.TxBuffer, "AT+CIPSERVER=1,%d\r\n", RemotePort);
        // if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
        //     break;

        // if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
        //     break;
        // if (result == 2)
        //     break;

        /* 发送建立TCP传输质指令 */
        Wifi_RxClear();
        if (Wifi.TcpIpMultiConnection == false)
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSTART=\"TCP\",\"%s\",%d,%d\r\n", RemoteIp, RemotePort, TimeOut);
        else
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSTART=%d,\"TCP\",\"%s\",%d,%d\r\n", LinkId, RemoteIp, RemotePort, TimeOut);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_HIGH, &result, 3, "OK", "CONNECT", "ERROR") == false)
            break;

        /* 参数匹配数量超过3，则失败 */
        if (result == 3)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  建立UDP连接
  * @param  LinkId : 连接ID号
  * @param  RemoteIp : IP地址
  * @param  RemotePort : 端口
  * @param  TimeOut : 超时时间
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_StartUdpConnection(uint8_t LinkId, char *RemoteIp, uint16_t RemotePort, uint16_t LocalPort)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送建立UDP传输质指令 */
        Wifi_RxClear();
        if (Wifi.TcpIpMultiConnection == false)
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSTART=\"UDP\",\"%s\",%d,%d\r\n", RemoteIp, RemotePort, LocalPort);
        else
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSTART=%d,\"UDP\",\"%s\",%d,%d\r\n", LinkId, RemoteIp, RemotePort, LocalPort);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_HIGH, &result, 3, "OK", "ALREADY", "ERROR") == false)
            break;

        /* 参数匹配数量为3，则失败 */
        if (result == 3)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  关闭TCP/UDP/SSL传输
  * @param  LinkId ： 连接ID号
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_Close(uint8_t LinkId)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送关闭指令 */
        Wifi_RxClear();
        if (Wifi.TcpIpMultiConnection == false)
            sprintf((char *)Wifi.TxBuffer, "AT+CIPCLOSE\r\n");
        else
            sprintf((char *)Wifi.TxBuffer, "AT+CIPCLOSE=%d\r\n", LinkId);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量为2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);
    osSemaphoreRelease(WifiSemHandle);
    return returnVal;
}

/**
  * @brief  打开TCP服务器
  * @param  PortNumber : 端口号
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_SetEnableTcpServer(uint16_t PortNumber)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送打开TCP服务器指令 */
        Wifi_RxClear();
        if (Wifi.TcpIpMultiConnection == false)
        {
            /* 如果是单连接模式，先将其切换为多连接（服务器模式只支持多连接） */
            sprintf((char *)Wifi.TxBuffer, "AT+CIPMUX=1\r\n");
            if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
                break;
            if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
                break;
            Wifi.TcpIpMultiConnection = true;
            Wifi_RxClear();
        }
        else
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSERVER=1,%d\r\n", PortNumber);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量为2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  关闭TCP服务器
  * @param  PortNumber : 端口号
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_SetDisableTcpServer(uint16_t PortNumber)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 发送关闭TCP服务器指令 */
        Wifi_RxClear();
        sprintf((char *)Wifi.TxBuffer, "AT+CIPSERVER=0,%d\r\n", PortNumber);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 参数匹配数量为2，则失败 */
        if (result == 2)
            break;

        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}

/**
  * @brief  发送UDP数据
  * @param  Link Id : 连接ID号
  * @param  DataLen : 数据长度
  * @param  Data : 数据地址
  * @retval true : 成功; false : 失败
  */
bool Wifi_TcpIp_SendDataUdp(uint8_t LinkId, uint16_t dataLen, uint8_t *data)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 数据长度写入TCP发包缓存 */
        Wifi_RxClear();
        if (Wifi.TcpIpMultiConnection == false)
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSERVER=0\r\n");
        else
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSEND=%d,%d\r\n", LinkId, dataLen);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, ">", "ERROR") == false)
            break;

        /* 参数匹配数量为2，则失败 */
        if (result == 2)
            break;

        /* 数据发送 */
        Wifi_RxClear();
        Wifi_SendRaw(data, dataLen);

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 发送成功 */
        returnVal = true;
    } while (0);
    osSemaphoreRelease(WifiSemHandle);
    return returnVal;
}

/**
  * @brief  发送TCP数据
  * @param  Link Id : 连接ID号
  * @param  DataLen : 数据长度
  * @param  Data : 数据地址
  * @retval true : 成功; false : 失败 
  */
bool Wifi_TcpIp_SendDataTcp(uint8_t LinkId, uint16_t dataLen, uint8_t *data)
{
    osSemaphoreWait(WifiSemHandle, osWaitForever);

    uint8_t result;
    bool returnVal = false;

    do
    {
        /* 数据长度写入TCP发包缓存 */
        Wifi_RxClear();
        if (Wifi.TcpIpMultiConnection == false)
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSENDBUF=%d\r\n", dataLen);
        else
            sprintf((char *)Wifi.TxBuffer, "AT+CIPSENDBUF=%d,%d\r\n", LinkId, dataLen);
        if (Wifi_SendString((char *)Wifi.TxBuffer) == false)
            break;

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 3, ">", "ERROR", "busy") == false)
            break;

        /* 参数匹配数量超过1，则失败 */
        if (result > 1)
            break;

        /* 数据发送 */
        Wifi_RxClear();
        Wifi_SendRaw(data, dataLen);

        /* 等待接收数据，未接收到下列数据表示发送失败 */
        if (Wifi_WaitForString(_WIFI_WAIT_TIME_LOW, &result, 2, "OK", "ERROR") == false)
            break;

        /* 发送成功 */
        returnVal = true;
    } while (0);

    osSemaphoreRelease(WifiSemHandle);

    return returnVal;
}
