#ifndef _WIFI_H
#define _WIFI_H
/* Includes ------------------------------------------------------------------*/
#include "WifiConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "usart.h"
#include "gpio.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* Exported types ------------------------------------------------------------*/
/**
  * @brief  wifi工作模式
  */
typedef enum
{
    WifiMode_Error = 0,
    WifiMode_Station = 1,
    WifiMode_SoftAp = 2,
    WifiMode_StationAndSoftAp = 3,
} WifiMode_t;

/**
  * @brief  wifi加密方式
  */
typedef enum
{
    WifiEncryptionType_Open = 0,
    WifiEncryptionType_WPA_PSK = 2,
    WifiEncryptionType_WPA2_PSK = 3,
    WifiEncryptionType_WPA_WPA2_PSK = 4,
} WifiEncryptionType_t;

/**
  * @brief  wifi连接状态
  */
typedef enum
{
    WifiConnectionStatus_Error = 0,
    WifiConnectionStatus_GotIp = 2,
    WifiConnectionStatus_Connected = 3,
    WifiConnectionStatus_Disconnected = 4,
    WifiConnectionStatus_ConnectionFail = 5,
} WifiConnectionStatus_t;

/**
  * @brief  TCP/IP连接信息结构体
  */
typedef struct
{
    WifiConnectionStatus_t status;
    uint8_t LinkId;
    char Type[4];
    char RemoteIp[17];
    uint16_t RemotePort;
    uint16_t LocalPort;
    bool RunAsServer;
} WifiConnection_t;

/**
  * @brief  wifi结构体
  */
typedef struct
{
    //----------------Usart	Paremeter
    uint8_t usartBuff;                               /* 接收到的单个数据 */
    uint8_t RxBuffer[_WIFI_RX_SIZE];                 /* 接收数据缓存 */
    uint8_t TxBuffer[_WIFI_TX_SIZE];                 /* 发送数据缓存 */
    uint16_t RxIndex;                                /* 接收数据索引 */
    uint8_t RxBufferForData[_WIFI_RX_FOR_DATA_SIZE]; /* +IPD数据缓存 */
    uint8_t RxBufferForDataTmp[8];                   /* 临时读取+IPD指令缓存 */
    uint8_t RxIndexForDataTmp;                       /* 临时读取+IPD指令缓存索引 */
    uint16_t RxIndexForData;                         /* +IPD数据索引 */
    uint16_t RxDataLen;                              /* +IPD数据长度 */
    uint8_t RxDataConnectionNumber;                  /* Link ID */
    uint32_t RxDataLastTime;                         /* 接收到+IPD的时间 */
    bool RxIsData;                                   /* 是否已接收到+IPD */
    bool GotNewData;                                 /* 接收到数据 */
    //----------------General	Parameter
    WifiMode_t Mode;    /* wifi模式 */
    char MyIP[16];      /* 本地IP */
    char MyGateWay[16]; /* 本地网关 */
    //----------------Station	Parameter
    bool StationDhcp; /* STA模式是否打开DNCP服务 */
    char StationIp[16];
    //----------------SoftAp Parameter
    bool SoftApDhcp;                       /* AP模式是否打开DNCP服务 */
    char SoftApConnectedDevicesIp[6][16];  /* 连接到AP的子设备IP地址 */
    char SoftApConnectedDevicesMac[6][18]; /* 连接到AP的子设备MAC地址 */
    //----------------TcpIp Parameter
    bool TcpIpMultiConnection;            /* TCP 单/多连接标志 */
    uint16_t TcpIpPingAnswer;             /* ping应答信息 */
    WifiConnection_t TcpIpConnections[5]; /* TCP/IP连接信息 */
                                          //----------------
} Wifi_t;

extern Wifi_t Wifi;
/* Exported constants --------------------------------------------------------*/

#define USER_DEBUG 1
/* Exported macro ------------------------------------------------------------*/
/* 用户调试信息 */
#if (USER_DEBUG == 1)
#define LOG_USER_DEBUG(fmt, ...)               \
    do                                         \
    {                                          \
        printf("[%s:%d]", __func__, __LINE__); \
        printf(fmt, ##__VA_ARGS__);            \
    } while (0)
#else
#define LOG_USER_DEBUG(fmt, ...) \
    {                            \
        (void)fmt;               \
    }
#endif

/* 发送调试信息 */
#if (SEND_DEBUG == 1)
#define LOG_SEND_DEBUG(fmt, ...)    \
    do                              \
    {                               \
        printf(">> ");              \
        printf(fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_SEND_DEBUG(fmt, ...) \
    {                            \
        (void)fmt;               \
    }
#endif

/* 接收调试信息 */
#if (RECV_DEBUG == 1)
#define LOG_RECV_DEBUG(fmt, ...)    \
    do                              \
    {                               \
        printf(fmt, ##__VA_ARGS__); \
    } while (0)
#else
#define LOG_RECV_DEBUG(fmt, ...) \
    {                            \
        (void)fmt;               \
    }
#endif

/* Exported functions prototypes ---------------------------------------------*/
/* wifi */
void Wifi_UserInit(void);
void Wifi_UserProcess(void);
void Wifi_UserGetUdpData(uint8_t LinkId, uint16_t DataLen, uint8_t *Data);
void Wifi_UserGetTcpData(uint8_t LinkId, uint16_t DataLen, uint8_t *Data);

/* 包含在串口接收中断中的函数 */
void Wifi_RxCallBack(void);

/* wifi任务初始化 */
void Wifi_Init(void);

/* 设置wifi状态 */
bool Wifi_Restart(void);
bool Wifi_DeepSleep(uint16_t DelayMs);
bool Wifi_FactoryReset(void);
bool Wifi_Update(void);
bool Wifi_SetRfPower(uint8_t Power_0_to_82);

/* 设置wifi工作模式 */
bool Wifi_SetMode(WifiMode_t WifiMode_);
bool Wifi_GetMode(void);
bool Wifi_GetMyIp(void);

/* wifi station模式相关函数 */
bool Wifi_Station_ConnectToAp(char *SSID, char *Pass, char *MAC);
bool Wifi_Station_Disconnect(void);
bool Wifi_Station_DhcpEnable(bool Enable);
bool Wifi_Station_DhcpIsEnable(void);
bool Wifi_Station_SetIp(char *IP, char *GateWay, char *NetMask);

/* wifi softap模式相关函数 */
bool Wifi_SoftAp_GetConnectedDevices(void);
bool Wifi_SoftAp_Create(char *SSID, char *password, uint8_t channel, WifiEncryptionType_t WifiEncryptionType, uint8_t MaxConnections_1_to_4, bool HiddenSSID);

/* TCP/IP网络服务相关函数 */
bool Wifi_TcpIp_GetConnectionStatus(void);
bool Wifi_TcpIp_Ping(char *PingTo);
bool Wifi_TcpIp_SetMultiConnection(bool EnableMultiConnections);
bool Wifi_TcpIp_GetMultiConnection(void);
bool Wifi_TcpIp_StartTcpConnection(uint8_t LinkId, char *RemoteIp, uint16_t RemotePort, uint16_t TimeOut_S);
bool Wifi_TcpIp_StartUdpConnection(uint8_t LinkId, char *RemoteIp, uint16_t RemotePort, uint16_t LocalPort);
bool Wifi_TcpIp_Close(uint8_t LinkId);
bool Wifi_TcpIp_SetEnableTcpServer(uint16_t PortNumber);
bool Wifi_TcpIp_SetDisableTcpServer(uint16_t PortNumber);
bool Wifi_TcpIp_SendDataUdp(uint8_t LinkId, uint16_t dataLen, uint8_t *data);
bool Wifi_TcpIp_SendDataTcp(uint8_t LinkId, uint16_t dataLen, uint8_t *data);

/* Private defines -----------------------------------------------------------*/

#endif
