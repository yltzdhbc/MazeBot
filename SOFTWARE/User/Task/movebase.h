#ifndef __MOVEBASE__H
#define __MOVEBASE__H


/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stm32f4xx.h"
/* Exported types ------------------------------------------------------------*/

//轮子到中心距离
#define WHEEL_DISTANCE_TO_CENTER 0.32f	//0.087m

/* Encoder PulseMode type structure definition */
typedef struct
{
  int32_t Cnt;
  float Vel;
} PulseModeType;

/* Encoder TimeMode type structure definition */
typedef struct
{
  int32_t Tim; //间隔包含的100us的个数
  float TimUite;
  int32_t TimNum; //间隔总时间 单位：us
  float Vel;
} TimeModeType;

/* Encoder type structure definition */
typedef struct
{
  TimeModeType TimeMode;
  PulseModeType PulseMode;
  int32_t period;
  int32_t Direct;
} EncoderType;

/* Commutation type structure definition */
typedef struct
{
  int32_t Mode;
} CommutationType;

/* Commutation type structure definition */
typedef struct
{
  // int32_t can_status;
  // int32_t canId;
  int32_t status;
  int32_t Id;
} CommandType;

/* CurCtrl type structure definition */
typedef struct
{
  float current;
  float desiredCur;
  float curErr;
  float acc;
  float dec;
  float kp;
  float ki;
  float iOut;
  float output;
  float maxOutput;
} CurCtrlType;

/* VelCtrl type structure definition */
typedef struct
{
  float speed;
  float desiredVel[3];
  float velErr;
  float acc;
  float dec;
  float kp;
  float ki;
  float iOut;
  float output;
  float maxOutput;
} VelCtrlType;

/* PosCtrl type structure definition */
typedef struct
{
  float actualPos;
  float desiredPos;
  float posErr, posErrLast;
  float posVel, acc;
  float basicPos;
  float kp;
  float kd;
  float output;
} PosCtrlType;

/* HomingMode type structure definition */
typedef struct
{
  float vel;
  float current;
  float initPos;
  int32_t cnt;
  float output;
} HomingModeType;

/* Driver type structure definition */
typedef struct
{
  uint32_t unitMode;
  int32_t time;
  FunctionalState status;
  float output;
  VelCtrlType velCtrl;
  PosCtrlType posCtrl;
  CurCtrlType curCtrl;
  HomingModeType homingMode;
  EncoderType encoder;
  CommutationType commutation;
  CommandType command;
	//BoardType_e boardType;
} DriverType;

/* Control type structure definition */
typedef struct
{
  float DesiredValue[8];
  float KP[4];
  float KI[4];
} CtrlType;

typedef enum
{
  DC_MOTOR = 0,
	STEPPER = 1,
  NONE = 2 
} Motor_e;

/*  Motor type structure definition */
typedef struct
{
	uint16_t last_tick;      //上一次的编码器脉冲
	int16_t round_cnt ;       //累计圈数（编码器溢出的次数）
	int32_t total_tick ;      //总脉冲（绝对脉冲）
	int32_t last_total_tick ; //上一次的总脉冲（绝对）
	float total_tick_diff ; //两次总脉冲的差
	int32_t pos_diff;
	int32_t last_pos;

	Motor_e type;
  int32_t pos, posLast;
  float vel; //电机返回的速度，单位： rpm
  int16_t cur; //电机返回的电流值
  int8_t temp; //电机的温度
} MotorType;

typedef struct
{
  float x;
  float y;
  float theta;
} OdomPos_t;

typedef struct
{
  float vx;
  float vy;
  float wz;
} OdomVel_t;

typedef struct
{
  OdomPos_t Pos;
  OdomVel_t Vel;
} Odom_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/********************FOC参数******************************/
#define VOL_AMP 1.10f          //Voltage amplitude 作用于电流的电压幅值约0.8--13.6A
#define VOL_MAX 18.00f         //电压最大值
#define VOL_BLIND_AREA 0.80f   //输出盲区，电机不动的最大矢量电压值
#define EMF_CONSTANT 0.020926f //电动势常数，电机本身参数 = 电压矢量(V)/速度(pulse/ms)

#define CURRENT_MAX_DC 1000.0f
#define CURRENT_MAX_STEPPER 6.0f

#define VEL_MAX_DC 1000.0f //最大速度

#define VEL_KP_DC 1.2f     //速度环Kp
#define VEL_KI_DC 0.1f   //速度环Ki

#define POS_KP_DC 0.11f//位置环Kp
#define POS_KD_DC 0.0f//位置环Kp

#define VEL_MAX_STEPPER 2400.0f //最大速度
#define VEL_KP_STEPPER 0.05f    //速度环Kp
#define VEL_KI_STEPPER 0.003f   //速度环Ki
#define POS_KP_STEPPER 0.027f
#define POS_KD_STEPPER 0.05f

//驱动器工作模式
#define SPEED_CONTROL_MODE 2
#define POSITION_CONTROL_MODE 5
#define HOMING_MODE 6
//区别使用斜坡前后的速度
#define CMD 0
#define SOFT 1
#define MAX_V 2

//换相模式
#define BLDC_MODE 1
#define FOC_MODE 2

#define CAN_ID_NUM 5
//自动5号初始  电流为2.5其余为1.5

extern Odom_t Odom;

/* Exported functions ------------------------------------------------------- */
float OutPutLim(float val);
float VelSlope(VelCtrlType *velPid);
float VelPidCtrl(VelCtrlType *velPid);
float PosCtrl(PosCtrlType *posPid);
//float 	VelCtrl(float cmdVel);
void VelCtrlInit(void);
void PosCtrlInit(void);
//float	 	CalculSpeed(void);
float CalculSpeed_Pos(DriverType *driver, MotorType *motor);
float GetVelPidOut(void);
float GetSpeed(void);
float GetPosPidOut(void);
float MaxMinLimit(float val, float limit);
void DriverInit(void);
void MotorCtrl(void);
void HomingMode(DriverType *driver);
void HomingModeInit(void);
void MotorOn(int n);
void MotorOff(int n);
void VelCtrlTest(float vel, int tim);

void EXTI_DIR_Handler(void);
void EXTI_ENABLE_Handler(void);
void UpdataEncoderInfo(MotorType *Motor, uint16_t tick, uint32_t d_time);
void UpdateStepInfo(DriverType *Driver, uint16_t tick, uint32_t d_time);
void SetPwmMotor0(float PwmInput);
void SetPwmMotor1(float PwmInput);

void calcOdometry(Odom_t *Odom, uint32_t diff_time);

float CurPidCtrl(CurCtrlType *curPid);

void Output_Wheel(float vel_x, float vel_y, float w_z);

#endif
