#include <math.h>
#include <stdio.h>
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "bsp_usart.h"
#include "bsp_led.h"
#include "movebase.h"
#include "cmsis_os.h"

#define MOTOR_NUM 2
#define TICK2ANGLE 0.42857f     //脉冲与角度的转换系数
#define RPM2MPS 0.00178023f     // 2PIR/60  R=0.017
#define MPS2RPM 561.723338f     // 60/2PIR  R=0.017

#define TICK2RAD 0.00747998238f //TICK*2PI/840 = RAD
#define WHEEL_SEPARATION 0.087f //左右轮间距
#define WHEEL_RADIUS 0.017f     //轮子半径

Odom_t Odom;
MotorType Motor[MOTOR_NUM];
DriverType Driver[MOTOR_NUM] = {0};
float PerPwm[MOTOR_NUM] = {0};

//*****************      m/s          m/s         rad/s
void Output_Wheel(float vel_x, float vel_y, float w_z)
{
  float Vel_l, Vel_r;

  Vel_l = vel_y - w_z * WHEEL_SEPARATION / 2;
  Vel_r = vel_y + w_z * WHEEL_SEPARATION / 2;

  Driver[0].velCtrl.desiredVel[CMD] = Vel_l * MPS2RPM;
  Driver[1].velCtrl.desiredVel[CMD] = Vel_r * MPS2RPM;
}

extern float ADC_Value_Temp[6];

#include "adc.h"
uint16_t ADC_Value;

void ChassisTask(void const *argument)
{

  uint32_t time_now, prev_time, d_time;
  DriverInit(); //驱动器初始化

  for (;;)
  {
//	  HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, 50);
// 
//    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
//    {
//      ADC_Value = HAL_ADC_GetValue(&hadc1);
//      printf("ADC Reading : %d \r\n",ADC_Value);
//		}
		//printf("%f , %f , %f , %f , %f , %f\r\n",ADC_Value_Temp[0],ADC_Value_Temp[1],ADC_Value_Temp[2],ADC_Value_Temp[3],ADC_Value_Temp[4],ADC_Value_Temp[5]);
    /*获取当前时间 计算周期时间差 记录*/
    time_now = HAL_GetTick();
    d_time = time_now - prev_time;
    prev_time = time_now;

    /*读取编码器信息 获得反馈 速度、位置*/
    UpdataEncoderInfo(&Motor[0], 65535 - __HAL_TIM_GET_COUNTER(&MOTOR0_TIM), d_time);
    UpdataEncoderInfo(&Motor[1], __HAL_TIM_GET_COUNTER(&MOTOR1_TIM), d_time);

    MotorCtrl(); //电机控制

    calcOdometry(&Odom, d_time); //计算里程计

    osDelay(10);
  }
}

void calcOdometry(Odom_t *Odom, uint32_t diff_time)
{
  float wheel_l, wheel_r; // 运动弧长 [rad]
  float delta_s, delta_theta;
  float step_time;

  step_time = diff_time / 1000.0f; //MS转换为S
  if (step_time == 0)
    return;

  wheel_l = (float)Motor[0].total_tick_diff * TICK2RAD * WHEEL_RADIUS; //左轮运动弧长 m
  wheel_r = (float)Motor[1].total_tick_diff * TICK2RAD * WHEEL_RADIUS; //又轮运动弧长 m

  delta_s = (wheel_l + wheel_r) / 2;                    //计算单位弧长
  delta_theta = (wheel_r - wheel_l) / WHEEL_SEPARATION; //计算偏转角

  Odom->Pos.theta += delta_theta;
  Odom->Pos.y += delta_s * cos(Odom->Pos.theta);
  Odom->Pos.x -= delta_s * sin(Odom->Pos.theta);

  Odom->Vel.wz = (Motor[1].vel - Motor[0].vel) * RPM2MPS / WHEEL_SEPARATION;
  Odom->Vel.vy = (Motor[1].vel + Motor[0].vel) * RPM2MPS / 2;
  Odom->Vel.vx = 0.0f;
}

/**
  * @brief  电机控制函数
  * @brief  放在定时器定时中断回调函数中 使用中断运行
	* @param  None
	* @retval None
  */
void MotorCtrl(void)
{
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    if (Motor[i].type == NONE)
      break;

    /*计算最后的期望位置 有位置判断 防溢出*/
    CalculSpeed_Pos(&Driver[i], &Motor[i]);

    if (Driver[i].status != ENABLE)
    {
      Driver[i].output = 0.0f;
      continue;
    }

    /*驱动器模式选择状态机*/
    switch (Driver[i].unitMode)
    {
    case POSITION_CONTROL_MODE: //位置控制
      /*进行位置环计算 输入期望位置 输入反馈位置 输出速度控制量*/
      PosCtrl(&Driver[i].posCtrl);
      /*将位置环的输出传递给速度环的期望速度*/
      Driver[i].velCtrl.desiredVel[CMD] = Driver[i].posCtrl.output;
      /*速度斜坡输入 */
      VelSlope(&Driver[i].velCtrl);
      /*进行速度环计算 输入期望速度 输入反馈速度 输出电流环控制量*/
      Driver[i].output = VelPidCtrl(&Driver[i].velCtrl);
      break;
    case SPEED_CONTROL_MODE: //速度控制
      /*速度斜坡输入*/
      VelSlope(&Driver[i].velCtrl);
      /*进行速度环计算 输入期望速度 输入反馈速度 输出电流环控制量*/
      Driver[i].output = VelPidCtrl(&Driver[i].velCtrl);
      break;
    case HOMING_MODE:
      HomingMode(&Driver[i]); //零点校准
      Driver[i].output = Driver[i].homingMode.output;
      break;
    default:
      break;
    }
  }

  /*驱动输出控制量给到驱动器控制电机*/
  for (int i = 0; i < MOTOR_NUM; i++)
  {
    if (Motor[i].type == DC_MOTOR)
    {
      PerPwm[i] = Driver[i].output * 1.0f;
    }
    else
      PerPwm[i] = 0.0f;
  }

  /*设定电机Pwm值*/
  SetPwmMotor0(PerPwm[0]);
  SetPwmMotor1(PerPwm[1]);
}

/**
  * @brief  驱动器初始化
  * @param  None
  * @retval 在main.c中初始调用
  */
void DriverInit(void)
{
  Driver[0].command.Id = 0;
  Driver[1].command.Id = 1;
  Motor[0].type = DC_MOTOR;
  Motor[1].type = DC_MOTOR;

  for (int i = 0; i < MOTOR_NUM; i++)
  {
    Driver[i].status = ENABLE;        //驱动状态
    Driver[i].encoder.period = 65535; //编码器溢出值

    if (Motor[i].type == DC_MOTOR)
    {
      Driver[i].unitMode = SPEED_CONTROL_MODE;
      Driver[i].velCtrl.kp = VEL_KP_DC;
      Driver[i].velCtrl.ki = VEL_KI_DC;
      Driver[i].velCtrl.maxOutput = CURRENT_MAX_DC;
      Driver[i].velCtrl.desiredVel[MAX_V] = VEL_MAX_DC;
      Driver[i].posCtrl.kd = POS_KD_DC;
      Driver[i].posCtrl.kp = POS_KP_DC;
      Driver[i].homingMode.current = 0.8f;
      Driver[i].velCtrl.acc = 50.0f;
      Driver[i].velCtrl.dec = 50.0f;
      Driver[i].velCtrl.desiredVel[CMD] = 0.0f;
      Driver[i].posCtrl.acc = Driver[i].velCtrl.dec;
      Driver[i].posCtrl.posVel = 50.0f;
      Driver[i].homingMode.vel = -160.0f;
    }
    else
    {
      break;
    }
  }
}

/**
  * @brief  更新编码器输入脉冲信息
  * @param  None
  * @retval tick是编码器返回的脉冲数值 范围为0-65536
  *         last_tick记录上一个周期的脉冲值 判断脉冲是否溢出
  */
void UpdataEncoderInfo(MotorType *Motor, uint16_t tick, uint32_t d_time)
{
  //判断编码器脉冲计数溢出
  if ((int32_t)(tick - Motor->last_tick) > 32768)
    Motor->round_cnt--;
  else if ((int32_t)(tick - Motor->last_tick) < -32768)
    Motor->round_cnt++;
  //保存本周期的脉冲
  Motor->last_tick = tick;
  //计算编码器的绝对脉冲
  Motor->total_tick = Motor->round_cnt * 65536 + tick;
  //计算绝对脉冲的差值
  Motor->total_tick_diff = Motor->total_tick - Motor->last_total_tick;
  //保存本周期的绝对脉冲值
  Motor->last_total_tick = Motor->total_tick;
  //计算总转角 度
  Motor->pos = Motor->total_tick * TICK2ANGLE;
  // //一个周期的角 度
  // Motor->pos_diff = Motor->pos - Motor->last_pos;
  // Motor->last_pos = Motor->pos;
  //计算速度 RPM
  Motor->vel = (Motor->total_tick_diff / 840.0f) * (60000.0f / d_time);
}
/**
  * @brief  设定电机1 Pwm值 控制电机速度
  * @param  输入Pwm值
  * @retval none
  */
void SetPwmMotor0(float PwmInput)
{
  float PwmOutput;
  if (PwmInput == 0)
  {
    PwmOutput = 0;
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  }
  else if (PwmInput > 0)
  {
    PwmOutput = PwmInput;
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
  }
  else
  {
    PwmOutput = -PwmInput;
    HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  }

  __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR0_PWM_TIM_CHANNEL, PwmOutput);
}

/**
  * @brief  设定电机2 Pwm值 控制电机速度
  * @param  输入Pwm值
  * @retval none
  */
void SetPwmMotor1(float PwmInput)
{
  float PwmOutput;
  if (PwmInput == 0)
  {
    PwmOutput = 0;
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  }
  else if (PwmInput > 0)
  {
    PwmOutput = PwmInput;
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    PwmOutput = -PwmInput;
    HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
  }

  __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR1_PWM_TIM_CHANNEL, PwmOutput);
}

/**
  * @brief  速度斜坡输入
  * @param  None
  * @retval 速度期望输出
  */
float VelSlope(VelCtrlType *velPid)
{
  /*************计算加减速度斜坡**************/
  if (velPid->desiredVel[SOFT] < (velPid->desiredVel[CMD] - velPid->acc))
  {
    velPid->desiredVel[SOFT] += velPid->acc;
  }
  else if (velPid->desiredVel[SOFT] > (velPid->desiredVel[CMD] + velPid->dec))
  {
    velPid->desiredVel[SOFT] -= velPid->dec;
  }
  else
  {
    velPid->desiredVel[SOFT] = velPid->desiredVel[CMD];
  }
  return velPid->desiredVel[SOFT];
}

/**
  * @brief  速度控制
  * @param  None
  * @retval 速度PID的输出
  */
float VelPidCtrl(VelCtrlType *velPid)
{
  /*****************速度环PID*****************/
  velPid->velErr = velPid->desiredVel[SOFT] - velPid->speed;
  //计算积分
  velPid->iOut += velPid->ki * velPid->velErr;
  //积分限幅
  velPid->iOut = MaxMinLimit(velPid->iOut, velPid->maxOutput);
  //计算输出
  velPid->output = velPid->kp * velPid->velErr + velPid->iOut;
  //输出限幅
  velPid->output = MaxMinLimit(velPid->output, velPid->maxOutput);

  return velPid->output;
}

/**
  * @brief  限制输出幅值
  * @param  val：输入值
  * @retval 输出值
  */
float OutPutLim(float value)
{
  float outputMax, outputMin, outputBasic;
  /********************计算动态最大最小输出****************************/
  outputBasic = Driver[0].velCtrl.speed * EMF_CONSTANT; //估算反电动势
  outputMax = outputBasic + VOL_AMP;                    //输出幅度
  outputMin = outputBasic - VOL_AMP;                    //需要根据速度与电压关系改变
  if (outputMax < VOL_AMP)
    outputMax = VOL_AMP; //
  if (outputMin > -VOL_AMP)
    outputMin = -VOL_AMP;

  if (value < outputMin)
    value = outputMin; //
  if (value > outputMax)
    value = outputMax;

  if (value > VOL_MAX)
    value = VOL_MAX;
  if (value < -VOL_MAX)
    value = -VOL_MAX;

  //	CurrentOutput = (value - (float)velpms*0.04315f)*25.0f;
  if (value < 0)
    value -= VOL_BLIND_AREA; //消除控制盲区0.3043f Vq
  else
    value += VOL_BLIND_AREA;

  return value;
}

/**
  * @brief  位置控制(新位置环程序)
  * @param  None
  * @retval 位置环PID的输出。
  */
float PosCtrl(PosCtrlType *posPid)
{
  float posPidOut = 0.0f;
  float desiredVel = 0.0f, signVel = 1.0f;

  /******************************计算位置环输出**************************************/
  posPid->posErr = posPid->desiredPos - posPid->actualPos;
  posPidOut = posPid->posErr * posPid->kp + posPid->kd * (posPid->posErr - posPid->posErrLast);
  posPid->posErrLast = posPid->posErr;

  if (posPid->posErr < 0.0f)
    signVel = -1.0f;

  //乘以0.7是因为减速需要有裕量，有待优化（斜坡问题）
  desiredVel = signVel * sqrtf(2.0f * 0.7f * posPid->acc * signVel * posPid->posErr);

  if (fabsf(desiredVel) < fabsf(posPidOut))
    posPidOut = desiredVel;

  //给一定大小的死区
  //if(fabsf(posPid->posErr) <= 200.0f)		posPidOut = 0.0f;

  posPid->output = MaxMinLimit(posPidOut, posPid->posVel);

  return posPid->output;
}

/**
  * @brief  Homing mode
  * @param  None
  * @retval 输出的值
  */

void HomingMode(DriverType *driver)
{
  float output;

  driver->velCtrl.desiredVel[SOFT] = driver->homingMode.vel;
  output = VelPidCtrl(&driver->velCtrl);

  driver->homingMode.output = MaxMinLimit(output, driver->homingMode.current); //限制home模式时电流值

  if (fabsf(driver->velCtrl.speed) <= 2)
  {
    driver->homingMode.cnt++;
  }
  else
  {
    driver->homingMode.cnt = 0;
  }

  if (driver->homingMode.cnt >= 500)
  {

    driver->posCtrl.actualPos = 0.0f;
    driver->posCtrl.desiredPos = driver->posCtrl.actualPos + 8192.0f;
    //清除输出
    driver->homingMode.output = 0.0f;
    driver->velCtrl.desiredVel[CMD] = 0.0f;
    driver->velCtrl.desiredVel[SOFT] = 0.0f;
    driver->velCtrl.output = 0.0f;
    driver->output = 0.0f;
    driver->homingMode.output = 0.0f;
    driver->velCtrl.iOut = 0.0f;
    driver->unitMode = POSITION_CONTROL_MODE;
  }
}

/**
  * @brief  传递输出电压
  * @param  None
  * @retval 位置环输出的值
  */
float GetPosPidOut(void)
{
  return Driver[0].posCtrl.output;
}

/**
  * @brief  Calculate Speed
  * @param  None
  * @retval Subtraction number between every two times.
**/
float CalculSpeed_Pos(DriverType *driver, MotorType *motor)
{
  int deltaPos = 0;
  deltaPos = (motor->pos - motor->posLast);
  motor->posLast = motor->pos;
  if (deltaPos > (driver->encoder.period / 2))
    deltaPos -= driver->encoder.period;
  if (deltaPos < -(driver->encoder.period / 2))
    deltaPos += driver->encoder.period;

  driver->posCtrl.actualPos += deltaPos;

  //用反馈速度输入
  driver->velCtrl.speed = (float)(motor->vel); //* 0.1365333f; //1/60*8192/1000=0.136533

  //用位置差分出的速度输入
  //driver->velCtrl.speed = speed;

  return driver->velCtrl.speed;
}
/**
  * @brief  Get Speed
  * @param  None
  * @retval Speed
**/
float GetSpeed(void)
{
  return Driver[0].velCtrl.speed;
}

/**
  * @brief  传递输出电压
  * @param  None
  * @retval 得到的值
  */
float GetVelPidOut(void)
{
  return Driver[0].velCtrl.output;
}

/**
  * @brief  max min limit
	* @param  inDat:
	* @retval outDat
  */
float MaxMinLimit(float val, float limit)
{
  if (val > limit)
    val = limit;
  if (val < -limit)
    val = -limit;

  return val;
}

/**
  * @brief  电机使能
  * @param  n:哪个电机	(0-1)
	* @retval None
  */
void MotorOn(int n)
{
  if (Driver[n].unitMode == POSITION_CONTROL_MODE)
    Driver[n].posCtrl.desiredPos = Driver[n].posCtrl.actualPos;

  if (Driver[n].unitMode == SPEED_CONTROL_MODE)
    Driver[n].velCtrl.desiredVel[CMD] = 0.0f;

  Driver[n].velCtrl.iOut = 0.0f;

  Driver[n].status = ENABLE;
}

/**
  * @brief  电机失能
  * @param  n:哪个电机  (0-7)
	* @retval None
  */
void MotorOff(int n)
{
  Driver[n].status = DISABLE;
}

/**
  * @brief  速度环测试
	* @param  vel：测试用速度大小
	* @param  tim：速度切换时间
	* @retval None
  */
void VelCtrlTest(float vel, int tim)
{
  Driver[0].velCtrl.desiredVel[CMD] = vel;
  HAL_Delay(tim);
  Driver[0].velCtrl.desiredVel[CMD] = -vel;
  HAL_Delay(tim);
}

/************************ (C) COPYRIGHT 2020 HCRT ********************/
