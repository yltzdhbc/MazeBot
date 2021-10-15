///**
//  ****************************(C) COPYRIGHT 2020 HCRT****************************
//  * @file       ins_task.c/h
//  * @brief      陀螺仪和磁力计控制任务
//  * @note				惯性导航系统（Inertial Navigation System）
//  * @history
//  *  Version    Date            Author          Modification
//  *  V1.0.0     2020.2.4     		 YLT              	1.0
//  *
//  @verbatim
//  ==============================================================================
//	* 更新改函数为最新HAL库的写法，可以使用cubemx配置，为以后做兼容
//	* 完成基本的四元数解算的基础上，增加了PWM控制加热电阻，控制加速度计恒温，减少温漂
//  ==============================================================================
//  @endverbatim
//  ****************************(C) COPYRIGHT 2020 HCRT****************************
//  */
//#include "ins_task.h"
//#include "bsp_imu_pwm.h"
//#include "ist8310_reg.h"
//#include "mpu6500_reg.h"
//#include "cmsis_os.h"
//#include "arm_math.h"
//#include "user_lib.h"
//#include "spi.h"
//#include "pid.h"
//#include <math.h>

//#define BOARD_DOWN (1)
//#define IST8310
//#define MPU_HSPI hspi5
//#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
//#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
//#define Kp 2.0f   //比例增益控制加速度计/磁强计的收敛速度
//#define Ki 0.01f  //积分增益决定陀螺偏差的收敛速度           

//volatile float        q0 = 1.0f;
//volatile float        q1 = 0.0f;
//volatile float        q2 = 0.0f;
//volatile float        q3 = 0.0f;
//volatile float        exInt, eyInt, ezInt;                   /* error integral */
//static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;
//volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
//static uint8_t        tx, rx;
//static uint8_t        tx_buff[14] = { 0xff };
//uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
//uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
//mpu_data_t            mpu_data;
//imu_t                 imu= {0};

//#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

//static uint8_t first_temperate;
//static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//static pid_type_def imu_temp_pid;

///**
//  * @brief          惯性导航系统（Inertial Navigation System）解算任务
//	*									初始化 mpu6500, ist8310, 计算欧拉角
//  * @param[in]      pvParameters: NULL
//  * @retval         none
//  */
//void InsTask(void const *pvParameters)
//{
//	//在初始化IMU的时候
//  mpu_device_init();
//	//初始化四元数，用于姿态解算
//  init_quaternion();

//  for(;;)
//    {
//			//获得imu原始数据
//      mpu_get_data();
//			//更新四元数和imu姿态
//      imu_ahrs_update();
//			//解算imu的pitch,roll,yaw角度
//      imu_attitude_update();
//			//加热电阻温度控制
//			imu_temp_control(imu.temp);
//      //系统延时
//      osDelay(INS_CONTROL_TIME_MS);
//    }
//}


///**
//  * @brief          控制IMU加热电阻温度
//  * @param[in]      temp:IMU的温度
//  * @retval         none
//  */
//static void imu_temp_control(fp32 temp)
//{
//    uint16_t tempPWM;
//    static uint8_t temp_constant_time = 0;
//    if (first_temperate)
//    {
//        PID_calc(&imu_temp_pid, temp, 50);
//        if (imu_temp_pid.out < 0.0f)
//        {
//            imu_temp_pid.out = 0.0f;
//        }
//        tempPWM = (uint16_t)imu_temp_pid.out;
//        IMU_temp_PWM(tempPWM);
//    }
//    else
//    {
//        //在没有达到设置的温度，一直最大功率加热
//        if (temp > 50)
//        {
//            temp_constant_time++;
//            if (temp_constant_time > 200)
//            {
//                //达到设置温度，将积分项设置为一半最大功率，加速收敛
//                first_temperate = 1;
//                imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
//            }
//        }

//        IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
//    }
//}


///**
//  * @brief  快速开方, 计算 1/Sqrt(x)
//  * @param  x: the number need to be calculated
//  * @retval 1/Sqrt(x)
//  * @usage  call in imu_ahrs_update() function
//  */
//float inv_sqrt(float x)
//{
//  float halfx = 0.5f * x;
//  float y     = x;
//  long  i     = *(long*)&y;

//  i = 0x5f3759df - (i >> 1);
//  y = *(float*)&i;
//  y = y * (1.5f - (halfx * y * y));

//  return y;
//}

///**
//  * @brief  write a byte of data to specified register
//  * @param  reg:  the address of register to be written
//  *         data: data to be written
//  * @retval
//  * @usage  call in ist_reg_write_by_mpu(),
//  *                 ist_reg_read_by_mpu(),
//  *                 mpu_master_i2c_auto_read_config(),
//  *                 ist8310_init(),
//  *                 mpu_set_gyro_fsr(),
//  *                 mpu_set_accel_fsr(),
//  *                 mpu_device_init() function
//  */
//uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
//{
//  MPU_NSS_LOW;
//  tx = reg & 0x7F;
//  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//  tx = data;
//  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//  MPU_NSS_HIGH;
//  return 0;
//}

///**
//  * @brief  read a byte of data from specified register
//  * @param  reg: the address of register to be read
//  * @retval
//  * @usage  call in ist_reg_read_by_mpu(),
//  *                 mpu_device_init() function
//  */
//uint8_t mpu_read_byte(uint8_t const reg)
//{
//  MPU_NSS_LOW;
//  tx = reg | 0x80;
//  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//  MPU_NSS_HIGH;
//  return rx;
//}

///**
//  * @brief  read bytes of data from specified register
//  * @param  reg: address from where data is to be written
//  * @retval
//  * @usage  call in ist8310_get_data(),
//  *                 mpu_get_data(),
//  *                 mpu_offset_call() function
//  */
//uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
//{
//  MPU_NSS_LOW;
//  tx         = regAddr | 0x80;
//  tx_buff[0] = tx;
//  HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
//  HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
//  MPU_NSS_HIGH;
//  return 0;
//}

///**
//	* @brief  write IST8310 register through MPU6500's I2C master
//  * @param  addr: the address to be written of IST8310's register
//  *         data: data to be written
//	* @retval
//  * @usage  call in ist8310_init() function
//	*/
//static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
//{
//  /* turn off slave 1 at first */
//  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
//  MPU_DELAY(2);
//  mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
//  MPU_DELAY(2);
//  mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
//  MPU_DELAY(2);
//  /* turn on slave 1 with one byte transmitting */
//  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
//  /* wait longer to ensure the data is transmitted from slave 1 */
//  MPU_DELAY(10);
//}

///**
//	* @brief  write IST8310 register through MPU6500's I2C Master
//	* @param  addr: the address to be read of IST8310's register
//	* @retval
//  * @usage  call in ist8310_init() function
//	*/
//static uint8_t ist_reg_read_by_mpu(uint8_t addr)
//{
//  uint8_t retval;
//  mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
//  MPU_DELAY(10);
//  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
//  MPU_DELAY(10);
//  retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
//  /* turn off slave4 after read */
//  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
//  MPU_DELAY(10);
//  return retval;
//}

///**
//	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
//* @param    device_address: slave device address, Address[6:0]
//	* @retval   void
//	* @note
//	*/
//static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
//{
//  /*
//   * configure the device address of the IST8310
//   * use slave1, auto transmit single measure mode
//   */
//  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
//  MPU_DELAY(2);
//  mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
//  MPU_DELAY(2);
//  mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
//  MPU_DELAY(2);

//  /* use slave0,auto read data */
//  mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
//  MPU_DELAY(2);
//  mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
//  MPU_DELAY(2);

//  /* every eight mpu6500 internal samples one i2c master read */
//  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
//  MPU_DELAY(2);
//  /* enable slave 0 and 1 access delay */
//  mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
//  MPU_DELAY(2);
//  /* enable slave 1 auto transmit */
//  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
//  /* Wait 6ms (minimum waiting time for 16 times internal average setup) */
//  MPU_DELAY(6);
//  /* enable slave 0 with data_num bytes reading */
//  mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
//  MPU_DELAY(2);
//}

///**
//	* @brief  Initializes the IST8310 device
//	* @param
//	* @retval
//  * @usage  call in mpu_device_init() function
//	*/
//uint8_t ist8310_init()
//{
//  /* enable iic master mode */
//  mpu_write_byte(MPU6500_USER_CTRL, 0x30);
//  MPU_DELAY(10);
//  /* enable iic 400khz */
//  mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d);
//  MPU_DELAY(10);

//  /* turn on slave 1 for ist write and slave 4 to ist read */
//  mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);
//  MPU_DELAY(10);
//  mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
//  MPU_DELAY(10);

//  /* IST8310_R_CONFB 0x01 = device rst */
//  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
//  MPU_DELAY(10);
//  if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
//    return 1;

//  /* soft reset */
//  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
//  MPU_DELAY(10);

//  /* config as ready mode to access register */
//  ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00);
//  if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
//    return 2;
//  MPU_DELAY(10);

//  /* normal state, no int */
//  ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
//  if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
//    return 3;
//  MPU_DELAY(10);

//  /* config low noise mode, x,y,z axis 16 time 1 avg */
//  ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
//  if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
//    return 4;
//  MPU_DELAY(10);

//  /* Set/Reset pulse duration setup,normal mode */
//  ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
//  if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
//    return 5;
//  MPU_DELAY(10);

//  /* turn off slave1 & slave 4 */
//  mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
//  MPU_DELAY(10);
//  mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
//  MPU_DELAY(10);

//  /* configure and turn on slave 0 */
//  mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
//  MPU_DELAY(100);
//  return 0;
//}

///**
//	* @brief  get the data of IST8310
//  * @param  buff: the buffer to save the data of IST8310
//	* @retval
//  * @usage  call in mpu_get_data() function
//	*/
//void ist8310_get_data(uint8_t* buff)
//{
//  mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
//}


///**
//	* @brief  get the data of imu
//  * @param
//	* @retval
//  * @usage  call in main() function
//	*/
//void mpu_get_data()
//{
//  mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

//  mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
//  mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
//  mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
//  mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

//  mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
//  mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
//  mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

//  ist8310_get_data(ist_buff);
//  memcpy(&mpu_data.mx, ist_buff, 6);

//  memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));

//  imu.temp = 21 + mpu_data.temp / 333.87f;
//  /* 2000dps -> rad/s */
//  imu.wx   = mpu_data.gx / 16.384f / 57.3f;
//  imu.wy   = mpu_data.gy / 16.384f / 57.3f;
//  imu.wz   = mpu_data.gz / 16.384f / 57.3f;
//}


///**
//	* @brief  set imu 6500 gyroscope measure range
//  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
//	* @retval
//  * @usage  call in mpu_device_init() function
//	*/
//uint8_t mpu_set_gyro_fsr(uint8_t fsr)
//{
//  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
//}


///**
//	* @brief  set imu 6050/6500 accelerate measure range
//  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
//	* @retval
//  * @usage  call in mpu_device_init() function
//	*/
//uint8_t mpu_set_accel_fsr(uint8_t fsr)
//{
//  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3);
//}

//uint8_t id;

///**
//	* @brief  initialize imu mpu6500 and magnet meter ist3810
//  * @param
//	* @retval
//  * @usage  call in main() function
//	*/
//uint8_t mpu_device_init(void)
//{
//  MPU_DELAY(100);

//  id                               = mpu_read_byte(MPU6500_WHO_AM_I);
//  uint8_t i                        = 0;
//  uint8_t MPU6500_Init_Data[10][2] = {
//		{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */
//    { MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */
//    { MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */
//    { MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */
//    { MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */
//    { MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */
//    { MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */
//    { MPU6500_USER_CTRL, 0x20 },
//  };    /* Enable AUX */
//  for (i = 0; i < 10; i++)
//    {
//      mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
//      MPU_DELAY(1);
//    }
//		
//	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);

//  mpu_set_gyro_fsr(3);
//  mpu_set_accel_fsr(2);

//  ist8310_init();
//  mpu_offset_call();
//  return 0;
//}

///**
//	* @brief  get the offset data of MPU6500
//  * @param
//	* @retval
//  * @usage  call in main() function
//	*/
//void mpu_offset_call(void)
//{
//  int i;
//  for (i=0; i<300; i++)
//    {
//      mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

//      mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
//      mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
//      mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];

//      mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
//      mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
//      mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

//      MPU_DELAY(5);
//    }
//  mpu_data.ax_offset=mpu_data.ax_offset / 300;
//  mpu_data.ay_offset=mpu_data.ay_offset / 300;
//  mpu_data.az_offset=mpu_data.az_offset / 300;
//  mpu_data.gx_offset=mpu_data.gx_offset / 300;
//  mpu_data.gy_offset=mpu_data.gx_offset / 300;
//  mpu_data.gz_offset=mpu_data.gz_offset / 300;
//}



///**
//	* @brief  Initialize quaternion
//  * @param
//	* @retval
//  * @usage  call in main() function
//	*/
//void init_quaternion(void)
//{
//  int16_t hx, hy;//hz;

//  hx = imu.mx;
//  hy = imu.my;
//  //hz = imu.mz;

//#ifdef BOARD_DOWN
//  if (hx < 0 && hy < 0)
//    {
//      if (fabs(hx / hy) >= 1)
//        {
//          q0 = -0.005;
//          q1 = -0.199;
//          q2 = 0.979;
//          q3 = -0.0089;
//        }
//      else
//        {
//          q0 = -0.008;
//          q1 = -0.555;
//          q2 = 0.83;
//          q3 = -0.002;
//        }

//    }
//  else if (hx < 0 && hy > 0)
//    {
//      if (fabs(hx / hy)>=1)
//        {
//          q0 = 0.005;
//          q1 = -0.199;
//          q2 = -0.978;
//          q3 = 0.012;
//        }
//      else
//        {
//          q0 = 0.005;
//          q1 = -0.553;
//          q2 = -0.83;
//          q3 = -0.0023;
//        }

//    }
//  else if (hx > 0 && hy > 0)
//    {
//      if (fabs(hx / hy) >= 1)
//        {
//          q0 = 0.0012;
//          q1 = -0.978;
//          q2 = -0.199;
//          q3 = -0.005;
//        }
//      else
//        {
//          q0 = 0.0023;
//          q1 = -0.83;
//          q2 = -0.553;
//          q3 = 0.0023;
//        }

//    }
//  else if (hx > 0 && hy < 0)
//    {
//      if (fabs(hx / hy) >= 1)
//        {
//          q0 = 0.0025;
//          q1 = 0.978;
//          q2 = -0.199;
//          q3 = 0.008;
//        }
//      else
//        {
//          q0 = 0.0025;
//          q1 = 0.83;
//          q2 = -0.56;
//          q3 = 0.0045;
//        }
//    }
//#else
//  if (hx < 0 && hy < 0)
//    {
//      if (fabs(hx / hy) >= 1)
//        {
//          q0 = 0.195;
//          q1 = -0.015;
//          q2 = 0.0043;
//          q3 = 0.979;
//        }
//      else
//        {
//          q0 = 0.555;
//          q1 = -0.015;
//          q2 = 0.006;
//          q3 = 0.829;
//        }

//    }
//  else if (hx < 0 && hy > 0)
//    {
//      if(fabs(hx / hy) >= 1)
//        {
//          q0 = -0.193;
//          q1 = -0.009;
//          q2 = -0.006;
//          q3 = 0.979;
//        }
//      else
//        {
//          q0 = -0.552;
//          q1 = -0.0048;
//          q2 = -0.0115;
//          q3 = 0.8313;
//        }

//    }
//  else if (hx > 0 && hy > 0)
//    {
//      if(fabs(hx / hy) >= 1)
//        {
//          q0 = -0.9785;
//          q1 = 0.008;
//          q2 = -0.02;
//          q3 = 0.195;
//        }
//      else
//        {
//          q0 = -0.9828;
//          q1 = 0.002;
//          q2 = -0.0167;
//          q3 = 0.5557;
//        }

//    }
//  else if (hx > 0 && hy < 0)
//    {
//      if(fabs(hx / hy) >= 1)
//        {
//          q0 = -0.979;
//          q1 = 0.0116;
//          q2 = -0.0167;
//          q3 = -0.195;
//        }
//      else
//        {
//          q0 = -0.83;
//          q1 = 0.014;
//          q2 = -0.012;
//          q3 = -0.556;
//        }
//    }
//#endif
//}

///**
//	* @brief  update imu AHRS
//  * @param
//	* @retval
//  * @usage  call in main() function
//	*/
//void imu_ahrs_update(void)
//{
//  float norm;
//  float hx, hy, hz, bx, bz;
//  float vx, vy, vz, wx, wy, wz;
//  float ex, ey, ez, halfT;
//  float tempq0,tempq1,tempq2,tempq3;

//  float q0q0 = q0*q0;
//  float q0q1 = q0*q1;
//  float q0q2 = q0*q2;
//  float q0q3 = q0*q3;
//  float q1q1 = q1*q1;
//  float q1q2 = q1*q2;
//  float q1q3 = q1*q3;
//  float q2q2 = q2*q2;
//  float q2q3 = q2*q3;
//  float q3q3 = q3*q3;

//  gx = imu.wx;
//  gy = imu.wy;
//  gz = imu.wz;
//  ax = imu.ax;
//  ay = imu.ay;
//  az = imu.az;
//  mx = imu.mx;
//  my = imu.my;
//  mz = imu.mz;

//  now_update  = HAL_GetTick(); //ms
//  halfT       = ((float)(now_update - last_update) / 2000.0f);
//  last_update = now_update;

//  /* Fast inverse square-root */
//  norm = inv_sqrt(ax*ax + ay*ay + az*az);
//  ax = ax * norm;
//  ay = ay * norm;
//  az = az * norm;

//#ifdef IST8310
//  norm = inv_sqrt(mx*mx + my*my + mz*mz);
//  mx = mx * norm;
//  my = my * norm;
//  mz = mz * norm;
//#else
//  mx = 0;
//  my = 0;
//  mz = 0;
//#endif
//  /* compute reference direction of flux */
//  hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
//  hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
//  hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);
//  bx = sqrt((hx*hx) + (hy*hy));
//  bz = hz;

//  /* estimated direction of gravity and flux (v and w) */
//  vx = 2.0f*(q1q3 - q0q2);
//  vy = 2.0f*(q0q1 + q2q3);
//  vz = q0q0 - q1q1 - q2q2 + q3q3;
//  wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
//  wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
//  wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);

//  /*
//   * error is sum of cross product between reference direction
//   * of fields and direction measured by sensors
//   */
//  ex = (ay*vz - az*vy) + (my*wz - mz*wy);
//  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
//  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

//  /* PI */
//  if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
//    {
//      exInt = exInt + ex * Ki * halfT;
//      eyInt = eyInt + ey * Ki * halfT;
//      ezInt = ezInt + ez * Ki * halfT;

//      gx = gx + Kp*ex + exInt;
//      gy = gy + Kp*ey + eyInt;
//      gz = gz + Kp*ez + ezInt;
//    }

//  tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
//  tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
//  tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
//  tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;

//  /* normalise quaternion */
//  norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
//  q0 = tempq0 * norm;
//  q1 = tempq1 * norm;
//  q2 = tempq2 * norm;
//  q3 = tempq3 * norm;
//}

///**
//	* @brief  update imu attitude
//  * @param
//	* @retval
//  * @usage  call in main() function
//	*/
//void imu_attitude_update(void)
//{
//  /* yaw    -pi----pi */
//  imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3;
//  /* pitch  -pi/2----pi/2 */
//  imu.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;
//  /* roll   -pi----pi  */
//  imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
//}


