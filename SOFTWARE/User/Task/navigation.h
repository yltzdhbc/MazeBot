#ifndef NAVIGATION_H
#define NAVIGATION_H

//#include "struct_typedef.h"
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

//圆周率
#ifndef PI
#define PI 3.1415926f
#endif

#define constrain_to_0_2pi(amt) ((amt > 2 * PI) ? (amt -= 2 * PI) : ((amt < 0) ? (amt += 2 * PI) : (amt = amt))) //转换到0-2PI
#define constrain_to_npi_pi(amt) ((amt > PI) ? (amt -= 2 * PI) : ((amt < -PI) ? (amt += 2 * PI) : (amt = amt)))	 //转换到-pi-pi

#define constrain(amt, low, high) ((amt > high) ? (amt = high) : ((amt < low) ? (amt = low) : (amt = amt))) //限幅

#define low_constrain(amt, low) ((amt) < (low) ? (amt = low) : (amt = amt))		//下限 限幅
#define high_constrain(amt, high) ((amt) > (high) ? (amt = high) : (amt = amt)) //上限 限幅

//两次插补间隔时间
#define TRAIL_REVISE_TIME 0.020f // 单位 s 插补函数执行一个周期的时间

#define MAX_PLAN_VEL 1.6f //最大直线速度
#define MAX_PLAN_W 0.9f   //最大角速度(旋转速度)

#define ACC 0.1f		 //直线段加速度m/s^2 1.5
#define W_ACC 0.1f		 //自传段角加速度 5.0

#define EACC 2 * ACC	 //直线自转纠偏加速度m/s^2
#define EW_ACC 2 * W_ACC //直线自转纠偏角加速度

#define Round_Error 0.001f	//圆整误差 请勿改小，过小会导致程序出错
#define Round_Error_T 0.01f //圆整误差 请勿改小，过小会导致程序出错

/***************** 行走过程中使用的精度范围 *****************/
#define X_Y_Error_0 0.02f			//允许误差值 单位(M)
#define Theta_Error_0 3  * PI / 180 //允许误差值 单位(RAD)

/***************** 终点使用的精度范围 *****************/
#define X_Y_Error_1 0.01f		   //允许误差值 单位(M)
#define Theta_Error_1 1 * PI / 180 //允许误差值 单位(RAD)

float A_x = 20.0f;
float B_x = 20.0f;
float C_x = 8.0f;
////纠偏算法参数
//#define A_x 15 / 1.5
//#define B_x 5
//#define C_x 10
//X方向和y方向的参数应当保持一致
#define A_y A_x
#define B_y B_x
#define C_y C_x
//自转纠偏参数
#define A_theta 5.0f
#define B_theta 5.0f
#define C_theta 9.0f

//初始定位误差 单位为mm
#define position_x_err 0.0f
#define position_y_err 0.0f
#define position_thate_err 0 //单位 度

typedef enum
{
	NAV_STATE_INIT = 0,
	NAV_STATE_RUN = 1,
	NAV_STATE_ERROR = 2,
} E_NAV_STATE;

#define NONE  0
typedef enum
{
	//NONE = 0, //直线无旋转方向
	CCW = 1,  //逆时针
	CW = 2,	  //顺时针
} rotate_direction_e;

typedef enum
{
	TRUE = 1,
	FALSE = 0,
} logic_e;

// 路径信息
typedef struct
{
	/* 在小车中有两个角度 一个是车自身的姿态角 start_theta 表示车头的朝向
    ** 另一个是 start_vel_angle 表示当前速度向量的角度 即车的行驶方向*/

	/********************** 输入数据 **********************/
	float start_x; //起点 x
	float start_y; //起点 y

	float end_x; //终点 x
	float end_y; //终点 y

	float arc_center_x; //圆弧圆心x坐标
	float arc_center_y; //圆弧圆心y坐标

	float arc_radius;				  //圆弧轨迹半径
	rotate_direction_e arc_direction; //圆弧旋转方向

	float start_theta; //起点 theta 小车位姿的角度 非速度角度
	float end_theta;   //终点 theta

	float v_start;	//起点线速度
	float v_target; //运行过程中最大线速度
	float v_end;	//终点线速度

	float w_start;	//起点小车方向角速度
	float w_target; //运行过程中最大小车方向角速度
	float w_end;	//终点小车方向角速度
	/********************** END 输入数据 **********************/

	float w_vTheta_start;  //起点速度方向角速度
	float w_vTheta_target; //运行过程中最大速度方向角速度
	float w_vTheta_end;	   //终点速度方向角速度

	//真实的圆弧轨迹半径 在圆弧插补中实时的计算
	//因为圆弧插补的半径是变化的 用变化值代替固定值减小误差
	float arc_radius_actual;

	float path_theta; //起始点和终止点连线 L 相对于x轴的角度 路径角度

	float arc_quadrant; //圆弧轨迹的象限 1、2、3、4

	uint8_t path_id; //每条路径的唯一序号

	rotate_direction_e rotate_direction; //转角方向标志位

} path_t;

//插补数据
typedef struct
{
	float x;	  //插补点x (单位:m)
	float y;	  //插补点y (单位:m)
	float theta;  //插补点theta 车头朝向
	float vTheta; //插补点vel_theta 线速度方向

	float w_z;		//小车自旋 角速度
	float vel;		//小车线速度
	float w_vTheta; //小车方向角 旋转速度

	float total_distance;	//总共的插补距离 (单位:m)
	float brake_distance;	//刹车距离 (单位:m)
	float past_distance;	//以插补过的距离 (单位:m)
	float residue_distance; //未插补的距离 (单位:m)

	float total_theta;	 //总小车方向转角(rad)
	float brake_theta;	 //小车方向刹车转角(rad)
	float past_theta;	 //已插补过的小车方向角度(rad)
	float residue_theta; //未插补的小车方向角度(rad)

	float total_vTheta;	  //总线速度方向转角(rad)
	float brake_vTheta;	  //线速度方向刹车转角(rad)
	float past_vTheta;	  //已插补过的线速度方向角度(rad)
	float residue_vTheta; //未插补的线速度方向角度(rad)

	float total_arcAngle;	//总圆弧角度(rad)
	float brake_arcAngle;	//刹车圆弧角度(rad)
	float past_arcAngle;	//已插补过的圆弧角度(rad)
	float residue_arcAngle; //未插补的圆弧角度(rad)

	logic_e line_interpolation_is_done;	 //直线插补完成标志位
	logic_e arc_interpolation_is_done;	 //小车角度插补完成标志位
	logic_e theta_interpolation_is_done; //小车角度插补完成标志位
	logic_e both_interpolation_is_done;	 //插补完成标志位
} interpolation_t;

//定位系统(positon positing system)反馈的数据
typedef struct
{
	float pos_x; //位置 x轴坐标
	float pos_y; //位置 y轴坐标
	float theta; //位置 z轴角度
	float v_x;	 //速度 x轴方向的分量
	float v_y;	 //速度 y轴方向的分量
	float w_z;	 //速度 z轴方向的分量 机器人的角速度

} pps_feedback_t;

typedef struct
{
	float pos_x; //位置 x轴坐标
	float pos_y; //位置 y轴坐标
	float theta; //位置 z轴角度
} golbal_position_t;

extern golbal_position_t global_position;
extern pps_feedback_t pps_feedback;
extern uint8_t path_index;
extern logic_e track_is_done; // 全局 跟踪完成标志位
// extern interpolation_t interpolation_data[TEST_PATH_NUMBER];
// extern path_t test_path[TEST_PATH_NUMBER];

/**
  * @brief          初始化轨迹 计算一些变量
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_init(uint8_t total_path_num, uint8_t current_path_num, path_t *path, interpolation_t *inp_data);

/**
  * @brief          轨迹插补
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_interpolation(path_t *path, interpolation_t *inp_data);

/**
  * @brief          轨迹跟踪控制器 实时输出控制量给电机 控制机器人到达理论位置
  * @param[in]      chassis_move_t *chassisMove 理论位置
  * @param[in]      pps_feedback_t ppsFeedback 实际位置(定位系统反馈)
  * @param[out]     vx_set: x方向速度指针, vy_set: y方向速度指针, wz_set: z方向速度指针
  * @retval         none
  */
void track_controller(path_t *path, interpolation_t *inp_data, pps_feedback_t *ppsFeedback, fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

///**
//  * @brief          导航任务 输出电机控制量
//  * @param[in]      none 
//  * @param[in]      none
//  * @retval         none
//  */
//void navigation_task();

#endif
