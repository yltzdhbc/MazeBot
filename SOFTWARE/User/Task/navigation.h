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

//Բ����
#ifndef PI
#define PI 3.1415926f
#endif

#define constrain_to_0_2pi(amt) ((amt > 2 * PI) ? (amt -= 2 * PI) : ((amt < 0) ? (amt += 2 * PI) : (amt = amt))) //ת����0-2PI
#define constrain_to_npi_pi(amt) ((amt > PI) ? (amt -= 2 * PI) : ((amt < -PI) ? (amt += 2 * PI) : (amt = amt)))	 //ת����-pi-pi

#define constrain(amt, low, high) ((amt > high) ? (amt = high) : ((amt < low) ? (amt = low) : (amt = amt))) //�޷�

#define low_constrain(amt, low) ((amt) < (low) ? (amt = low) : (amt = amt))		//���� �޷�
#define high_constrain(amt, high) ((amt) > (high) ? (amt = high) : (amt = amt)) //���� �޷�

//���β岹���ʱ��
#define TRAIL_REVISE_TIME 0.020f // ��λ s �岹����ִ��һ�����ڵ�ʱ��

#define MAX_PLAN_VEL 1.6f //���ֱ���ٶ�
#define MAX_PLAN_W 0.9f   //�����ٶ�(��ת�ٶ�)

#define ACC 0.1f		 //ֱ�߶μ��ٶ�m/s^2 1.5
#define W_ACC 0.1f		 //�Դ��νǼ��ٶ� 5.0

#define EACC 2 * ACC	 //ֱ����ת��ƫ���ٶ�m/s^2
#define EW_ACC 2 * W_ACC //ֱ����ת��ƫ�Ǽ��ٶ�

#define Round_Error 0.001f	//Բ����� �����С����С�ᵼ�³������
#define Round_Error_T 0.01f //Բ����� �����С����С�ᵼ�³������

/***************** ���߹�����ʹ�õľ��ȷ�Χ *****************/
#define X_Y_Error_0 0.02f			//�������ֵ ��λ(M)
#define Theta_Error_0 3  * PI / 180 //�������ֵ ��λ(RAD)

/***************** �յ�ʹ�õľ��ȷ�Χ *****************/
#define X_Y_Error_1 0.01f		   //�������ֵ ��λ(M)
#define Theta_Error_1 1 * PI / 180 //�������ֵ ��λ(RAD)

float A_x = 20.0f;
float B_x = 20.0f;
float C_x = 8.0f;
////��ƫ�㷨����
//#define A_x 15 / 1.5
//#define B_x 5
//#define C_x 10
//X�����y����Ĳ���Ӧ������һ��
#define A_y A_x
#define B_y B_x
#define C_y C_x
//��ת��ƫ����
#define A_theta 5.0f
#define B_theta 5.0f
#define C_theta 9.0f

//��ʼ��λ��� ��λΪmm
#define position_x_err 0.0f
#define position_y_err 0.0f
#define position_thate_err 0 //��λ ��

typedef enum
{
	NAV_STATE_INIT = 0,
	NAV_STATE_RUN = 1,
	NAV_STATE_ERROR = 2,
} E_NAV_STATE;

#define NONE  0
typedef enum
{
	//NONE = 0, //ֱ������ת����
	CCW = 1,  //��ʱ��
	CW = 2,	  //˳ʱ��
} rotate_direction_e;

typedef enum
{
	TRUE = 1,
	FALSE = 0,
} logic_e;

// ·����Ϣ
typedef struct
{
	/* ��С�����������Ƕ� һ���ǳ��������̬�� start_theta ��ʾ��ͷ�ĳ���
    ** ��һ���� start_vel_angle ��ʾ��ǰ�ٶ������ĽǶ� ��������ʻ����*/

	/********************** �������� **********************/
	float start_x; //��� x
	float start_y; //��� y

	float end_x; //�յ� x
	float end_y; //�յ� y

	float arc_center_x; //Բ��Բ��x����
	float arc_center_y; //Բ��Բ��y����

	float arc_radius;				  //Բ���켣�뾶
	rotate_direction_e arc_direction; //Բ����ת����

	float start_theta; //��� theta С��λ�˵ĽǶ� ���ٶȽǶ�
	float end_theta;   //�յ� theta

	float v_start;	//������ٶ�
	float v_target; //���й�����������ٶ�
	float v_end;	//�յ����ٶ�

	float w_start;	//���С��������ٶ�
	float w_target; //���й��������С��������ٶ�
	float w_end;	//�յ�С��������ٶ�
	/********************** END �������� **********************/

	float w_vTheta_start;  //����ٶȷ�����ٶ�
	float w_vTheta_target; //���й���������ٶȷ�����ٶ�
	float w_vTheta_end;	   //�յ��ٶȷ�����ٶ�

	//��ʵ��Բ���켣�뾶 ��Բ���岹��ʵʱ�ļ���
	//��ΪԲ���岹�İ뾶�Ǳ仯�� �ñ仯ֵ����̶�ֵ��С���
	float arc_radius_actual;

	float path_theta; //��ʼ�����ֹ������ L �����x��ĽǶ� ·���Ƕ�

	float arc_quadrant; //Բ���켣������ 1��2��3��4

	uint8_t path_id; //ÿ��·����Ψһ���

	rotate_direction_e rotate_direction; //ת�Ƿ����־λ

} path_t;

//�岹����
typedef struct
{
	float x;	  //�岹��x (��λ:m)
	float y;	  //�岹��y (��λ:m)
	float theta;  //�岹��theta ��ͷ����
	float vTheta; //�岹��vel_theta ���ٶȷ���

	float w_z;		//С������ ���ٶ�
	float vel;		//С�����ٶ�
	float w_vTheta; //С������� ��ת�ٶ�

	float total_distance;	//�ܹ��Ĳ岹���� (��λ:m)
	float brake_distance;	//ɲ������ (��λ:m)
	float past_distance;	//�Բ岹���ľ��� (��λ:m)
	float residue_distance; //δ�岹�ľ��� (��λ:m)

	float total_theta;	 //��С������ת��(rad)
	float brake_theta;	 //С������ɲ��ת��(rad)
	float past_theta;	 //�Ѳ岹����С������Ƕ�(rad)
	float residue_theta; //δ�岹��С������Ƕ�(rad)

	float total_vTheta;	  //�����ٶȷ���ת��(rad)
	float brake_vTheta;	  //���ٶȷ���ɲ��ת��(rad)
	float past_vTheta;	  //�Ѳ岹�������ٶȷ���Ƕ�(rad)
	float residue_vTheta; //δ�岹�����ٶȷ���Ƕ�(rad)

	float total_arcAngle;	//��Բ���Ƕ�(rad)
	float brake_arcAngle;	//ɲ��Բ���Ƕ�(rad)
	float past_arcAngle;	//�Ѳ岹����Բ���Ƕ�(rad)
	float residue_arcAngle; //δ�岹��Բ���Ƕ�(rad)

	logic_e line_interpolation_is_done;	 //ֱ�߲岹��ɱ�־λ
	logic_e arc_interpolation_is_done;	 //С���ǶȲ岹��ɱ�־λ
	logic_e theta_interpolation_is_done; //С���ǶȲ岹��ɱ�־λ
	logic_e both_interpolation_is_done;	 //�岹��ɱ�־λ
} interpolation_t;

//��λϵͳ(positon positing system)����������
typedef struct
{
	float pos_x; //λ�� x������
	float pos_y; //λ�� y������
	float theta; //λ�� z��Ƕ�
	float v_x;	 //�ٶ� x�᷽��ķ���
	float v_y;	 //�ٶ� y�᷽��ķ���
	float w_z;	 //�ٶ� z�᷽��ķ��� �����˵Ľ��ٶ�

} pps_feedback_t;

typedef struct
{
	float pos_x; //λ�� x������
	float pos_y; //λ�� y������
	float theta; //λ�� z��Ƕ�
} golbal_position_t;

extern golbal_position_t global_position;
extern pps_feedback_t pps_feedback;
extern uint8_t path_index;
extern logic_e track_is_done; // ȫ�� ������ɱ�־λ
// extern interpolation_t interpolation_data[TEST_PATH_NUMBER];
// extern path_t test_path[TEST_PATH_NUMBER];

/**
  * @brief          ��ʼ���켣 ����һЩ����
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_init(uint8_t total_path_num, uint8_t current_path_num, path_t *path, interpolation_t *inp_data);

/**
  * @brief          �켣�岹
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_interpolation(path_t *path, interpolation_t *inp_data);

/**
  * @brief          �켣���ٿ����� ʵʱ�������������� ���ƻ����˵�������λ��
  * @param[in]      chassis_move_t *chassisMove ����λ��
  * @param[in]      pps_feedback_t ppsFeedback ʵ��λ��(��λϵͳ����)
  * @param[out]     vx_set: x�����ٶ�ָ��, vy_set: y�����ٶ�ָ��, wz_set: z�����ٶ�ָ��
  * @retval         none
  */
void track_controller(path_t *path, interpolation_t *inp_data, pps_feedback_t *ppsFeedback, fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

///**
//  * @brief          �������� ������������
//  * @param[in]      none 
//  * @param[in]      none
//  * @retval         none
//  */
//void navigation_task();

#endif
