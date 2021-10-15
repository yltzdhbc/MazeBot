/**
  ****************************(C) COPYRIGHT 2020 HCRT****************************
  * @file       navigation.c/h
  * @brief      �������ܰ� 
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V0.1.0     2020.2.12     	 YLT              	0.1
  *  V1.0.0     2020.5.2     	 YLT              	1.0
  @verbatim
  ==============================================================================
   	* update 2020.5.2 :(release)
		1.Webots������԰汾 ���й��ܾ���ʵ�� �������㷨���ڵ�BUG
		2.����ͳһ��һ���������navigation_task״̬������ʵ��·���ĳ�ʼ�����岹�����١�
		�Լ�����״̬�Ĵ���
		3.�岹�͸��ٺϲ���һ�������� ֻ�и�����Ż�岹����һ����
	* update 2020.2.12 :(beta)
		1.�Ż������ṹ ����ʹ�ýṹ������ �������������޸�·�� 
    	2.����Զ�̴��εĽṹ ʹ��ң�ؾͿ��Է���·����Ϣ �޸������е����ݾͿ��Ըı�·��
    	3.ֱ�� Բ�� ��ת �ں�Ϊһ������ ������һЩ֮ǰ����ı��� 
    	4.�ص��޸ĵ���Բ������ Բ����֮ǰ������Բ����Ϊ ����Ӱ뾶Բ�� ������Ҫ�������
    	5.�� �������Ͳ岹�� �ֿ� �岹����200HZ��Ƶ�ʸ������۵� ���۽Ƕ�  ��������500HZ
    	��Ƶ�ʿ��ƻ����˵������۵� �������������۵�֮���ı�һ����־λ���߲岹�� �岹��
    	�ͻῪʼ�岹��һ���� ֱ�����е�·������ ��ĩ�ٶ�Ϊ�� �岹���� ��������Ȼʵʱ��ƫ
    	�������˶���ĩβ��  
    	6.���� ������Ȼ�ǲ岹�� ���һ���� ����������һ���� �������� �ټ�����һ���� ����
    	�ķ�ʽ��ʡ�ڴ� ����������Ƭ�����ڴ沢�������������� ���Կ�����һ��ʼ�����۵�ȫ
		������� ������������ �ٶ�ȡ ���ܻ���߾���
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2020 HCRT****************************
  */
/*
*                   0
*                   y+
*                   |
*                   |
*                   |
*                   |
* +90  ---------------------------> x+  -90
*                   |
*                   |
*                   |
*                   |
*              +180   -180  
*/
#include <math.h>
#include <stdio.h>
#include "movebase.h"
#include "navigation.h"
#include "cmsis_os.h"
#include "main.h"

#define DEBUG

#define TEST_PATH_NUMBER 6 //����·��������

uint8_t path_index = 0; //·������ �����л�·��
static fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

E_NAV_STATE NAV_STATE = NAV_STATE_INIT; //״̬������
logic_e isTrackDone;                    //ȫ�ָ�����ɱ�־λ
pps_feedback_t pps_feedback;            //��λϵͳ����ȫ��λ����Ϣ
golbal_position_t global_position;      //ȫ��λ��

static interpolation_t interpolation_data[TEST_PATH_NUMBER];
/*	�������·����һ��Բ�Ǿ��� ���Ķ�ֱ�ߺ��Ķ�Բ�����
	���룺start_x , start_y  | end_x , end_y  | arc_center_x , arc_center_y | arc_radius , arc_direction
	��ѡ: v_start , v_target , v_end | w_start , w_target , w_end
*/
static path_t test_path[TEST_PATH_NUMBER] = {
    //sx     sy  |  ex    ey    | ax     ay  | ar     ad
    //���x ���y | �յ�x  �յ�y | Բ��x  Բ��y| Բ���뾶 Բ������
//		{0.00f, 0.00f, 0.00f, 0.20f, NONE, NONE, NONE, NONE, 0.00f, 0.00f},
//		{0.00f, 0.20f, -0.10f, 0.30f, -0.1, 0.2, 0.1, CCW, 0.00f, PI / 2},

		 {0.00f, 0.00f, 0.00f, 3.00f, NONE, NONE, NONE, NONE, 0.00f, 0.00f},
    {0.00f, 3.00f, -0.50f, 3.50f, -0.50f, 3.00f, 0.50f, CCW, 0.00f, PI / 2},
    {-0.50f, 3.50f, -1.00f, 3.00f, -0.50f, 3.00f, 0.50f, CCW, PI / 2, PI},
    {-1.00f, 3.00f, -1.50f, 2.50f, -1.50f, 3.00f, 0.50f, CW, PI, PI / 2},
    {-1.50f, 2.50f, -2.00f, 3.00f, -1.50f, 3.00f, 0.50f, CW, PI / 2, 0.00f},
    {-2.00f, 3.00f, -2.00f, 6.00f, NONE, NONE, NONE, NONE, 0.00f, 0.00f},
};

/**
  * @brief          �������� ������������
  * @param[in]      none 
  * @retval         none
  */
void NavigationTask(void const *argument)
{
    for (;;)
    {
        switch (NAV_STATE)
        {
        case NAV_STATE_INIT: //********** ��ʼ��״̬ **********
            for (path_index = 0; path_index < TEST_PATH_NUMBER; path_index++)
            {
                //��ʼ������ ·��ʹ�õ��ǲ�ͬ���ڴ� ��˿���һ����ȫ����ʼ���� ���Ḳ��
                path_init(TEST_PATH_NUMBER, path_index, &test_path[path_index], &interpolation_data[path_index]);
#ifdef DEBUG
                printf("INIT!: path_index %d, residue_distance %f, residue_theta %f, residue_vTheta %f \r\n",
                       path_index, interpolation_data[path_index].residue_distance,
                       interpolation_data[path_index].residue_theta, interpolation_data[path_index].residue_vTheta);
#endif
            }
            path_index = 0; //·����������
            isTrackDone = TRUE;
            if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0)
                NAV_STATE = NAV_STATE_RUN; //�л�������״̬
            break;
        case NAV_STATE_RUN: //********** ����״̬ **********
            //�ж�һ�β岹�Ƿ���� �����Ļ��Ͱ� path_index ��һ������һ�β岹
            (interpolation_data[path_index].both_interpolation_is_done == TRUE) ? (path_index++) : (path_index = path_index);
            high_constrain(path_index, TEST_PATH_NUMBER - 1); //���� �޷�
            //�岹 ����һ�� �岹��һ����
            path_interpolation(&test_path[path_index], &interpolation_data[path_index]);
            /* ������������ */
            pps_feedback.pos_y = Odom.Pos.y;
            pps_feedback.pos_x = Odom.Pos.x;
            pps_feedback.theta = Odom.Pos.theta;
            /* �켣���ٿ����� */
            track_controller(&test_path[path_index], &interpolation_data[path_index], &pps_feedback, &vx_set, &vy_set, &wz_set);
            /* vx-m/s  vy-m/s  wz-rad/s */
            Output_Wheel(vx_set, vy_set, wz_set); //����ٶȵ�����
#ifdef DEBUG
            printf("GLOBAL!: x %f, y %f, theta %f \r\n", global_position.pos_x, global_position.pos_y, global_position.theta);
            printf("INTERP!: x %f, y %f, theta %f, residue_distance %f, residue_theta%f, residue_vTheta%f \r\n",
                   interpolation_data[path_index].x, interpolation_data[path_index].y, interpolation_data[path_index].theta,
                   interpolation_data[path_index].residue_distance, interpolation_data[path_index].residue_theta, interpolation_data[path_index].residue_vTheta);
            printf("PATH!: path_index %d, both %d, line %d, arc %d, theta %d arc_actuall %f\r\n", path_index, interpolation_data[path_index].both_interpolation_is_done,
                   interpolation_data[path_index].line_interpolation_is_done, interpolation_data[path_index].arc_interpolation_is_done,
                   interpolation_data[path_index].theta_interpolation_is_done, test_path[path_index].arc_radius_actual);
            printf("TRACK!: vx_set %f, vy_set %f, wz_set %f\r\n", vx_set, vy_set, wz_set);
#endif
            break;
        case NAV_STATE_ERROR: //********** ����״̬ **********
            /* code */
            break;
        default:
            break;
        }
        osDelay(20);
    }
}

/**
  * @brief          ��ʼ���켣 ����һЩ����
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_init(uint8_t total_path_num, uint8_t current_path_num, path_t *path, interpolation_t *inp_data)
{
    static float last_path_v_end = 0.0f;

    if (fabs(path->arc_radius) < Round_Error) // ****ֱ�߲岹������ʼ��****
    {
        //����켣�Ƕ� ��ʼ�����ֹ������ L �����x��ĽǶ� ·���Ƕ�
        path->path_theta = atan2(path->end_y - path->start_y, path->end_x - path->start_x);
        //��ʼ���岹���� ֱ�߲岹���ݳ�ʼ��
        inp_data->total_distance = sqrt(pow((path->end_x - path->start_x), 2) + pow((path->end_y - path->start_y), 2)); //ֱ�߲岹 ֱ�ߵ��ܳ�
        inp_data->brake_distance = 0;
        inp_data->past_distance = 0;
        inp_data->residue_distance = inp_data->total_distance; //ʣ�����
    }
    else // ****Բ���岹������ʼ��****
    {
        //ʹ�����Ҷ����������յ㻡��������ת��
        inp_data->total_arcAngle = acos(1 - (pow(path->start_x - path->end_x, 2) + pow(path->start_y - path->end_y, 2)) / (2 * pow(path->arc_radius, 2)));
        inp_data->brake_arcAngle = 0;
        inp_data->past_arcAngle = 0;
        inp_data->residue_arcAngle = inp_data->total_arcAngle;
        //��ʼ���岹���� Բ���岹���ݳ�ʼ��
        inp_data->total_distance = (2 * PI * path->arc_radius) * (inp_data->total_arcAngle / (2 * PI)); //Բ���ܳ�
        inp_data->brake_distance = 0;
        inp_data->past_distance = 0;
        inp_data->residue_distance = inp_data->total_distance; //ʣ�����
        path->arc_radius_actual = path->arc_radius;
        // path->arc_radius_actual = path->arc_radius * (1 + 0.5 * pow(inp_data->past_arcAngle, 2) -
        //                                               pow(inp_data->past_arcAngle, 3) / inp_data->total_arcAngle +
        //                                               0.5 * pow(inp_data->past_arcAngle, 4) / pow(inp_data->total_arcAngle, 2));
        // printf("arc_radius %f,arc_center_x %f,arc_center_y %f \r\n", path->arc_radius, path->arc_center_x, path->arc_center_y);
        // printf("ARC! total_distance %f \r\n", inp_data->total_distance);
    }

    //��ʼ���岹���� ��ת�岹���ݳ�ʼ��
    inp_data->total_theta = fabs(path->end_theta - path->start_theta); //��ת�ǶȲ岹 С������� ��ת��
    inp_data->brake_theta = 0;
    inp_data->past_theta = 0;
    inp_data->residue_theta = (inp_data->total_theta); //ʣ�໡���þ���ֵ��ʾ

    //�岹��־λ ��λ
    inp_data->line_interpolation_is_done = FALSE;  //ֱ�߲岹��ɱ�־λ
    inp_data->theta_interpolation_is_done = FALSE; //����ת�ǲ岹��ɱ�־λ
    inp_data->arc_interpolation_is_done = FALSE;   //Բ���岹��ɱ�־λ
    inp_data->both_interpolation_is_done = FALSE;  //�ܲ岹��ɱ�־λ

    //�ж���������
    (path->start_theta <= path->end_theta) ? (path->rotate_direction = CCW) : (path->rotate_direction = CW);

    path->path_id = current_path_num; //��ÿһ��·������Ψһ��ID

    inp_data->x = path->start_x; //����һ���岹������궨λ��ֵΪ�������
    inp_data->y = path->start_y; //����һ���岹������궨λ��ֵΪ�������

    if (current_path_num == 0) //��һ��·�� ����ٶ�Ϊ0
    {
        inp_data->vel = 0.0f;
        path->v_start = 0.0f;
        path->v_end = MAX_PLAN_VEL;
    }
    else if (current_path_num == total_path_num - 1) //���һ��·�� �յ��ٶ�Ϊ0
    {
        inp_data->vel = last_path_v_end;
        path->v_start = last_path_v_end;
        path->v_end = 0.0f;
    }
    else //�м�·��
    {
        inp_data->vel = last_path_v_end;
        path->v_start = last_path_v_end;
        path->v_end = MAX_PLAN_VEL;
    }
    //printf("INIT!: path_id %d, v_start %f, v_end %f \r\n", path->path_id, path->v_start, path->v_end);

    last_path_v_end = path->v_end; //��¼��һ��·�����յ��ٶ�

    path->v_target = MAX_PLAN_VEL; //�趨����ٶ� ��Ŀ���ٶ�
    path->w_target = MAX_PLAN_W;   //�趨�����ٶ� ��Ŀ����ٶ�

    inp_data->theta = path->start_theta;

    //�ж���ʼ�㡢��ֹ����ٶ��Ƿ񳬹�����ٶ� �����򱨴�
    if ((path->v_start > path->v_target) || (path->w_start > path->w_target) ||
        (path->v_end > path->v_target) || (path->w_end > path->w_target))
    {

        printf("error paramenter! please check");
        return;
    }
}

/**
  * @brief          �켣�岹
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_interpolation(path_t *path, interpolation_t *inp_data)
{
    static float pre_v = 0;      //ǰ�岹���ٶ� ֱ�߲岹��Բ���岹���õ�
    static float pre_w_z = 0;    //ǰ�岹��ת�� �³�ת���岹���õ�
    static float dl = 0, ds = 0; //΢��ֱ�߳� ΢�ֻ���
    static float dtheta = 0;     //΢�ֽǶ�
    static float acc_adjusted;   //ֱ�� ���ٶȵ���ֵ
    static float w_acc_adjusted; //С������� �Ǽ��ٶȵ���ֵ

    //����δ����򲻿�����һ�β岹
    if (isTrackDone == FALSE)
    {
        return;
    }

    if (fabs(path->arc_radius) < Round_Error) //����켣�İ뾶Ϊ0 �����ֱ�߲岹
    {
        if (inp_data->residue_distance > Round_Error) //ʣ��������0
        {
            dl = 0; //dl���� 0
            pre_v = inp_data->vel;
            inp_data->brake_distance = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (2 * ACC);
            //ʣ������������ɲ������ ���ٶ��𲽵ĵ���Ϊ������Ŀ��(���)�ٶ� ���Լ��١����١�����
            if (inp_data->residue_distance - inp_data->brake_distance > Round_Error)
            {
                if (inp_data->vel < path->v_target) //����
                {
                    inp_data->vel += ACC * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_target);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else if (inp_data->vel > path->v_target) //����
                {
                    inp_data->vel -= ACC * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_target);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else //����(�ﵽĿ���ٶ�)
                {
                    inp_data->vel = path->v_target;
                    dl = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }
            //ʣ�����С������ɲ������ ���ٶ���������Ϊ �������յ��ٶ�
            else
            {
                //���ļ��ٶȣ�ʹС���������յ��ٶ�ǡ��ΪҪ����ٶ�
                acc_adjusted = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (2 * inp_data->residue_distance);
                if (inp_data->vel < path->v_end) //����
                {
                    inp_data->vel += acc_adjusted * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_end);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else if (inp_data->vel > path->v_end) //����
                {
                    inp_data->vel -= acc_adjusted * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_end);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else //����(�ﵽĿ���ٶ�֮��)
                {
                    inp_data->vel = path->v_end;
                    dl = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }

            inp_data->past_distance += dl;                                                   //�������߹��ľ���
            inp_data->residue_distance = inp_data->total_distance - inp_data->past_distance; //����ʣ�����

            if (inp_data->residue_distance < Round_Error) //�ж�ʣ������Ƿ�Ϊ0
            {
                inp_data->past_distance = inp_data->total_distance;
                inp_data->residue_distance = 0;
                inp_data->vel = path->v_end;
                inp_data->line_interpolation_is_done = TRUE; //ֱ�߲岹��ɱ�־λ
            }

            inp_data->x = inp_data->past_distance * cos(path->path_theta) + path->start_x; //��һ�����λ�� �������������
            inp_data->y = inp_data->past_distance * sin(path->path_theta) + path->start_y; //��һ�����λ�� �������������

        }    //END inp_data->residue_distance > Round_Error
        else //ʣ�����0
        {
            inp_data->line_interpolation_is_done = TRUE; //ֱ�߲岹��ɱ�־λ
        }

    }    //END ֱ�߲岹
    else //����켣�İ뾶��Ϊ0�� ���� ****Բ���岹****
    {
        //Բ���岹��distance����Բ���ĳ���
        if (inp_data->residue_distance > Round_Error) //�ٶȷ���� �岹
        {
            ds = 0;
            pre_v = inp_data->vel;
            inp_data->brake_distance = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (ACC * 2);
            //ʣ��Ƕȴ�������ɲ���Ƕ� ���ٶ��𲽵ĵ���Ϊ������Ŀ��(���)�ٶ� ���Լ��١����١�����
            if (inp_data->residue_distance - inp_data->brake_distance > Round_Error)
            {
                if (inp_data->vel < path->v_target) //����
                {
                    inp_data->vel += ACC * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_target);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else if (inp_data->vel > path->v_target) //����
                {
                    inp_data->vel -= ACC * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_target);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else //����(�ﵽĿ���ٶ�)
                {
                    inp_data->vel = path->v_target;
                    ds = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }
            //ʣ��Ƕ�С������ɲ���Ƕ� �����ٶ���������Ϊ �������յ��ٶ�
            else
            {
                //���ļ��ٶȣ�ʹС���������յ��ٶ�ǡ��ΪҪ����ٶ�
                acc_adjusted = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (2 * inp_data->residue_distance);
                if (inp_data->vel < path->v_end) //����
                {
                    inp_data->vel += acc_adjusted * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_end);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else if (inp_data->vel > path->v_end) //����
                {
                    inp_data->vel -= acc_adjusted * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_end);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else //����(�ﵽĿ���ٶ�֮��)
                {
                    inp_data->vel = path->v_end;
                    ds = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }

            inp_data->past_distance += ds;                                                   //�������߹��ľ���
            inp_data->residue_distance = inp_data->total_distance - inp_data->past_distance; //����ʣ�����

            inp_data->past_arcAngle = (inp_data->past_distance / inp_data->total_distance) * (inp_data->total_arcAngle); //�������߹���Բ���Ƕ�
            inp_data->residue_arcAngle = inp_data->total_arcAngle - inp_data->past_arcAngle;                             //����ʣ���Բ���Ƕ�

            if (inp_data->residue_distance < Round_Error)
            {
                inp_data->past_distance = inp_data->total_distance;
                inp_data->residue_distance = 0;
                inp_data->vel = path->v_end;
                inp_data->arc_interpolation_is_done = TRUE; //Բ���岹��ɱ�־
            }
            else
            {
                inp_data->arc_interpolation_is_done = FALSE; //Բ���岹��ɱ�־
            }

            if (path->arc_direction == CW)
            {
                inp_data->theta = path->start_theta - inp_data->past_arcAngle;
            }
            else
            {
                inp_data->theta = path->start_theta + inp_data->past_arcAngle;
            }

            //ʹ������Բ���뾶
            path->arc_radius_actual = path->arc_radius;
            //ʹ����ʵԲ���뾶 �����������ʱ����ʵ�����ʰ뾶(Ҳ�������ΪԲ���뾶) �������ʵ��Ĵη�����
            // path->arc_radius_actual = path->arc_radius * (1 + 0.5 * pow(inp_data->past_arcAngle, 2) -
            //                                               pow(inp_data->past_arcAngle, 3) / inp_data->total_arcAngle +
            //                                               0.5 * pow(inp_data->past_arcAngle, 4) / pow(inp_data->total_arcAngle, 2));

            if (path->arc_direction == CW) //˳ʱ��
            {
                if ((path->start_x < path->end_x) && (path->start_y > path->end_y))
                {
                    inp_data->x = path->start_x + path->arc_radius_actual * sin(inp_data->past_arcAngle);
                    inp_data->y = path->start_y - path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                }
                else if ((path->start_x < path->end_x) && (path->start_y < path->end_y))
                {
                    inp_data->x = path->start_x + path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                    inp_data->y = path->start_y + path->arc_radius_actual * sin(inp_data->past_arcAngle);
                }
                else if ((path->start_x > path->end_x) && (path->start_y < path->end_y))
                {
                    inp_data->x = path->start_x - path->arc_radius_actual * sin(inp_data->past_arcAngle);
                    inp_data->y = path->start_y + path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                }
                else if ((path->start_x > path->end_x) && (path->start_y > path->end_y))
                {
                    inp_data->x = path->start_x - path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                    inp_data->y = path->start_y - path->arc_radius_actual * sin(inp_data->past_arcAngle);
                }
            }
            else //��ʱ��
            {
                if ((path->start_x > path->end_x) && (path->start_y < path->end_y))
                {
                    inp_data->x = path->start_x - path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                    inp_data->y = path->start_y + path->arc_radius_actual * sin(inp_data->past_arcAngle);
                }
                else if ((path->start_x > path->end_x) && (path->start_y > path->end_y))
                {
                    inp_data->x = path->start_x - path->arc_radius_actual * sin(inp_data->past_arcAngle);
                    inp_data->y = path->start_y - path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                }
                else if ((path->start_x < path->end_x) && (path->start_y > path->end_y))
                {
                    inp_data->x = path->start_x + path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                    inp_data->y = path->start_y - path->arc_radius_actual * sin(inp_data->past_arcAngle);
                }
                else if ((path->start_x < path->end_x) && (path->start_y < path->end_y))
                {
                    inp_data->x = path->start_x + path->arc_radius_actual * sin(inp_data->past_arcAngle);
                    inp_data->y = path->start_y + path->arc_radius_actual * (1 - cos(inp_data->past_arcAngle));
                }
            }
        } //END
        else
        {
            inp_data->arc_interpolation_is_done = TRUE; //Բ���岹��ɱ�־
        }
    } //END Բ���岹

    //��������ת�ǶȲ岹 ��������ֱ�߻���Բ�������Խ�����ת�岹

    inp_data->theta_interpolation_is_done = TRUE; //��ת�ǶȲ岹��� ��־

    //printf("inp_data->x %f,inp_data->y %f,inp_data->w_z %f \r\n", inp_data->x, inp_data->y, inp_data->w_z);

    //�жϲ岹�Ƿ���� ֱ�߲岹��Բ���岹������ͬʱ���� ���� ֱ��+ת�� Բ��+ת�� ��ɾ���Ϊ���еĲ岹�������
    if ((inp_data->line_interpolation_is_done == TRUE || inp_data->arc_interpolation_is_done == TRUE) &&
        (inp_data->theta_interpolation_is_done == TRUE))
    {
        inp_data->both_interpolation_is_done = TRUE;
        //path->trail_flag = 1;
    }
}

//���ٶ����� ��֤ÿ��������ٶȲ��ᷢ��ͻ��
void accel_limit(fp32 value, fp32 pre_value, fp32 acc)
{
    if (fabs(value - pre_value) > acc)
    {
        if (value > pre_value)
        {
            value = pre_value + acc;
        }
        else if (value < pre_value)
        {
            value = pre_value - acc;
        }
    }
}

//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue)
{
    if (maxValue < minValue)
        return Input;
    if (Input > maxValue)
    {
        fp32 len = maxValue - minValue;
        while (Input > maxValue)
            Input -= len;
    }
    else if (Input < minValue)
    {
        fp32 len = maxValue - minValue;
        while (Input < minValue)
            Input += len;
    }
    return Input;
}
/**
  * @brief          �켣���ٿ����� ʵʱ�������������� ���ƻ����˵�������λ��
  * @param[in]      chassis_move_t *chassisMove ����λ��
  * @param[in]      pps_feedback_t ppsFeedback ʵ��λ��(��λϵͳ����)
  * @param[out]     vx_set: x�����ٶ�ָ��, vy_set: y�����ٶ�ָ��, wz_set: z�����ٶ�ָ��
  * @retval         none
  */
void track_controller(path_t *path, interpolation_t *inp_data, pps_feedback_t *ppsFeedback, fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    //����Ŀ�����
    static float Vx_to_control;
    static float Vy_to_control;
    static float Wz_to_control;
    //ȫ��������
    static float global_ex = 0;
    static float global_ey = 0;
    static float global_etheta = 0;
    //С������ϵ������
    static float local_ex = 0;
    static float local_ey = 0;
    static float local_etheta = 0;
    //�м���
    static float Kx, Ky, Kphi;

    //��¼��һ�����ڵĿ�����
    static float pre_x;
    static float pre_y;
    static float pre_w_z;

    pre_x = Vx_to_control;
    pre_y = Vy_to_control;
    pre_w_z = Wz_to_control;

    //ȫ��λ����� = ����λ�� - ʵ��λ��
    global_ex = inp_data->x - ppsFeedback->pos_x;
    global_ey = inp_data->y - ppsFeedback->pos_y;
    global_etheta = inp_data->theta - ppsFeedback->theta;

    //0-2PI ת��Ϊ -PI �� +PI
    loop_fp32_constrain(global_etheta, -PI, PI);

    //С������ϵ�µ����
    local_ex = global_ex * cos(inp_data->theta) + global_ey * sin(inp_data->theta);
    local_ey = -global_ex * sin(inp_data->theta) + global_ey * cos(inp_data->theta);
    local_etheta = global_etheta;

    //0-2PI ת��Ϊ -PI �� +PI
    loop_fp32_constrain(local_etheta, -PI, PI);

    // �ж������в岹�� ���й����в岹���ж�
    if (fabs(inp_data->y - ppsFeedback->pos_y) < X_Y_Error_0 &&
        fabs(inp_data->x - ppsFeedback->pos_x) < X_Y_Error_0 &&
        fabs(inp_data->theta - ppsFeedback->theta) < Theta_Error_0)
    {
        isTrackDone = TRUE;
    }
    else
    {
        isTrackDone = FALSE;
    }

    // �ж��յ� ���۵�-��ʵ�� ������Χ && ������·��Ϊ���һ��·�� ����Ϊ��ƫ����
    if (fabs(inp_data->y - ppsFeedback->pos_y) < X_Y_Error_1 &&
        fabs(inp_data->x - ppsFeedback->pos_x) < X_Y_Error_1 &&
        fabs(inp_data->theta - ppsFeedback->theta) < Theta_Error_1 &&
        (path_index == TEST_PATH_NUMBER - 1))
    {
        // ָ�봫��
        *wz_set = 0;
        *vx_set = 0;
        *vy_set = 0;
    }
    else // ��ƫû�н��� ���п���������
    {
        // ������Ϲ�ʽ ������������ʽ ���������
        Kx = 1 / (A_x + B_x * fabs(local_ex)) + C_x;
        Ky = 1 / (A_y + B_y * fabs(local_ey)) + C_y;
        Kphi = 1 / (A_theta + B_theta * fabs(local_etheta)) + C_theta;

        // ���������� x y z �Ŀ�����
        Wz_to_control = inp_data->w_z + Kphi * local_etheta;
        Vx_to_control = inp_data->vel * cos(inp_data->vTheta) + Kx * local_ex + Wz_to_control * local_ey;
        Vy_to_control = inp_data->vel * sin(inp_data->vTheta) + Ky * local_ey - Wz_to_control * local_ex;

        // �ԽǼ��ٶȽ�������
        accel_limit(Wz_to_control, pre_w_z, EW_ACC * TRAIL_REVISE_TIME);
        // ��X������ٶȽ�������
        accel_limit(Vx_to_control, pre_x, EACC * TRAIL_REVISE_TIME);
        // ��Y������ٶȽ�������
        accel_limit(Vy_to_control, pre_y, EACC * TRAIL_REVISE_TIME);
//    // �Խ��ٶȽ�������
//    constrain(Wz_to_control, -50, 50);
//    // ��Y�����ٶȽ�������
//    constrain(Vy_to_control, -40, 40);
        // ָ�봫��
        *wz_set = Wz_to_control;
        *vx_set = Vx_to_control;
        *vy_set = Vy_to_control;
    }
}
