/**
  ****************************(C) COPYRIGHT 2020 HCRT****************************
  * @file       navigation.c/h
  * @brief      导航功能包 
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V0.1.0     2020.2.12     	 YLT              	0.1
  *  V1.0.0     2020.5.2     	 YLT              	1.0
  @verbatim
  ==============================================================================
   	* update 2020.5.2 :(release)
		1.Webots仿真测试版本 所有功能均已实现 消除了算法存在的BUG
		2.现在统一以一个任务调用navigation_task状态机可以实现路径的初始化、插补、跟踪、
		以及错误状态的处理。
		3.插补和跟踪合并在一个任务中 只有跟踪完才会插补出下一个点
	* update 2020.2.12 :(beta)
		1.优化函数结构 传参使用结构体数组 现在在数组中修改路径 
    	2.留下远程传参的结构 使用遥控就可以发送路径信息 修改数组中的内容就可以改变路径
    	3.直线 圆弧 自转 融合为一个函数 精简了一些之前冗余的变量 
    	4.重点修改的是圆弧部分 圆弧由之前的三点圆弧改为 两点加半径圆弧 计算量要减少许多
    	5.将 控制器和插补器 分开 插补器以200HZ的频率更新理论点 理论角度  控制器以500HZ
    	的频率控制机器人到达理论点 控制器到达理论点之后会改变一个标志位告诉插补器 插补器
    	就会开始插补下一个点 直到所有的路径走完 即末速度为零 插补结束 控制器依然实时纠偏
    	将机器人定在末尾点  
    	6.补充 现在依然是插补器 算出一个点 控制器跟踪一个点 跟踪完了 再计算下一个点 这样
    	的方式节省内存 但是如若单片机的内存并不是问题的情况下 可以考虑在一开始将理论点全
		部算出来 储存在闪存中 再读取 可能会提高精度
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

#define TEST_PATH_NUMBER 6 //测试路径的条数

uint8_t path_index = 0; //路径索引 用来切换路径
static fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

E_NAV_STATE NAV_STATE = NAV_STATE_INIT; //状态机索引
logic_e isTrackDone;                    //全局跟踪完成标志位
pps_feedback_t pps_feedback;            //定位系统反馈全局位置信息
golbal_position_t global_position;      //全局位姿

static interpolation_t interpolation_data[TEST_PATH_NUMBER];
/*	这个测试路径是一个圆角矩形 由四段直线和四段圆弧组成
	必须：start_x , start_y  | end_x , end_y  | arc_center_x , arc_center_y | arc_radius , arc_direction
	可选: v_start , v_target , v_end | w_start , w_target , w_end
*/
static path_t test_path[TEST_PATH_NUMBER] = {
    //sx     sy  |  ex    ey    | ax     ay  | ar     ad
    //起点x 起点y | 终点x  终点y | 圆心x  圆心y| 圆弧半径 圆弧方向
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
  * @brief          导航任务 输出电机控制量
  * @param[in]      none 
  * @retval         none
  */
void NavigationTask(void const *argument)
{
    for (;;)
    {
        switch (NAV_STATE)
        {
        case NAV_STATE_INIT: //********** 初始化状态 **********
            for (path_index = 0; path_index < TEST_PATH_NUMBER; path_index++)
            {
                //初始化数据 路径使用的是不同的内存 因此可以一次性全部初始化完 不会覆盖
                path_init(TEST_PATH_NUMBER, path_index, &test_path[path_index], &interpolation_data[path_index]);
#ifdef DEBUG
                printf("INIT!: path_index %d, residue_distance %f, residue_theta %f, residue_vTheta %f \r\n",
                       path_index, interpolation_data[path_index].residue_distance,
                       interpolation_data[path_index].residue_theta, interpolation_data[path_index].residue_vTheta);
#endif
            }
            path_index = 0; //路径索引置零
            isTrackDone = TRUE;
            if (HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin) == 0)
                NAV_STATE = NAV_STATE_RUN; //切换到运行状态
            break;
        case NAV_STATE_RUN: //********** 运行状态 **********
            //判断一段插补是否结束 结束的话就把 path_index 加一进入下一段插补
            (interpolation_data[path_index].both_interpolation_is_done == TRUE) ? (path_index++) : (path_index = path_index);
            high_constrain(path_index, TEST_PATH_NUMBER - 1); //上限 限幅
            //插补 运行一次 插补出一个点
            path_interpolation(&test_path[path_index], &interpolation_data[path_index]);
            /* 更新世界坐标 */
            pps_feedback.pos_y = Odom.Pos.y;
            pps_feedback.pos_x = Odom.Pos.x;
            pps_feedback.theta = Odom.Pos.theta;
            /* 轨迹跟踪控制器 */
            track_controller(&test_path[path_index], &interpolation_data[path_index], &pps_feedback, &vx_set, &vy_set, &wz_set);
            /* vx-m/s  vy-m/s  wz-rad/s */
            Output_Wheel(vx_set, vy_set, wz_set); //输出速度到轮子
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
        case NAV_STATE_ERROR: //********** 错误状态 **********
            /* code */
            break;
        default:
            break;
        }
        osDelay(20);
    }
}

/**
  * @brief          初始化轨迹 计算一些变量
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_init(uint8_t total_path_num, uint8_t current_path_num, path_t *path, interpolation_t *inp_data)
{
    static float last_path_v_end = 0.0f;

    if (fabs(path->arc_radius) < Round_Error) // ****直线插补参数初始化****
    {
        //计算轨迹角度 起始点和终止点连线 L 相对于x轴的角度 路径角度
        path->path_theta = atan2(path->end_y - path->start_y, path->end_x - path->start_x);
        //初始化插补数据 直线插补数据初始化
        inp_data->total_distance = sqrt(pow((path->end_x - path->start_x), 2) + pow((path->end_y - path->start_y), 2)); //直线插补 直线的总长
        inp_data->brake_distance = 0;
        inp_data->past_distance = 0;
        inp_data->residue_distance = inp_data->total_distance; //剩余距离
    }
    else // ****圆弧插补参数初始化****
    {
        //使用余弦定理根据起点终点弧长计算总转角
        inp_data->total_arcAngle = acos(1 - (pow(path->start_x - path->end_x, 2) + pow(path->start_y - path->end_y, 2)) / (2 * pow(path->arc_radius, 2)));
        inp_data->brake_arcAngle = 0;
        inp_data->past_arcAngle = 0;
        inp_data->residue_arcAngle = inp_data->total_arcAngle;
        //初始化插补数据 圆弧插补数据初始化
        inp_data->total_distance = (2 * PI * path->arc_radius) * (inp_data->total_arcAngle / (2 * PI)); //圆弧总长
        inp_data->brake_distance = 0;
        inp_data->past_distance = 0;
        inp_data->residue_distance = inp_data->total_distance; //剩余距离
        path->arc_radius_actual = path->arc_radius;
        // path->arc_radius_actual = path->arc_radius * (1 + 0.5 * pow(inp_data->past_arcAngle, 2) -
        //                                               pow(inp_data->past_arcAngle, 3) / inp_data->total_arcAngle +
        //                                               0.5 * pow(inp_data->past_arcAngle, 4) / pow(inp_data->total_arcAngle, 2));
        // printf("arc_radius %f,arc_center_x %f,arc_center_y %f \r\n", path->arc_radius, path->arc_center_x, path->arc_center_y);
        // printf("ARC! total_distance %f \r\n", inp_data->total_distance);
    }

    //初始化插补数据 自转插补数据初始化
    inp_data->total_theta = fabs(path->end_theta - path->start_theta); //自转角度插补 小车方向角 总转角
    inp_data->brake_theta = 0;
    inp_data->past_theta = 0;
    inp_data->residue_theta = (inp_data->total_theta); //剩余弧度用绝对值表示

    //插补标志位 置位
    inp_data->line_interpolation_is_done = FALSE;  //直线插补完成标志位
    inp_data->theta_interpolation_is_done = FALSE; //车声转角插补完成标志位
    inp_data->arc_interpolation_is_done = FALSE;   //圆弧插补完成标志位
    inp_data->both_interpolation_is_done = FALSE;  //总插补完成标志位

    //判断自旋方向
    (path->start_theta <= path->end_theta) ? (path->rotate_direction = CCW) : (path->rotate_direction = CW);

    path->path_id = current_path_num; //将每一段路径赋予唯一的ID

    inp_data->x = path->start_x; //将第一个插补点的坐标定位赋值为起点坐标
    inp_data->y = path->start_y; //将第一个插补点的坐标定位赋值为起点坐标

    if (current_path_num == 0) //第一条路径 起点速度为0
    {
        inp_data->vel = 0.0f;
        path->v_start = 0.0f;
        path->v_end = MAX_PLAN_VEL;
    }
    else if (current_path_num == total_path_num - 1) //最后一条路径 终点速度为0
    {
        inp_data->vel = last_path_v_end;
        path->v_start = last_path_v_end;
        path->v_end = 0.0f;
    }
    else //中间路径
    {
        inp_data->vel = last_path_v_end;
        path->v_start = last_path_v_end;
        path->v_end = MAX_PLAN_VEL;
    }
    //printf("INIT!: path_id %d, v_start %f, v_end %f \r\n", path->path_id, path->v_start, path->v_end);

    last_path_v_end = path->v_end; //记录上一条路径的终点速度

    path->v_target = MAX_PLAN_VEL; //设定最大速度 即目标速度
    path->w_target = MAX_PLAN_W;   //设定最大角速度 即目标角速度

    inp_data->theta = path->start_theta;

    //判断起始点、终止点的速度是否超过最大速度 超过则报错
    if ((path->v_start > path->v_target) || (path->w_start > path->w_target) ||
        (path->v_end > path->v_target) || (path->w_end > path->w_target))
    {

        printf("error paramenter! please check");
        return;
    }
}

/**
  * @brief          轨迹插补
  * @param[in]      path_t path 
  * @param[in]      interpolation_t inp_data
  * @retval         none
  */
void path_interpolation(path_t *path, interpolation_t *inp_data)
{
    static float pre_v = 0;      //前插补点速度 直线插补和圆弧插补中用到
    static float pre_w_z = 0;    //前插补点转速 下车转件插补中用到
    static float dl = 0, ds = 0; //微分直线长 微分弧长
    static float dtheta = 0;     //微分角度
    static float acc_adjusted;   //直线 加速度调整值
    static float w_acc_adjusted; //小车方向角 角加速度调整值

    //跟踪未完成则不开启下一次插补
    if (isTrackDone == FALSE)
    {
        return;
    }

    if (fabs(path->arc_radius) < Round_Error) //如果轨迹的半径为0 则进入直线插补
    {
        if (inp_data->residue_distance > Round_Error) //剩余距离大于0
        {
            dl = 0; //dl先清 0
            pre_v = inp_data->vel;
            inp_data->brake_distance = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (2 * ACC);
            //剩余距离大于理论刹车距离 将速度逐步的调整为给定的目标(最大)速度 可以加速、减速、匀速
            if (inp_data->residue_distance - inp_data->brake_distance > Round_Error)
            {
                if (inp_data->vel < path->v_target) //加速
                {
                    inp_data->vel += ACC * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_target);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else if (inp_data->vel > path->v_target) //减速
                {
                    inp_data->vel -= ACC * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_target);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else //匀速(达到目标速度)
                {
                    inp_data->vel = path->v_target;
                    dl = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }
            //剩余距离小于理论刹车距离 将速度慢慢调整为 给定的终点速度
            else
            {
                //更改加速度，使小车可以在终点速度恰好为要求的速度
                acc_adjusted = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (2 * inp_data->residue_distance);
                if (inp_data->vel < path->v_end) //加速
                {
                    inp_data->vel += acc_adjusted * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_end);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else if (inp_data->vel > path->v_end) //减速
                {
                    inp_data->vel -= acc_adjusted * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_end);
                    dl = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else //匀速(达到目标速度之后)
                {
                    inp_data->vel = path->v_end;
                    dl = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }

            inp_data->past_distance += dl;                                                   //更新已走过的距离
            inp_data->residue_distance = inp_data->total_distance - inp_data->past_distance; //更新剩余距离

            if (inp_data->residue_distance < Round_Error) //判断剩余距离是否为0
            {
                inp_data->past_distance = inp_data->total_distance;
                inp_data->residue_distance = 0;
                inp_data->vel = path->v_end;
                inp_data->line_interpolation_is_done = TRUE; //直线插补完成标志位
            }

            inp_data->x = inp_data->past_distance * cos(path->path_theta) + path->start_x; //下一个点的位置 这个给到控制器
            inp_data->y = inp_data->past_distance * sin(path->path_theta) + path->start_y; //下一个点的位置 这个给到控制器

        }    //END inp_data->residue_distance > Round_Error
        else //剩余距离0
        {
            inp_data->line_interpolation_is_done = TRUE; //直线插补完成标志位
        }

    }    //END 直线插补
    else //如果轨迹的半径不为0则 进入 ****圆弧插补****
    {
        //圆弧插补中distance代表圆弧的长度
        if (inp_data->residue_distance > Round_Error) //速度方向角 插补
        {
            ds = 0;
            pre_v = inp_data->vel;
            inp_data->brake_distance = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (ACC * 2);
            //剩余角度大于理论刹车角度 将速度逐步的调整为给定的目标(最大)速度 可以加速、减速、匀速
            if (inp_data->residue_distance - inp_data->brake_distance > Round_Error)
            {
                if (inp_data->vel < path->v_target) //加速
                {
                    inp_data->vel += ACC * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_target);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else if (inp_data->vel > path->v_target) //减速
                {
                    inp_data->vel -= ACC * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_target);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * ACC);
                }
                else //匀速(达到目标速度)
                {
                    inp_data->vel = path->v_target;
                    ds = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }
            //剩余角度小于理论刹车角度 将角速度慢慢调整为 给定的终点速度
            else
            {
                //更改加速度，使小车可以在终点速度恰好为要求的速度
                acc_adjusted = fabs(pow(inp_data->vel, 2) - pow(path->v_end, 2)) / (2 * inp_data->residue_distance);
                if (inp_data->vel < path->v_end) //加速
                {
                    inp_data->vel += acc_adjusted * TRAIL_REVISE_TIME;
                    high_constrain(inp_data->vel, path->v_end);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else if (inp_data->vel > path->v_end) //减速
                {
                    inp_data->vel -= acc_adjusted * TRAIL_REVISE_TIME;
                    low_constrain(inp_data->vel, path->v_end);
                    ds = fabs(pow(inp_data->vel, 2) - pow(pre_v, 2)) / (2 * acc_adjusted);
                }
                else //匀速(达到目标速度之后)
                {
                    inp_data->vel = path->v_end;
                    ds = inp_data->vel * TRAIL_REVISE_TIME;
                }
            }

            inp_data->past_distance += ds;                                                   //更新已走过的距离
            inp_data->residue_distance = inp_data->total_distance - inp_data->past_distance; //更新剩余距离

            inp_data->past_arcAngle = (inp_data->past_distance / inp_data->total_distance) * (inp_data->total_arcAngle); //更新已走过的圆弧角度
            inp_data->residue_arcAngle = inp_data->total_arcAngle - inp_data->past_arcAngle;                             //更新剩余的圆弧角度

            if (inp_data->residue_distance < Round_Error)
            {
                inp_data->past_distance = inp_data->total_distance;
                inp_data->residue_distance = 0;
                inp_data->vel = path->v_end;
                inp_data->arc_interpolation_is_done = TRUE; //圆弧插补完成标志
            }
            else
            {
                inp_data->arc_interpolation_is_done = FALSE; //圆弧插补完成标志
            }

            if (path->arc_direction == CW)
            {
                inp_data->theta = path->start_theta - inp_data->past_arcAngle;
            }
            else
            {
                inp_data->theta = path->start_theta + inp_data->past_arcAngle;
            }

            //使用理论圆弧半径
            path->arc_radius_actual = path->arc_radius;
            //使用真实圆弧半径 计算现在这个时刻真实的曲率半径(也可以理解为圆弧半径) 连续曲率的四次方曲线
            // path->arc_radius_actual = path->arc_radius * (1 + 0.5 * pow(inp_data->past_arcAngle, 2) -
            //                                               pow(inp_data->past_arcAngle, 3) / inp_data->total_arcAngle +
            //                                               0.5 * pow(inp_data->past_arcAngle, 4) / pow(inp_data->total_arcAngle, 2));

            if (path->arc_direction == CW) //顺时针
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
            else //逆时针
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
            inp_data->arc_interpolation_is_done = TRUE; //圆弧插补完成标志
        }
    } //END 圆弧插补

    //机器人自转角度插补 无论是在直线还是圆弧都可以进行旋转插补

    inp_data->theta_interpolation_is_done = TRUE; //自转角度插补完成 标志

    //printf("inp_data->x %f,inp_data->y %f,inp_data->w_z %f \r\n", inp_data->x, inp_data->y, inp_data->w_z);

    //判断插补是否结束 直线插补和圆弧插补不可能同时进行 所以 直线+转角 圆弧+转件 完成就认为所有的插补都完成了
    if ((inp_data->line_interpolation_is_done == TRUE || inp_data->arc_interpolation_is_done == TRUE) &&
        (inp_data->theta_interpolation_is_done == TRUE))
    {
        inp_data->both_interpolation_is_done = TRUE;
        //path->trail_flag = 1;
    }
}

//加速度限制 保证每次输出的速度不会发生突变
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

//循环限幅函数
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
  * @brief          轨迹跟踪控制器 实时输出控制量给电机 控制机器人到达理论位置
  * @param[in]      chassis_move_t *chassisMove 理论位置
  * @param[in]      pps_feedback_t ppsFeedback 实际位置(定位系统反馈)
  * @param[out]     vx_set: x方向速度指针, vy_set: y方向速度指针, wz_set: z方向速度指针
  * @retval         none
  */
void track_controller(path_t *path, interpolation_t *inp_data, pps_feedback_t *ppsFeedback, fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
    //输出的控制量
    static float Vx_to_control;
    static float Vy_to_control;
    static float Wz_to_control;
    //全局误差矩阵
    static float global_ex = 0;
    static float global_ey = 0;
    static float global_etheta = 0;
    //小车坐标系误差矩阵
    static float local_ex = 0;
    static float local_ey = 0;
    static float local_etheta = 0;
    //中间量
    static float Kx, Ky, Kphi;

    //记录上一个周期的控制量
    static float pre_x;
    static float pre_y;
    static float pre_w_z;

    pre_x = Vx_to_control;
    pre_y = Vy_to_control;
    pre_w_z = Wz_to_control;

    //全局位置误差 = 理论位置 - 实际位置
    global_ex = inp_data->x - ppsFeedback->pos_x;
    global_ey = inp_data->y - ppsFeedback->pos_y;
    global_etheta = inp_data->theta - ppsFeedback->theta;

    //0-2PI 转换为 -PI 到 +PI
    loop_fp32_constrain(global_etheta, -PI, PI);

    //小车坐标系下的误差
    local_ex = global_ex * cos(inp_data->theta) + global_ey * sin(inp_data->theta);
    local_ey = -global_ex * sin(inp_data->theta) + global_ey * cos(inp_data->theta);
    local_etheta = global_etheta;

    //0-2PI 转换为 -PI 到 +PI
    loop_fp32_constrain(local_etheta, -PI, PI);

    // 判断运行中插补点 运行过程中插补点判断
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

    // 判断终点 理论点-真实点 满足误差范围 && 所处的路径为最后一条路径 则认为纠偏结束
    if (fabs(inp_data->y - ppsFeedback->pos_y) < X_Y_Error_1 &&
        fabs(inp_data->x - ppsFeedback->pos_x) < X_Y_Error_1 &&
        fabs(inp_data->theta - ppsFeedback->theta) < Theta_Error_1 &&
        (path_index == TEST_PATH_NUMBER - 1))
    {
        // 指针传参
        *wz_set = 0;
        *vx_set = 0;
        *vy_set = 0;
    }
    else // 纠偏没有结束 进行控制量计算
    {
        // 参数混合公式 将参数换个形式 更方便调参
        Kx = 1 / (A_x + B_x * fabs(local_ex)) + C_x;
        Ky = 1 / (A_y + B_y * fabs(local_ey)) + C_y;
        Kphi = 1 / (A_theta + B_theta * fabs(local_etheta)) + C_theta;

        // 根据误差计算 x y z 的控制量
        Wz_to_control = inp_data->w_z + Kphi * local_etheta;
        Vx_to_control = inp_data->vel * cos(inp_data->vTheta) + Kx * local_ex + Wz_to_control * local_ey;
        Vy_to_control = inp_data->vel * sin(inp_data->vTheta) + Ky * local_ey - Wz_to_control * local_ex;

        // 对角加速度进行限制
        accel_limit(Wz_to_control, pre_w_z, EW_ACC * TRAIL_REVISE_TIME);
        // 对X方向加速度进行限制
        accel_limit(Vx_to_control, pre_x, EACC * TRAIL_REVISE_TIME);
        // 对Y方向加速度进行限制
        accel_limit(Vy_to_control, pre_y, EACC * TRAIL_REVISE_TIME);
//    // 对角速度进行限制
//    constrain(Wz_to_control, -50, 50);
//    // 对Y方向速度进行限制
//    constrain(Vy_to_control, -40, 40);
        // 指针传参
        *wz_set = Wz_to_control;
        *vx_set = Vx_to_control;
        *vy_set = Vy_to_control;
    }
}
