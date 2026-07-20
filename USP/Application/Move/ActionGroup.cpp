#include "ActionGroup.h"
#include "internal.h"
/**
 * @brief 关节坐标系的动作组执行函数
 * @param _tpa1 轨迹点组
 * @param _a 输入动作组，默认是七个关节+一个时间
 * @param _Num 动作组的目标数量
 */
void ActionGroup_WithJoint(TrajectoryPointArray _tpa1[7], float _a[][8], int _Num)
{
    // 是否需要加一些判断条件？满足条件后才能执行动作组

    for (uint8_t i = 0; i < 7; i++)
    {
        _tpa1[i].clear(); // 需要清空，不仅是节省空间，更是防止会产生两个不同动作的阶跃（一个结束和一个开始的）
    }

    for (uint8_t i = 0; i < _Num; i++)
    {
        for (uint8_t j = 0; j < 7; j++)
        {
            _tpa1[j].add(_a[i][7], _a[i][j]);
        }
    }
}


/**
 * @brief 动作组的选择函数
 * @param _tpa1 轨迹点组
 * @param _action_enum 动作组枚举
 * @param _Add_onlyOnce 是否只添加一次
 */
void ActionGroup_Choose(TrajectoryPointArray _tpa1[7], ActionGroup_enum _action_enum, uint8_t _Add_onlyOnce)
{
    /*选择动作组*/
    switch (_action_enum)
    {
    case ActionGroup_enum::NuLLAction: // 空动作组什么都不执行
        break;

        /*复位动作组*/
    case ActionGroup_enum::ResetNoOre: // 无矿复位动作组
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, ResetActionNoOre_Joint, 2);
        }
        break;
    case ActionGroup_enum::ResetOre: // 有矿复位动作组
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, ResetActionOre_Joint, 3);
        }
        break;

        /*机动动作组*/
    case ActionGroup_enum::Mobile_NoOre: // 无矿机动
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, MobileNoOreAction_Joint, 3);
            // ActionGroup_World(_tpa1, TempAction_World, 4);
        }
        break;
    case ActionGroup_enum::Mobile_Ore: // 有矿机动
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, MobileOreAction_Joint, 2);
        }
        break;
    case ActionGroup_enum::Mobile_Tunnel:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, MobileTunnelAction_Joint, 4);
        }
        break;

    // ========================================================================
    // 储矿槽动作组 — 4 槽 (LF=左前, RF=右前, LB=左后, RB=右后)
    // ========================================================================
    /*储存 (Store) */
    case ActionGroup_enum::StoreOre_LF:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, StoreOre_LF, 2);
        break;
    case ActionGroup_enum::StoreOre_RF:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, StoreOre_RF, 2);
        break;
    case ActionGroup_enum::StoreOre_LB:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, StoreOre_LB, 2);
        break;
    case ActionGroup_enum::StoreOre_RB:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, StoreOre_RB, 2);
        break;

    /*退出储存 (ExitStore) */
    case ActionGroup_enum::ExitStoreOre_LF:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, ExitStoreOre_LF, 2);
        break;
    case ActionGroup_enum::ExitStoreOre_RF:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, ExitStoreOre_RF, 2);
        break;
    case ActionGroup_enum::ExitStoreOre_LB:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, ExitStoreOre_LB, 2);
        break;
    case ActionGroup_enum::ExitStoreOre_RB:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, ExitStoreOre_RB, 3);
        break;

    /*取出 (TakeIn) */
    case ActionGroup_enum::TakeInOre_LF:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeInOre_LF, 2);
        break;
    case ActionGroup_enum::TakeInOre_RF:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeInOre_RF, 2);
        break;
    case ActionGroup_enum::TakeInOre_LB:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeInOre_LB, 2);
        break;
    case ActionGroup_enum::TakeInOre_RB:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeInOre_RB, 2);
        break;

    /*取出结束 (TakeIn_end) */
    case ActionGroup_enum::TakeinOreLF_end:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeinOreLF_end, 2);
        break;
    case ActionGroup_enum::TakeinOreRF_end:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeinOreRF_end, 2);
        break;
    case ActionGroup_enum::TakeinOreLB_end:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeinOreLB_end, 2);
        break;
    case ActionGroup_enum::TakeinOreRB_end:
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeinOreRB_end, 2);
        break;
    // 取能量单元
    case ActionGroup_enum::TakeOre1_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre1_prepare, 2);
        }
        break;
    case ActionGroup_enum::TakeOre2_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre2_prepare, 2);
        }
        break;    
    case ActionGroup_enum::TakeOre3_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre3_prepare, 2);
        }
        break;
    case ActionGroup_enum::TakeOre4_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre4_prepare, 2);
        }
        break;
    case ActionGroup_enum::TakeOre5_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre5_prepare, 2);
        }
        break;
    case ActionGroup_enum::TakeOre6_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre6_prepare, 2);
        }
        break;
    case ActionGroup_enum::TakeOre1_ing:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre1_ing, 2);
        }
        break;
    case ActionGroup_enum::TakeOre2_ing:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre2_ing, 2);
        }
        break;
    case ActionGroup_enum::TakeOre3_ing:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre3_ing, 2);
        }
        break;
    case ActionGroup_enum::TakeOre4_ing:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre4_ing, 2);
        }
        break;
    case ActionGroup_enum::TakeOre5_ing:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre5_ing, 2);
        }
        break;
    case ActionGroup_enum::TakeOre6_ing:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre6_ing, 2);
        }
        break;
    case ActionGroup_enum::TakeOre1_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre1_end, 2);
        }
        break;
    case ActionGroup_enum::TakeOre2_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre2_end, 2);
        }
        break;
    case ActionGroup_enum::TakeOre3_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre3_end, 2);
        }
        break;
    case ActionGroup_enum::TakeOre4_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre4_end, 2);
        }
        break;
    case ActionGroup_enum::TakeOre5_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre5_end, 2);
        }
        break;
    case ActionGroup_enum::TakeOre6_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre6_end, 2);
        }
        break;
    case ActionGroup_enum::ToActionGroup:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, ToActionGroup, 2);
        }
        break;

    // case ActionGroup_enum::TakeLandOre_prepare: // 取地矿准备
    //     if (_Add_onlyOnce == 1)
    //     {
    //         ActionGroup_WithJoint(_tpa1, TakeLandOre_prepare, 2);
    //     }
    //     break;
    // case ActionGroup_enum::TakeLandOre_end:
    //     if (_Add_onlyOnce == 1)
    //     {
    //         // ActionGroup_WithJoint(_tpa1, TakeLandOre_end, 3);
    //         ActionGroup_WithJoint(_tpa1, TakeLandOre_end, 1);
    //     }
    //     break;
    /*取肚子内能量单元*/


    case ActionGroup_enum::Stop: // 紧急停止执行动作

        break;

    default:
        break;
    }
}
/**
 * @brief 动作组执行函数
 * @param _j 添加的关节组
 * @param _tpa1 添加的轨迹点组
 * @param Joint_num 关节数量
 * @param _Add_onlyOnce 确保动作组只被添加一次的标志位
 */
void ActionGroup_Execute(Joint _j[7], TrajectoryPointArray _tpa1[7], int Joint_num, uint8_t &_Add_onlyOnce)
{
    if (_Add_onlyOnce == 1) // 确保添加轨迹点组只执行一次
    {
        for (int i = 0; i < Joint_num; i++)
        {

            if (_j[i].isCalibrated() == true && _j[i].isTrajectoryPlannerFree() == true)
            {
                _j[i].attach(&_tpa1[i]);
                _j[i].startTrajectoryPlanner(HAL_GetTick());
            }

            if (i == Joint_num - 1)
            {
                _Add_onlyOnce = 0;
            }
        }
    }

    uint32_t exec_tick = HAL_GetTick();
    for (int i = 0; i < Joint_num; i++)
    {
        _j[i].takeOver(exec_tick); // 执行放在这里执行的一个好处就是下面对于储矿的各个延时停顿能够即时的反映到机械臂上，而不是再做一个传递到其他任务上去

        // 轨迹速度/加速度前馈: 取关节空间目标导数, 转电机空间后写入 motor 前馈
        //   无轨迹(free)时 getTrajectoryDerivative 输出 0, 前馈自动关闭
        float v_des = 0.0f, a_des = 0.0f;
        _j[i].getTrajectoryDerivative(exec_tick + 1, &v_des, &a_des); // 与 takeOver 同用 +1

        const float k = _j[i].mapping.k;
        if (k != 0.0f)
        {
            // 关节空间(度/s, 度/s²) → 电机空间, 度→弧度 (假设 getSpeed 单位 rad/s)
            const float deg2rad = 3.14159265f / 180.0f;
            float v_motor = v_des / k * deg2rad; // rad/s, 注入速度环
            float a_motor = a_des / k * deg2rad; // rad/s², ×增益(含惯量)得力矩

            motor[i].setSpeed_feedforward(v_motor * speed_ff_gain[i]);
            motor[i].setAcceleration_feedforward(a_motor * acc_ff_gain[i]);
        }
    }
}

/*关节坐标系下*/
//  第一个【】内是多少套关节目标，后一个【】里的6是固定的，代表6个关节 全是角度制
// 多少套关节目标最好小于轨迹规划点组的数量

/*复位动作组*/
float ResetActionNoOre_Joint[2][8] = {
    {0, 90, 0, 0, 0, 0, 0, 700}, // 分别是J1，J2，。。。J6,间隔时间
    {-5, 150, -70, 0, 0, 0, 0, 700}};

float ResetActionOre_Joint[3][8] = {
    {0, 90, 0, 0, 0, 0, 0, 300},
    {0, 110, -20, 0, 0, 0, 0, 300},
    {0, 125, -45, 0, 0, 0, 0, 300}};

/*机动动作组*/
float MobileNoOreAction_Joint[3][8] = {
    {0, 155, 77, 17, 85, -93, 5, 700},
    {0, 140, 133, -72, 0, 0, 0, 700},
    {0, 153, 145, -77, 0, 0, 0, 700}};

float MobileOreAction_Joint[2][8] =
    {
        {0, 90, 90, 17, 165, 0, 0, 600},
        {0, 90, 90, -77, -165, 0, 0, 600}};

float MobileTunnelAction_Joint[4][8] = {
    {0, 155, 77, 17, 85, -93, 5, 700},
    {0, 130, 115, -75, 0, 0, 0, 700},
    {0, 155, 77, 17, 85, -93, 5, 700},
    {0, 144.8, 143.6, 15.7, 90, -100, 13.6, 500}};

/*取地矿*/
float TakeLandOre_prepare[2][8] = {
    {0, 80, -40, 0, -21, 0, 0, 800},
    {-0.966801, 67, -38.5765, 2.31819, -28.5342, 0, 0, 600}};

float TakeLandOre_ing[1][8] = {
    {-0.966801, 53.8021, -37.7262, 3.09748, -20.9517, 0, 0, 500}};

float TakeLandOre_end[1][8] = {
    {0, 135, -56.67, 0, -30, 0, 0, 1000}};

/*招财进宝*/
float TestAction[9][8] = {
    {0, 90, 0, 0, 0, 0, 0, 2000},
    {0, 35, 30, 0, 30, 0, 0, 1000},
    {-60, 35, 30, 0, 30, 0, 0, 1000},
    {0, 35, 30, 0, 30, 0, 0, 1000},
    {60, 35, 30, 0, 30, 0, 0, 1000},
    {0, 35, 30, 0, 30, 0, 0, 1000},
    {-60, 35, 30, 0, 30, 0, 0, 1000},   
    {0, 35, 30, 0, 30, 0, 0, 1000},
    {60, 35, 30, 0, 30, 0, 0, 1000}

};

float ShieldArmor[5][8] = {
    {0, 90, 0, 0, 0, 0, 0, 500},
    {0, 90, 25, 0, 0, 0, 0, 500},
    {0, 90, 0, 0, 0, 0, 0, 500},
    {0, 90, -40, 0, 0, 0, 0, 500},
    {0, 90, 0, 0, 0, 0, 0, 500}};

/*翻矿动作组*/
float TurnOre_prepare[2][8] = {
    {0, 90, 0, 0, 30, 0, 0, 800},
    {0, 48.435120, -39.614136, 0, 45.768303, 7.966847, 0, 800}};

float TurnOre_ing[5][8] = {
    {0, 45.260742, -56.531067, 0, 45.242409, 7.756055, 0, 400},
    {0, 60.190674, -63.145599, 0, 45.242409, 7.756055, 0, 400},
    {0, 85.327759, -48.014038, 0, 19.126671, 6.961287, 0, 400},
    {0, 85.877136, -31.916626, 0, -27.125877, 6.961681, 0, 400},
    {0, 52.598816, -30.506927, 0, -27.569252, 15.754708, 0, 1000}};

float TurnOre_end[1][8] = {
    {0, 120, -56.67, 0, 0, 9.579893, 0, 700}};

// 取能量单元
float TakeOre1_prepare[2][8] = 
{
    {0, 122, 83, -77, 0, 0, 0, 500},
    {0, 100.89, 69.22, -77, 0, -12.2, 0, 500}
    // {0, 114.79, -17.07, 0, 47.63, 0, 500}
};

float TakeOre2_prepare[2][8] = 
{
    {27.03, 104.52, 81.51, -77, 57.08, 0, 0, 500},
    {15.03, 92.61, 71.51, -77, 66.12, -10.54, -9.86, 500}
};

float TakeOre3_prepare[2][8] = 
{
    {18, 114, 111, -77, 66, -8, 46, 500},
    {13.25, 91.09, 95.35, -77, 65.48, -11.82, 51.09, 500}
};

float TakeOre4_prepare[2][8] = 
{
    {0, 96, 112, -77, 165, 0, 15, 800},
    {0, 72.98, 85.3, -77, 165, 3.67, 10.65, 500}
};

float TakeOre5_prepare[2][8] = 
{
    {-51, 123, 117, -77, -78, 23, -38, 500},
    {-28.85, 93.72, 95.83, -77, -107.31, 2.11, -10.43, 500}
};

float TakeOre6_prepare[2][8] = 
{
    {-52, 123, 94, -77, -82, 20, 6, 500},
    {-22.53, 98.21, 75.74, -77, -97.2, -4.3, 29.12, 500}
};


float TakeOre1_ing[2][8] = 
{
    {0, 90.12, 38.53, -77, 0, 8.87, 0, 500},
    {0, 90.12, 38.53, -77, 0, -30, 0, 500},
    // {0, 103, -49, 0, 60.20, 0, 400}
    // {0, 105.09, -44.52, 0, 61.66, 0, 500}
};

float TakeOre2_ing[2][8] = 
{
    // {39.72, 103.46, -15.63, 51.37, 48.03, 10.87, 400},
    // {46.27, 95.70, -28.98, 55.81, 57.69, 9.99, 400}
    {30.5, 80.23, 51.75, -77, 66.16, 3.3, -6.42, 500},
    {30.5, 80.23, 51.75, -77, 66.16, -30, -6.42, 500}
};

float TakeOre3_ing[2][8] = 
{
    {31.77, 79.97, 90.27, -77, 79.52, 9.75, 22.85, 500},
    {31.77, 79.97, 90.27, -77, 79.52, -20, 22.85, 500}
};

float TakeOre4_ing[2][8] = 
{
    {0, 64.97, 91.2, -77, 157.39, 16.41, 16.21, 500},
    {0, 64.97, 91.2, -77, 157.39, -10, 16.21, 500}
};

float TakeOre5_ing[2][8] = 
{
    {-42.56, 83.92,  97.63, -77, -131.92, 18.13, 20.76, 500},
    {-42.56, 83.92, 97.63, -77, -131.92, -30, 20.76, 500}
};

float TakeOre6_ing[2][8] = 
{
    {-40.41, 90.63, 57.95, -77, -95.89, 12.35, 22.34, 500},
    {-40.41, 90.63, 57.95, -77, -95.89, -30, 22.34, 500}
};
        

float TakeOre1_end[2][8] = 
{
    //{0, 99.14, -49, 0, 5.58, 0, 300}
    {0, 134, 88, -77, 0, -32, 0, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};

float TakeOre2_end[2][8] = 
{
    {32, 116, 95, -65, 43, -30, 0, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};

float TakeOre3_end[2][8] =
{
    {32, 118, 114, -77, 72, -20, 27, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};

float TakeOre4_end[2][8] = 
{
    {0, 100, 114, -66, 160, -10, 7, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};

float TakeOre5_end[2][8] = 
{
    {-39, 130, 112, -77, -136, -42, 0, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};

float TakeOre6_end[2][8] = 
{
    {-35, 128, 100, -77, -50, -20, 10, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};

// ============================================================================
// 储矿槽动作组 — 4 槽 (LF=左前, RF=右前, LB=左后, RB=右后)
//   格式: [N][8] = { {J1,J2,J3,J4,J5,J6,J7,time_ms}, ... }
//   TODO: 填入实际关节角度
// ============================================================================

// ── 储存 (Store) ──────────────────────────────────
float StoreOre_LF[2][8] = 
{
    {58.38, 135.17, 133.77, -77, 7.93, -17.37, -20.98, 500},
    {46.95, 144.75, 138, -46.78, 33.72, -33.81, -41.13, 500}
};
float StoreOre_RF[2][8] = 
{
    {-74.35, 132, 130.46, -74.29, -12, -26.19, 24.13, 500},
    {-82.53, 139.34, 146.43, -64.47, -12.94, -28.43, 26.5, 500}
};
float StoreOre_LB[2][8] = 
{
    // {0, 154, 103, -28, 0, -42, 0, 500},
    {162.27, 74.23, 98.73, -69.9, 7.73, -70.6, -10.76, 1200},
    {161.27, 57.41, 112.13, -77, 6.54, -95.4, -4.89, 600}
};
float StoreOre_RB[2][8] = 
{
    // {0, 154, 103, -28, 0, -42, 0, 500},
    {-172.18, 74.85, 106, -77, 4.52, -71.5, -2.99, 1200},
    {-167.01, 56.83, 110.79, -77, -0.73, -84.40, -3.93, 600}
};

// ── 退出储存 (ExitStore) ──────────────────────────
float ExitStoreOre_LF[2][8] = 
{
    {46.96, 155, 100.3, -6.04, 26.67, -26.03, -28.56, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float ExitStoreOre_RF[2][8] = 
{
    {-80, 155, 114.2, -15.87, -5.8, -28.23, 21.2, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float ExitStoreOre_LB[2][8] = 
{
    {151.13, 67.97, 116.81, -52.96, 16.08, -100, 0.63, 500},
    {0, 154, 103, -28, 0, -42, 0, 1200}
};
float ExitStoreOre_RB[3][8] = 
{
    {-166.68, 68.61, 129.69, -76.82, -2.79, -97.42, -2.04, 500},
    {-83, 111, 100, -53, 0, -70, 0, 600},
    {0, 154, 103, -28, 0, -42, 0, 600}
};

// ── 取出 (TakeIn) ─────────────────────────────────
float TakeInOre_LF[2][8] = 
{
    {46.96, 155, 100.3, -6.04, 26.67, -26.03, -28.56, 500},
    {46.95, 144.75, 138, -46.78, 33.72, -33.81, -41.13, 500}
};
float TakeInOre_RF[2][8] = 
{
    {-80, 155, 114.2, -15.87, -5.8, -28.23, 21.2, 500},
    {-82.53, 139.34, 146.43, -64.47, -12.94, -28.43, 26.5, 500}
};
float TakeInOre_LB[2][8] = 
{
    {151.13, 67.97, 116.81, -52.96, 16.08, -100, 0.63, 1200},
    {162.27, 54.41, 110.13, -77, 6.54, -95.4, -4.89, 500}
};
float TakeInOre_RB[2][8] = 
{
    {-172.68, 68.61, 129.69, -76.82, -2.79, -97.42, -2.04, 1200},
    {-172.01, 56.83, 110.79, -77, -0.73, -84.40, -3.93, 500}
};

// ── 取出结束 (TakeIn_end) ─────────────────────────
float TakeinOreLF_end[2][8] = 
{
    {58.38, 135.17, 133.77, -77, 7.93, -17.37, -20.98, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float TakeinOreRF_end[2][8] = 
{
    {-74.35, 132, 130.46, -74.29, -12, -26.19, 24.13, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float TakeinOreLB_end[2][8] = 
{
    {158.83, 74.94, 96.13, -61.09, 8.59, -80.57, -11.56, 500},
    {0, 154, 103, -28, 0, -42, 0, 1200}
};
float TakeinOreRB_end[2][8] = 
{
    {-168.18, 74.85, 106, -77, 4.52, -71.5, -2.99, 500},
    {0, 154, 103, -28, 0, -42, 0, 1200}
};

float ToActionGroup[2][8] = 
{
    {0, 155, 77, 17, 85, -93, 5, 700},
    {0, 154, 103, -28, 0, -42, 0, 600}
};
