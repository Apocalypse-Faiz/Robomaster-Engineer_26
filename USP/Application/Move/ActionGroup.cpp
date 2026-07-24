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
        if (_Add_onlyOnce == 1) ActionGroup_WithJoint(_tpa1, TakeInOre_RB, 3);
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
// J2/J3/J4/J6 已切换到真实机械角坐标；以下第 2/3/4/6 列相对最初调试值
// 累计增加 6.85°/7.8°/1.1°/-0.4°，因此动作组对应的实际机械姿态保持不变。
float ResetActionNoOre_Joint[2][8] = {
    {0, 96.85, 7.8, 1.1, 0, -0.4, 0, 700}, // 分别是J1，J2，。。。J6,间隔时间
    {-5, 156.85, -62.2, 1.1, 0, -0.4, 0, 700}};

float ResetActionOre_Joint[3][8] = {
    {0, 96.85, 7.8, 1.1, 0, -0.4, 0, 300},
    {0, 116.85, -12.2, 1.1, 0, -0.4, 0, 300},
    {0, 131.85, -37.2, 1.1, 0, -0.4, 0, 300}};

/*机动动作组*/
float MobileNoOreAction_Joint[3][8] = {
    {0, 161.85, 84.8, 18.1, 85, -93.4, 5, 700},
    {0, 146.85, 140.8, -70.9, 0, -0.4, 0, 700},
    {0, 159.85, 152.8, -75.9, 0, -0.4, 0, 700}};

float MobileOreAction_Joint[2][8] =
    {
        {0, 96.85, 97.8, -75.9, 0, -100.4, -150, 600},
        {0, 96.85, 97.8, -75.9, 0, 101.6, 167, 600}};

float MobileTunnelAction_Joint[4][8] = {
    {0, 161.85, 84.8, 18.1, 85, -93.4, 5, 700},
    {0, 136.85, 122.8, -73.9, 0, -0.4, 0, 700},
    {0, 161.85, 84.8, 18.1, 85, -93.4, 5, 700},
    {0, 151.65, 151.4, 16.8, 90, -100.4, 13.6, 500}};

/*取地矿*/
float TakeLandOre_prepare[2][8] = {
    {0, 86.85, -32.2, 1.1, -21, -0.4, 0, 800},
    {-0.966801, 73.85, -30.7765, 3.41819, -28.5342, -0.4, 0, 600}};

float TakeLandOre_ing[1][8] = {
    {-0.966801, 60.6521, -29.9262, 4.19748, -20.9517, -0.4, 0, 500}};

float TakeLandOre_end[1][8] = {
    {0, 141.85, -48.87, 1.1, -30, -0.4, 0, 1000}};

/*招财进宝*/
float TestAction[9][8] = {
    {0, 96.85, 7.8, 1.1, 0, -0.4, 0, 2000},
    {0, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},
    {-60, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},
    {0, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},
    {60, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},
    {0, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},
    {-60, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},   
    {0, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000},
    {60, 41.85, 37.8, 1.1, 30, -0.4, 0, 1000}

};

float ShieldArmor[5][8] = {
    {0, 96.85, 7.8, 1.1, 0, -0.4, 0, 500},
    {0, 96.85, 32.8, 1.1, 0, -0.4, 0, 500},
    {0, 96.85, 7.8, 1.1, 0, -0.4, 0, 500},
    {0, 96.85, -32.2, 1.1, 0, -0.4, 0, 500},
    {0, 96.85, 7.8, 1.1, 0, -0.4, 0, 500}};

/*翻矿动作组*/
float TurnOre_prepare[2][8] = {
    {0, 96.85, 7.8, 1.1, 30, -0.4, 0, 800},
    {0, 55.28512, -31.814136, 1.1, 45.768303, 7.566847, 0, 800}};

float TurnOre_ing[5][8] = {
    {0, 52.110742, -48.731067, 1.1, 45.242409, 7.356055, 0, 400},
    {0, 67.040674, -55.345599, 1.1, 45.242409, 7.356055, 0, 400},
    {0, 92.177759, -40.214038, 1.1, 19.126671, 6.561287, 0, 400},
    {0, 92.727136, -24.116626, 1.1, -27.125877, 6.561681, 0, 400},
    {0, 59.448816, -22.706927, 1.1, -27.569252, 15.354708, 0, 1000}};

float TurnOre_end[1][8] = {
    {0, 126.85, -48.87, 1.1, 0, 9.179893, 0, 700}};

// 取能量单元
float TakeOre1_prepare[2][8] = 
{
    {0, 128.85, 90.8, -75.9, 0, -0.4, 0, 500},
    {0, 107.74, 77.02, -75.9, 0, -12.6, 0, 500}
    // {0, 121.64, -9.27, 1.1, 47.63, -0.4, 500}
};

float TakeOre2_prepare[2][8] = 
{
    {27.03, 111.37, 89.31, -75.9, 57.08, -0.4, 0, 500},
    {15.03, 99.46, 79.31, -75.9, 66.12, -10.94, -9.86, 500}
};

float TakeOre3_prepare[2][8] = 
{
    {18, 120.85, 118.8, -75.9, 66, -8.4, 46, 500},
    {13.25, 97.94, 103.15, -75.9, 65.48, -12.22, 51.09, 500}
};

float TakeOre4_prepare[2][8] = 
{
    {0, 102.85, 119.8, -75.9, 165, -0.4, 15, 800},
    {0, 79.83, 93.1, -75.9, 165, 3.27, 10.65, 500}
};

float TakeOre5_prepare[2][8] = 
{
    {-51, 129.85, 124.8, -75.9, -78, 22.6, -38, 500},
    {-28.85, 100.57, 103.63, -75.9, -107.31, 1.71, -10.43, 500}
};

float TakeOre6_prepare[2][8] = 
{
    {-52, 129.85, 101.8, -75.9, -82, 19.6, 6, 500},
    {-22.53, 105.06, 83.54, -75.9, -97.2, -4.7, 29.12, 500}
};


float TakeOre1_ing[2][8] = 
{
    {0, 96.97, 46.33, -75.9, 0, 8.47, 0, 500},
    {0, 96.97, 46.33, -75.9, 0, -30.4, 0, 500},
    // {0, 109.85, -41.2, 1.1, 60.20, -0.4, 400}
    // {0, 111.94, -36.72, 1.1, 61.66, -0.4, 500}
};

float TakeOre2_ing[2][8] = 
{
    // {39.72, 110.31, -7.83, 52.47, 48.03, 10.47, 400},
    // {46.27, 102.55, -21.18, 56.91, 57.69, 9.59, 400}
    {30.5, 87.08, 59.55, -75.9, 66.16, 2.9, -6.42, 500},
    {30.5, 87.08, 59.55, -75.9, 66.16, -30.4, -6.42, 500}
};

float TakeOre3_ing[2][8] = 
{
    {31.77, 86.82, 98.07, -75.9, 79.52, 9.35, 22.85, 500},
    {31.77, 86.82, 98.07, -75.9, 79.52, -20.4, 22.85, 500}
};

float TakeOre4_ing[2][8] = 
{
    {0, 71.82, 99, -75.9, 157.39, 16.01, 16.21, 500},
    {0, 71.82, 99, -75.9, 157.39, -10.4, 16.21, 500}
};

float TakeOre5_ing[2][8] = 
{
    {-42.56, 90.77,  105.43, -75.9, -131.92, 17.73, 20.76, 500},
    {-42.56, 90.77, 105.43, -75.9, -131.92, -30.4, 20.76, 500}
};

float TakeOre6_ing[2][8] = 
{
    {-40.41, 97.48, 65.75, -75.9, -95.89, 11.95, 22.34, 500},
    {-40.41, 97.48, 65.75, -75.9, -95.89, -30.4, 22.34, 500}
};
        

float TakeOre1_end[2][8] = 
{
    //{0, 105.99, -41.2, 1.1, 5.58, -0.4, 300}
    {0, 140.85, 95.8, -75.9, 0, -32.4, 0, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};

float TakeOre2_end[2][8] = 
{
    {32, 122.85, 102.8, -63.9, 43, -30.4, 0, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};

float TakeOre3_end[2][8] =
{
    {32, 124.85, 121.8, -75.9, 72, -20.4, 27, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};

float TakeOre4_end[2][8] = 
{
    {0, 106.85, 121.8, -64.9, 160, -10.4, 7, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};

float TakeOre5_end[2][8] = 
{
    {-39, 136.85, 119.8, -75.9, -136, -42.4, 0, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};

float TakeOre6_end[2][8] = 
{
    {-35, 134.85, 107.8, -75.9, -50, -20.4, 10, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};

// ============================================================================
// 储矿槽动作组 — 4 槽 (LF=左前, RF=右前, LB=左后, RB=右后)
//   格式: [N][8] = { {J1,J2,J3,J4,J5,J6,J7,time_ms}, ... }
//   TODO: 填入实际关节角度
// ============================================================================

// ── 储存 (Store) ──────────────────────────────────
float StoreOre_LF[2][8] = 
{
    {58.38, 142.02, 141.57, -75.9, 7.93, -17.77, -20.98, 500},
    {46.95, 151.6, 145.8, -45.68, 33.72, -34.21, -41.13, 500}
};
float StoreOre_RF[2][8] = 
{
    {-74.35, 138.85, 138.26, -73.19, -12, -26.59, 24.13, 500},
    {-82.53, 146.19, 154.23, -63.37, -12.94, -28.83, 26.5, 500}
};
float StoreOre_LB[2][8] = 
{
    // {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500},
    {162.27, 81.08, 106.53, -68.8, 7.73, -71, -10.76, 800},
    {161.27, 64.26, 119.93, -75.9, 6.54, -95.8, -4.89, 600}
};
float StoreOre_RB[2][8] = 
{
    // {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500},
    {-172.18, 81.7, 113.8, -75.9, 4.52, -71.9, -2.99, 800},
    {-167.01, 63.68, 118.59, -75.9, -0.73, -84.8, -3.93, 600}
};

// ── 退出储存 (ExitStore) ──────────────────────────
float ExitStoreOre_LF[2][8] = 
{
    {46.96, 161.85, 108.1, -4.94, 26.67, -26.43, -28.56, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};
float ExitStoreOre_RF[2][8] = 
{
    {-80, 161.85, 122, -14.77, -5.8, -28.63, 21.2, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};
float ExitStoreOre_LB[2][8] = 
{
    {151.13, 74.82, 124.61, -51.86, 16.08, -100.4, 0.63, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 1200}
};
float ExitStoreOre_RB[3][8] = 
{
    {-166.68, 75.46, 137.49, -75.72, -2.79, -97.82, -2.04, 500},
    {-83, 117.85, 107.8, -51.9, 0, -70.4, 0, 600},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 600}
};

// ── 取出 (TakeIn) ─────────────────────────────────
float TakeInOre_LF[2][8] = 
{
    {46.96, 161.85, 108.1, -4.94, 26.67, -26.43, -28.56, 500},
    {46.95, 151.6, 145.8, -45.68, 33.72, -34.21, -41.13, 500}
};
float TakeInOre_RF[2][8] = 
{
    {-80, 161.85, 122, -14.77, -5.8, -28.63, 21.2, 500},
    {-82.53, 146.19, 154.23, -63.37, -12.94, -28.83, 26.5, 500}
};
float TakeInOre_LB[2][8] = 
{
    {151.13, 74.82, 124.61, -51.86, 16.08, -100.4, 0.63, 800},
    {157.27, 61.26, 117.93, -75.9, 6.54, -97.8, -4.89, 500}
};
float TakeInOre_RB[3][8] = 
{
    {-83, 117.85, 107.8, -51.9, 0, -70.4, 0, 400},
    {-172.68, 75.46, 132.49, -75.72, -2.79, -97.82, -2.04, 500},
    {-172.01, 63.68, 118.59, -75.9, -0.73, -94.8, -3.93, 500}
};

// ── 取出结束 (TakeIn_end) ─────────────────────────
float TakeinOreLF_end[2][8] = 
{
    {58.38, 142.02, 141.57, -75.9, 7.93, -17.77, -20.98, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};
float TakeinOreRF_end[2][8] = 
{
    {-74.35, 138.85, 138.26, -73.19, -12, -26.59, 24.13, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 500}
};
float TakeinOreLB_end[2][8] = 
{
    {158.83, 81.79, 103.93, -59.99, 8.59, -80.97, -11.56, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 800}
};
float TakeinOreRB_end[2][8] = 
{
    {-168.18, 81.7, 113.8, -75.9, 4.52, -71.9, -2.99, 500},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 800}
};

float ToActionGroup[2][8] = 
{
    {0, 161.85, 84.8, 18.1, 85, -93.4, 5, 700},
    {0, 160.85, 110.8, -26.9, 0, -42.4, 0, 600}
};
