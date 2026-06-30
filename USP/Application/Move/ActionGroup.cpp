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
    case ActionGroup_enum::StoreOre_LF: case ActionGroup_enum::StoreOre_RF:
    case ActionGroup_enum::StoreOre_LB: case ActionGroup_enum::StoreOre_RB:
        if (_Add_onlyOnce == 1)
        {
            // 根据枚举值选取对应数组 (利用枚举连续排列: LF,RF,LB,RB)
            static float (*const store_tables[4])[8] = {
                StoreOre_LF, StoreOre_RF, StoreOre_LB, StoreOre_RB
            };
            int idx = (int)_action_enum - (int)ActionGroup_enum::StoreOre_LF;
            ActionGroup_WithJoint(_tpa1, store_tables[idx], 2);
        }
        break;

    /*退出储存 (ExitStore) */
    case ActionGroup_enum::ExitStoreOre_LF: case ActionGroup_enum::ExitStoreOre_RF:
    case ActionGroup_enum::ExitStoreOre_LB: case ActionGroup_enum::ExitStoreOre_RB:
        if (_Add_onlyOnce == 1)
        {
            static float (*const exit_tables[4])[8] = {
                ExitStoreOre_LF, ExitStoreOre_RF, ExitStoreOre_LB, ExitStoreOre_RB
            };
            int idx = (int)_action_enum - (int)ActionGroup_enum::ExitStoreOre_LF;
            ActionGroup_WithJoint(_tpa1, exit_tables[idx], 2);
        }
        break;

    /*取出 (TakeIn) */
    case ActionGroup_enum::TakeInOre_LF: case ActionGroup_enum::TakeInOre_RF:
    case ActionGroup_enum::TakeInOre_LB: case ActionGroup_enum::TakeInOre_RB:
        if (_Add_onlyOnce == 1)
        {
            static float (*const takein_tables[4])[8] = {
                TakeInOre_LF, TakeInOre_RF, TakeInOre_LB, TakeInOre_RB
            };
            int idx = (int)_action_enum - (int)ActionGroup_enum::TakeInOre_LF;
            ActionGroup_WithJoint(_tpa1, takein_tables[idx], 2);
        }
        break;

    /*取出结束 (TakeIn_end) */
    case ActionGroup_enum::TakeinOreLF_end: case ActionGroup_enum::TakeinOreRF_end:
    case ActionGroup_enum::TakeinOreLB_end: case ActionGroup_enum::TakeinOreRB_end:
        if (_Add_onlyOnce == 1)
        {
            static float (*const end_tables[4])[8] = {
                TakeinOreLF_end, TakeinOreRF_end, TakeinOreLB_end, TakeinOreRB_end
            };
            int idx = (int)_action_enum - (int)ActionGroup_enum::TakeinOreLF_end;
            ActionGroup_WithJoint(_tpa1, end_tables[idx], 2);
        }
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
            ActionGroup_WithJoint(_tpa1, TakeOre1_end, 3);
        }
        break;
    case ActionGroup_enum::TakeOre2_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre2_end, 3);
        }
        break;
    case ActionGroup_enum::TakeOre3_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre3_end, 3);
        }
        break;
    case ActionGroup_enum::TakeOre4_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre4_end, 3);
        }
        break;
    case ActionGroup_enum::TakeOre5_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre5_end, 3);
        }
        break;
    case ActionGroup_enum::TakeOre6_end:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre6_end, 3);
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
    {0, 143.5, 97, -1, 76, -100, -5, 700},
    {0, 140, 133, -72, 0, 0, 0, 700},
    {0, 153, 145, -77, 0, 0, 0, 700}};

float MobileOreAction_Joint[2][8] =
    {
        {0, 90, -10, 0, 0, 0, 0, 800},
        {0, 135, -56.67, 0, -30, 0, 0, 800}};

float MobileTunnelAction_Joint[4][8] = {
    {0, 143.5, 97, -1, 76, -100, -5, 700},
    {0, 130, 115, -75, 0, 0, 0, 700},
    {0, 143.5, 97, -1, 76, -100, -5, 700},
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
    {0, 121.3, 14.7, 0, 14.2, 0, 0, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
    // {0, 114.79, -17.07, 0, 47.63, 0, 500}
};

float TakeOre2_prepare[2][8] = 
{
    {32.22, 106.21, 11.64, 70.46, 27.01, -13.76, 0, 500},
    {18.18, 79.36, -10.63, 61.8, 13.64, -5.74, 0, 500}
};

float TakeOre3_prepare[2][8] = 
{
    {27.13, 96.61, 32.89, 103.43, 33.92, 7.19, 0, 500},
    {16.59, 72.68, 1.07, 122.58, 34.15, -12.03, 0, 500}
};

float TakeOre4_prepare[2][8] = 
{
    {0, 59.91, 12.41, 174.07, 41.24, 0, 0, 800},
    {0, 57.06, -7.03, 179.06, 30.94, 0, 0, 500}
};

float TakeOre5_prepare[2][8] = 
{
    {-27.13, 96.61, 32.89, -103.43, 33.92, -7.19, 0, 500},
    {-14.21, 72.10, 3.00, -119.75, 35.22, 0, 0, 500}
};

float TakeOre6_prepare[2][8] = 
{
    {-32.22, 106.21, 11.64, -70.46, 27.01, 13.76, 0, 500},
    {-23.75, 85.33, -8.18, -64.69, 26.92, 4.78, 0, 500}
};


float TakeOre1_ing[2][8] = 
{
    {0, 81.38, -45.97, 0, 29.56, 0, 0, 500},
    {0, 81.38, -45.97, 0, 0, 0, 0, 500},
    // {0, 103, -49, 0, 60.20, 0, 400}
    // {0, 105.09, -44.52, 0, 61.66, 0, 500}
};

float TakeOre2_ing[2][8] = 
{
    // {39.72, 103.46, -15.63, 51.37, 48.03, 10.87, 400},
    // {46.27, 95.70, -28.98, 55.81, 57.69, 9.99, 400}
    {36.61, 67.16, -37.68, 79.34, 33.66, -18.97, 0, 500},
    {36.61, 67.16, -37.68, 79.34, 0, -18.97, 0, 500}
};

float TakeOre3_ing[2][8] = 
{
    {40.83, 50.59, -11.33, 108.52, 36.33, -12.25, 0, 500},
    // {61.76, 94.23, 6.95, 94.79, 66.27, 21.73, 400},
    // {53.76, 100.23, 8.68, 89.36, 57.5, 23.61, 400},
    {40.83, 50.59, -11.33, 108.52, 0, -12.25, 0, 500}
};

float TakeOre4_ing[2][8] = 
{
    {0, 26.58, -20.98, 175.07, 46.81, 0, 0, 500},
    {0, 26.58, -20.98, 175.07, 0, 0, 0, 500}
};

float TakeOre5_ing[2][8] = 
{
    {-40.83, 50.59, -11.33, -108.52, 36.33, 12.25, 0, 500},
    {-40.83, 50.59, -11.33, -108.52, 0, 12.25, 0, 500}
};

float TakeOre6_ing[2][8] = 
{
    {-36.61, 67.16, -37.68, -79.34, 33.66, 18.97, 0, 500},
    {-36.61, 67.16, -37.68, -79.34, 0, 18.97, 0, 500}
};
        

float TakeOre1_end[3][8] = 
{
    //{0, 99.14, -49, 0, 5.58, 0, 300}
    {0, 117.6, -9.64, 0, 15.1, 0, 500, 500 },
    {0, 126, 17, 0, 26.7, 0, 400, 500},
    {0, 134.35, 42.62, 0, 41.7, 0, 400, 500}
};

float TakeOre2_end[3][8] = 
{
    {35.44, 105.4, 6.91, 65.76, -9.04, 2.52, 500, 500},
    {17.7, 120, 23, 53, -13.3, 28, 500, 500},
    {0, 134.35, 42.62, 0, 41.7, 0, 600, 600}
};

float TakeOre3_end[3][8] =
{
    {35.76, 88.37, 24.61, 63.76, -13.3, 3.43, 500, 500},
    {17, 111, 28.87, 32, 14, 0, 500, 500},
    {0, 134.35, 42.62, 0, 41.7, 0, 500, 500}
};

float TakeOre4_end[3][8] = 
{
    {0, 68.94, 18.11, 118, -9.3, 0, 500, 500},
    {0, 101, 30, 59, 7.7, 0, 500, 500},
    {0, 134.35, 42.62, 0, 41.7, 0, 500, 500}
};

float TakeOre5_end[3][8] = 
{
    {-35.76, 88.37, 24.61, -63.76, -13.3, -3.43, 500, 500},
    {-17, 111, 28.87, -32, 14, 0, 500, 500},
    {0, 134.35, 42.62, 0, 41.7, 0, 500, 500}
};

float TakeOre6_end[3][8] = 
{
    {-35.44, 105.4, 6.91,-65.76, -9.34, -2.52, 500, 500},
    {-17.7, 120, 23, -53, -13.3, -28, 500, 500},
    {0, 134.35, 42.62, 0, 41.7, 0, 600, 600}
};

// ============================================================================
// 储矿槽动作组 — 4 槽 (LF=左前, RF=右前, LB=左后, RB=右后)
//   格式: [N][8] = { {J1,J2,J3,J4,J5,J6,J7,time_ms}, ... }
//   TODO: 填入实际关节角度
// ============================================================================

// ── 储存 (Store) ──────────────────────────────────
float StoreOre_LF[2][8] = 
{
    {55.41, 133.72, 134.77, -77, 12.26, -16.37, -27.55, 500},
    {46.95, 144.75, 138, -46.78, 33.72, -33.81, -41.13, 500}
};
float StoreOre_RF[2][8] = 
{
    {-55.41, 133.72, 134.77, -77, -12.26, -16.37, 27.55, 500},
    {-46.95, 144.75, 138, -46.78, -33.72, -33.81, 41.13, 500}
};
float StoreOre_LB[2][8] = {0};
float StoreOre_RB[2][8] = {0};

// ── 退出储存 (ExitStore) ──────────────────────────
float ExitStoreOre_LF[2][8] = 
{
    {46.96, 155, 103.3, -6.04, 26.67, -26.03, -28.56, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float ExitStoreOre_RF[2][8] = 
{
    {-46.96, 155, 103.3, -6.04, -26.67, -26.03, 28.56, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float ExitStoreOre_LB[2][8] = {0};
float ExitStoreOre_RB[2][8] = {0};

// ── 取出 (TakeIn) ─────────────────────────────────
float TakeInOre_LF[2][8] = 
{
    {46.96, 155, 103.3, -6.04, 26.67, -26.03, -28.56, 500},
    {46.95, 144.75, 138, -46.78, 33.72, -33.81, -41.13, 500}
};
float TakeInOre_RF[2][8] = 
{
    {-46.96, 155, 103.3, -6.04, -26.67, -26.03, 28.56, 500},
    {-46.95, 144.75, 138, -46.78, -33.72, -33.81, 41.13, 500}
};
float TakeInOre_LB[2][8] = {0};
float TakeInOre_RB[2][8] = {0};

// ── 取出结束 (TakeIn_end) ─────────────────────────
float TakeinOreLF_end[2][8] = 
{
    {55.41, 133.72, 134.77, -77, 12.26, -16.37, -27.55, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float TakeinOreRF_end[2][8] = 
{
    {-55.41, 133.72, 134.77, -77, -12.26, -16.37, 27.55, 500},
    {0, 154, 103, -28, 0, -42, 0, 500}
};
float TakeinOreLB_end[2][8] = {0};
float TakeinOreRB_end[2][8] = {0};
