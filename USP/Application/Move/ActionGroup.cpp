#include "ActionGroup.h"
#include "internal.h"
/**
 * @brief 关节坐标系的动作组执行函数
 * @param _tpa1 轨迹点组
 * @param _a 输入动作组，默认是六个关节+一个时间
 * @param _Num 动作组的目标数量
 */
void ActionGroup_WithJoint(TrajectoryPointArray _tpa1[6], float _a[][7], int _Num)
{
    // 是否需要加一些判断条件？满足条件后才能执行动作组

    for (uint8_t i = 0; i < 6; i++)
    {
        _tpa1[i].clear(); // 需要清空，不仅是节省空间，更是防止会产生两个不同动作的阶跃（一个结束和一个开始的）
    }

    for (uint8_t i = 0; i < _Num; i++)
    {
        for (uint8_t j = 0; j < 6; j++)
        {
            _tpa1[j].add(_a[i][6], _a[i][j]);
        }
    }
}


/**
 * @brief 动作组的选择函数
 * @param _tpa1 轨迹点组
 * @param _action_enum 动作组枚举
 * @param _Add_onlyOnce 是否只添加一次
 */
void ActionGroup_Choose(TrajectoryPointArray _tpa1[6], ActionGroup_enum _action_enum, uint8_t _Add_onlyOnce)
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
            ActionGroup_WithJoint(_tpa1, MobileNoOreAction_Joint, 2);
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
            ActionGroup_WithJoint(_tpa1, MobileTunnelAction_Joint, 2);
        }
        break;

     /*储存能量单元*/
    case ActionGroup_enum::StoreOre_L: 
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, StoreOre_L, 3);
        }
        break;
    case ActionGroup_enum::StoreOre_R:  
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, StoreOre_R, 3);
        }
        break;
    case ActionGroup_enum::ExitStoreOre_L: 
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, ExitStoreOre_L, 3);
        }
        break;
    case ActionGroup_enum::ExitStoreOre_R: 
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, ExitStoreOre_R, 3);
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
            ActionGroup_WithJoint(_tpa1, TakeOre2_prepare, 3);
        }
        break;    
    case ActionGroup_enum::TakeOre3_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre3_prepare, 3);
        }
        break;
    case ActionGroup_enum::TakeOre4_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre4_prepare, 3);
        }
        break;
    case ActionGroup_enum::TakeOre5_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre5_prepare, 3);
        }
        break;
    case ActionGroup_enum::TakeOre6_prepare:
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeOre6_prepare, 3);
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
            ActionGroup_WithJoint(_tpa1, TakeOre4_end, 4);
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
    case ActionGroup_enum::TakeInOre_R: // 取肚子内 右
        if (_Add_onlyOnce == 1)
        {
             ActionGroup_WithJoint(_tpa1, TakeInOre_R, 3);
        }
        break;
    case ActionGroup_enum::TakeInOre_L: // 取肚子内 左
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeInOre_L, 3);
        }
        break;
    case ActionGroup_enum::TakeinOreRight_end: 
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeinOreRight_end, 3);
        }
        break;
    case ActionGroup_enum::TakeinOreLeft_end: 
        if (_Add_onlyOnce == 1)
        {
            ActionGroup_WithJoint(_tpa1, TakeinOreLeft_end, 3);
        }
        break;

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
void ActionGroup_Execute(Joint _j[6], TrajectoryPointArray _tpa1[6], int Joint_num, uint8_t &_Add_onlyOnce)
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

    for (int i = 0; i < Joint_num; i++)
    {
        _j[i].takeOver(HAL_GetTick()); // 执行放在这里执行的一个好处就是下面对于储矿的各个延时停顿能够即时的反映到机械臂上，而不是再做一个传递到其他任务上去
    }
}

/*关节坐标系下*/
//  第一个【】内是多少套关节目标，后一个【】里的6是固定的，代表6个关节 全是角度制
// 多少套关节目标最好小于轨迹规划点组的数量

/*复位动作组*/
float ResetActionNoOre_Joint[2][7] = {
    {0, 90, 0, 0, 0, 0, 700}, // 分别是J1，J2，。。。J6,间隔时间
    {-5, 150, -70, 0, 0, 0, 700}};

float ResetActionOre_Joint[3][7] = {
    {0, 90, 0, 0, 0, 0, 300},
    {0, 110, -20, 0, 0, 0, 300},
    {0, 125, -45, 0, 0, 0, 300}};

/*机动动作组*/
float MobileNoOreAction_Joint[2][7] = {
    {0, 120, 35, 0, 0, 0, 800},
    {0, 150, 64, 0, 0, 0, 800}};

float MobileOreAction_Joint[2][7] =
    {
        {0, 90, -10, 0, 0, 0, 800},
        {0, 135, -56.67, 0, -30, 0, 800}};

float MobileTunnelAction_Joint[2][7] = {
    {0, 120, 35, 0, 0, 0, 800},
    {0, 131, 44, 0, 0, 0, 800}};

/*取地矿*/
float TakeLandOre_prepare[2][7] = {
    {0, 80, -40, 0, -21, 0, 800},
    {-0.966801, 67, -38.5765, 2.31819, -28.5342, 0, 600}};

float TakeLandOre_ing[1][7] = {
    {-0.966801, 53.8021, -37.7262, 3.09748, -20.9517, 0, 500}};

float TakeLandOre_end[1][7] = {
    {0, 135, -56.67, 0, -30, 0, 1000}};

/*招财进宝*/
float TestAction[9][7] = {
    {0, 90, 0, 0, 0, 0, 2000},
    {0, 35, 30, 0, 30, 0, 1000},
    {-60, 35, 30, 0, 30, 0, 1000},
    {0, 35, 30, 0, 30, 0, 1000},
    {60, 35, 30, 0, 30, 0, 1000},
    {0, 35, 30, 0, 30, 0, 1000},
    {-60, 35, 30, 0, 30, 0, 1000},
    {0, 35, 30, 0, 30, 0, 1000},
    {60, 35, 30, 0, 30, 0, 1000}

};

float ShieldArmor[5][7] = {
    {0, 90, 0, 0, 0, 0, 500},
    {0, 90, 25, 0, 0, 0, 500},
    {0, 90, 0, 0, 0, 0, 500},
    {0, 90, -40, 0, 0, 0, 500},
    {0, 90, 0, 0, 0, 0, 500}};

/*翻矿动作组*/
float TurnOre_prepare[2][7] = {
    {0, 90, 0, 0, 30, 0, 800},
    {0, 48.435120, -39.614136, 0, 45.768303, 7.966847, 800}};

float TurnOre_ing[5][7] = {
    {0, 45.260742, -56.531067, 0, 45.242409, 7.756055, 400},
    {0, 60.190674, -63.145599, 0, 45.242409, 7.756055, 400},
    {0, 85.327759, -48.014038, 0, 19.126671, 6.961287, 400},
    {0, 85.877136, -31.916626, 0, -27.125877, 6.961681, 400},
    {0, 52.598816, -30.506927, 0, -27.569252, 15.754708, 1000}};

float TurnOre_end[1][7] = {
    {0, 120, -56.67, 0, 0, 9.579893, 700}};

// 取能量单元
float TakeOre1_prepare[2][7] = 
{
    {0, 121.3, 14.7, 0, 27.5, 0, 500},
    {0, 86.1, -10.98, 0, 18.15, 0, 500}
    // {0, 114.79, -17.07, 0, 47.63, 0, 500}
};

float TakeOre2_prepare[3][7] = 
{
    {8.33, 138.33, 28.87, 13.63, 10.09, 5.72, 500},
    {16.82, 120.33, 6.52, 27.26, 20.26, 11.43, 500},
    {25.07, 103.49, -11.39, 40.89, 30.25, 24.15, 500}
};

float TakeOre3_prepare[3][7] = 
{
    {13.63, 138.33, 28.87, 28.75, 15.09, 12.72, 500},
    {27.39, 126.5, 22.53, 56.94, 30.20, 24.21, 500},
    {41.99, 113.56, 15.70, 85.56, 44.40, 29.10, 500}
};

float TakeOre4_prepare[3][7] = 
{
    {0, 136.72, 41.85, 30, 0, 30, 500},
    {0, 124.45, 34.45, 60, 0, 60, 200},
    {0, 114.43, 26.06, 90, 0, 90, 200}
};

float TakeOre5_prepare[3][7] = 
{
    {-13.63, 138.33, 28.87, -28.75, 15.09, -12.72, 500},
    {-27.39, 126.5, 22.53, -56.94, 30.20, -24.21, 500},
    {-41.99, 113.56, 15.70, -70.00, 51.40, -60.13, 500}
};

float TakeOre6_prepare[3][7] = 
{
    {-8.33, 140.33, 29.59, -13.63, 10.09, -5.72, 500},
    {-16.82, 120.33, 6.52, -27.26, 20.26, -11.43, 500},
    {-25.07, 103.49, -11.39, -40.89, 30.25, -24.15, 500}
};


float TakeOre1_ing[2][7] = 
{
    {0, 81.38, -45.97, 0, 42.86, 0, 500},
    {0, 81.38, -45.97, 0, 0, 0, 500},
    // {0, 103, -49, 0, 60.20, 0, 400}
    // {0, 105.09, -44.52, 0, 61.66, 0, 500}
};

float TakeOre2_ing[2][7] = 
{
    // {39.72, 103.46, -15.63, 51.37, 48.03, 10.87, 400},
    // {46.27, 95.70, -28.98, 55.81, 57.69, 9.99, 400}
    {43.91, 93.16, -33.2, 58.39, 54.67, 14.24, 500},
    {56.33, 86.24, -49., 61.59, 65.41, 10.84, 500}
};

float TakeOre3_ing[2][7] = 
{
    {53.76, 105.06, 13.53, 87.21, 60.47, 25.73, 400},
    // {61.76, 94.23, 6.95, 94.79, 66.27, 21.73, 400},
    // {53.76, 100.23, 8.68, 89.36, 57.5, 23.61, 400},
    {66.42, 100.29, 14.07, 88.50, 72.24, 18.48, 400}
};

float TakeOre4_ing[2][7] = 
{
    {16.13, 110.81, 35.43, 123.37, 44.01, 53.70, 500},
    {19.16, 101.41, 41.66, 121.97, 62.99, 41.84, 400}
};

float TakeOre5_ing[2][7] = 
{
    {-54.91, 106.27, 15.95, -77.32, 62.88, -39.87, 400},
    // {-66.42, 100.29, 14.07, -88.50, 72.24, -18.48, 400}
    {-57.63, 97.16, 15.95, -84.64, 67.40, -26.41, 400}
};

float TakeOre6_ing[2][7] = 
{
    {-43.91, 93.16, -33.2, -58.39, 54.67, -14.24, 500},
    {-56.33, 86.24, -49., -61.59, 65.41, -10.84, 500}
};
        

float TakeOre1_end[3][7] = 
{
    //{0, 99.14, -49, 0, 5.58, 0, 300}
    {0, 117.6, -9.64, 0, 28.4, 0, 500 },
    {0, 126, 17, 0, 40, 0, 400},
    {0, 134.35, 42.62, 0, 55, 0, 400}
};

float TakeOre2_end[3][7] = 
{
    {36.03, 121.42, -7.41, 34.26, 43.67, 6.34, 500},
    {18.65, 138.33, 28.87, 0, 49.11, 0, 500},
    {0, 134.35, 42.62, 0, 55, 0, 600}
};

float TakeOre3_end[3][7] = 
{
    {53.15, 111.3, 9.83, 27.11, 44.26, 9.46, 500},
    {24.10, 138.2, 28.87, 13.45, 25.3, 0, 500},
    {0, 155.38, 41.46, 0, 38, 0, 500}
};

float TakeOre4_end[3][7] = 
{
    {9.58, 119.4, 41.46, 81.31, 53.05, 27.90, 500},
    {0, 137.39, 41.46, 40.66, 45.52, 14.03, 500},
    {0, 155.38, 41.46, 0, 38, 0, 500}
};

float TakeOre5_end[3][7] = 
{
    {-53.15, 111.3, 9.83, -27.11, 44.26, -9.46, 500},
    {-24.10, 138.2, 28.87, -13.45, 25.3, 0, 500},
    {0, 155.38, 41.46, 0, 38, 0, 500}
};

float TakeOre6_end[3][7] = 
{
    {-36.03, 121.42, -7.41, -34.26, 43.67, -6.34, 500},
    {-18.65, 138.33, 28.87, 0, 49.11, 0, 500},
    {0, 155.38, 41.46, 0, 38, 0, 500}
};

/*储能量单元*/
float StoreOre_R[3][7] = {
    {-120, 97, 36, 0, -8, 0, 600},
    {-169.8, 69.27, 21.65, 0, -20.29, 0, 800},
    {-169.8, 49.45, 24.52, 0, -47.44, 0, 400}
};

float StoreOre_L[3][7] = {
    {120, 97, 36, 0, -8, 0, 600},
    {158.71, 67.12, 23.36, 0, -38.21, 0, 700},
    {158.71, 49.74, 23.33, 0, -42.63, 0, 400}};

float ExitStoreOre_L[3][7] = {
    {157.41, 73.71, 43.73, 0, -40.92, 0, 400},
    {120, 97, 36, 0, -8, 0, 600},
    {0, 134.35, 42.62, 0, 55, 0, 600}};

float ExitStoreOre_R[3][7] = {
    {-169.41, 55.91, 42.43, 0, -62, 0, 400},
    {-120, 97, 36, 0, -8, 0, 600},
    {0, 134.35, 42.62, 0, 55, 0, 600}};

// 取肚子内能量单元
float TakeInOre_L[3][7] = {
    {120, 97, 36, 0, -8, 0, 600},
    {157.41, 73.71, 43.73, 0, -40.92, 0, 600},
    {158.71, 49.74, 23.33, 0, -42.63, 0, 400}};

float TakeInOre_R[3][7] = {
    {-120, 97, 36, 0, -8, 0, 600},
    {-169.41, 55.91, 42.43, 0, -62, 0, 600},
    {-169.8, 49.45, 24.52, 0, -50.44, 0, 400}
};

float TakeinOreRight_end[3][7] = {
    {-169.8, 70.11, 21.15, 0, -30.64, 0, 600},
    {-120, 97, 36, 0, -8, 0, 600},
    {0, 134.35, 42.62, 0, 55, 0, 600}};

float TakeinOreLeft_end[3][7] = {
    {158.71, 67.12, 20.78, 0, -37.62, 0, 500},
    {120, 97, 36, 0, -8, 0, 600},
    {0, 134.35, 42.62, 0, 55, 0, 600}

};
