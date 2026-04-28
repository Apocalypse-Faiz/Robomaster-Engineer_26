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
    {0, 149.27, 47.59, 0, 63, 0, 800}};

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

float TakeOre2_prepare[2][7] = 
{
    {32.22, 106.21, 11.64, 70.46, 40.31, -13.76, 500},
    {18.18, 79.36, -10.63, 61.8, 26.94, -5.74, 500}
};

float TakeOre3_prepare[2][7] = 
{
    {27.13, 96.61, 32.89, 103.43, 47.22, 7.19, 500},
    {14.21, 70.72, 4.46, 119.75, 33.52, -11.49, 500}
};

float TakeOre4_prepare[2][7] = 
{
    {0, 59.91, 12.41, 174.07, 54.54, 0, 800},
    {0, 44.42, -19.62, 174.07, 44.10, 0, 500}
};

float TakeOre5_prepare[2][7] = 
{
    {-27.13, 96.61, 32.89, -103.43, 47.22, -7.19, 500},
    {-14.21, 70.72, 4.46, -119.75, 33.52, 11.49, 500}
};

float TakeOre6_prepare[2][7] = 
{
    {-32.22, 106.21, 11.64, -70.46, 40.31, 13.76, 500},
    {-18.18, 79.36, -10.63, -61.8, 26.94, -9.35, 500}
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
    {36.61, 67.16, -37.68, 79.34, 46.96, -18.97, 500},
    {36.61, 67.16, -37.68, 79.34, 0, -18.97, 500}
};

float TakeOre3_ing[2][7] = 
{
    {40.83, 50.59, -11.33, 108.52, 49.63, -12.25, 500},
    // {61.76, 94.23, 6.95, 94.79, 66.27, 21.73, 400},
    // {53.76, 100.23, 8.68, 89.36, 57.5, 23.61, 400},
    {40.83, 50.59, -11.33, 108.52, 0, -12.25, 500}
};

float TakeOre4_ing[2][7] = 
{
    {0, 26.58, -20.98, 175.07, 60.11, 0, 500},
    {0, 26.58, -20.98, 175.07, 0, 0, 500}
};

float TakeOre5_ing[2][7] = 
{
    {-40.83, 50.59, -11.33, -108.52, 49.63, 12.25, 500},
    {-40.83, 50.59, -11.33, -108.52, 0, 12.25, 500}
};

float TakeOre6_ing[2][7] = 
{
    {-36.61, 67.16, -37.68, -79.34, 46.96, 18.97, 500},
    {-36.61, 67.16, -37.68, -79.34, 0, 18.97, 500}
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
    {35.44, 105.4, 6.91, 65.76, 3.96, 2.52, 500},
    {17.7, 120, 23, 53, 0, 28, 500},
    {0, 134.35, 42.62, 0, 55, 0, 600}
};

float TakeOre3_end[3][7] = 
{
    {35.76, 88.37, 24.61, 63.76, 0, 3.43, 500},
    {17, 111, 28.87, 32, 27.3, 0, 500},
    {0, 134.35, 42.62, 0, 55, 0, 500}
};

float TakeOre4_end[3][7] = 
{
    {0, 68.94, 18.11, 118, 17, 0, 500},
    {0, 101, 30, 59, 34, 0, 500},
    {0, 134.35, 42.62, 0, 55, 0, 500}
};

float TakeOre5_end[3][7] = 
{
    {-35.76, 88.37, 24.61, -63.76, 0, -3.43, 500},
    {-17, 111, 28.87, -32, 27.3, 0, 500},
    {0, 134.35, 42.62, 0, 55, 0, 500}
};

float TakeOre6_end[3][7] = 
{
    {-35.44, 105.4, 6.91,-65.76, 3.96, -2.52, 500},
    {-17.7, 120, 23, -53, 0, -28, 500},
    {0, 134.35, 42.62, 0, 55, 0, 600}
};

/*储能量单元*/
float StoreOre_R[3][7] = {
    {-120, 97, 36, 0, -8, 0, 600},
    {-169.8, 73.86, 18.23, 0, -11.6, -14.84, 800},
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
