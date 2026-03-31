#ifndef __ACTIONGROUP_H__
#define __ACTIONGROUP_H__

#include "stdint.h"
#include "Trajectory.h"
#include "Joint.h"

enum class ActionGroup_enum
{
    NuLLAction = 0, // 空动作组
    ResetNoOre,     // 复位收缩状态
    ResetOre,       // 复位展开状态

    Mobile_NoOre,  // 机动状态 - 没储矿
    Mobile_Ore,    // 机动状态 - 储矿
    Mobile_Tunnel, // 机动状态 - 隧道

    StoreOre_L,     // 右储存
    StoreOre_R,     // 左储存
    ExitStoreOre_L, // 退出右储存
    ExitStoreOre_R, // 退出左储存
    TakeInOre_L,    // 取自身储存的 右
    TakeInOre_R,    // 取自身储存的 左
    TakeinOreRight_end,
    TakeinOreLeft_end,

    TakeOre1_prepare,   // 准备
    TakeOre2_prepare,   // 准备
    TakeOre3_prepare,   // 准备
    TakeOre4_prepare,   // 准备
    TakeOre5_prepare,   // 准备
    TakeOre6_prepare,   // 准备

    TakeOre1_ing,   // 取矿进行
    TakeOre2_ing,   // 取矿进行
    TakeOre3_ing,   // 取矿进行
    TakeOre4_ing,   // 取矿进行
    TakeOre5_ing,   // 取矿进行
    TakeOre6_ing,   // 取矿进行

    TakeOre1_end,   // 取矿结束
    TakeOre2_end,   // 取矿结束
    TakeOre3_end,   // 取矿结束
    TakeOre4_end,   // 取矿结束
    TakeOre5_end,   // 取矿结束
    TakeOre6_end,   // 取矿结束
    
    TakeLandOre_prepare, // 准备取地矿
    TakeLandOre_ing,     // 取地矿进行
    TakeLandOre_end, // 取地矿结束
    Stop,            // 紧急停止执行动作

    ExchangeRotate

};

/**
 * @note 加完关节的目标后，其实可以再加入世界的坐标
 */
void ActionGroup_WithJoint(TrajectoryPointArray _tpa1[6], float _a[][7], int _Num);
void ActionGroup_Choose(TrajectoryPointArray _tpa1[6], ActionGroup_enum _action_enum, uint8_t _Add_onlyOnce);
void ActionGroup_Execute(Joint _j[6], TrajectoryPointArray _tpa1[6], int Joint_num, uint8_t &_Add_onlyOnce);

/*关节坐标系下*/
extern float ResetActionNoOre_Joint[2][7];
extern float ResetActionOre_Joint[3][7];

extern float MobileNoOreAction_Joint[2][7];
extern float MobileOreAction_Joint[2][7];
extern float MobileTunnelAction_Joint[2][7];

extern float TakeLandOre_prepare[2][7]; // 取地矿
extern float TakeLandOre_ing[1][7];
extern float TakeLandOre_end[1][7];

extern float TestAction[9][7];  // J1测试动作
extern float ShieldArmor[5][7]; // J3测试动作

extern float TurnOre_prepare[2][7]; // 翻转矿石
extern float TurnOre_ing[5][7];
extern float TurnOre_end[1][7];

// 取能量单元
extern float TakeOre1_prepare[3][7];
extern float TakeOre2_prepare[3][7];
extern float TakeOre3_prepare[3][7];
extern float TakeOre4_prepare[3][7];
extern float TakeOre5_prepare[3][7];
extern float TakeOre6_prepare[3][7];

extern float TakeOre1_ing[2][7];
extern float TakeOre2_ing[2][7];
extern float TakeOre3_ing[2][7];
extern float TakeOre4_ing[2][7];
extern float TakeOre5_ing[2][7];
extern float TakeOre6_ing[2][7];

extern float TakeOre1_end[4][7];
extern float TakeOre2_end[3][7];
extern float TakeOre3_end[3][7];
extern float TakeOre4_end[3][7];
extern float TakeOre5_end[3][7];
extern float TakeOre6_end[3][7];

// 储存能量单元
extern float StoreOre_R[4][7];
extern float StoreOre_L[3][7];
extern float ExitStoreOre_L[4][7];
extern float ExitStoreOre_R[4][7];

// 取肚子内能量单元
extern float TakeInOre_L[3][7];
extern float TakeInOre_R[3][7];
extern float TakeinOreRight_end[4][7];
extern float TakeinOreLeft_end[4][7];

/*世界坐标系下*/
extern float TempAction_World[4][7];
extern float TakeLandOre_end_World[12][7];        // 取地矿结束
extern float TakeSilverOre_end_World[12][7];      // 取左银矿结束
extern float TakeSilverOre_RightEnd_World[12][7]; // 取右银矿结束
#endif
