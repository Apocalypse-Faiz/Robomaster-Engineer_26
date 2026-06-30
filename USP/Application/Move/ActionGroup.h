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

    // 储矿槽 — 左前 (LF)
    StoreOre_LF,        // 储存-左前
    ExitStoreOre_LF,    // 退出储存-左前
    TakeInOre_LF,       // 取出-左前
    TakeinOreLF_end,    // 取出结束-左前
    // 储矿槽 — 右前 (RF)
    StoreOre_RF,        // 储存-右前
    ExitStoreOre_RF,    // 退出储存-右前
    TakeInOre_RF,       // 取出-右前
    TakeinOreRF_end,    // 取出结束-右前
    // 储矿槽 — 左后 (LB)
    StoreOre_LB,        // 储存-左后
    ExitStoreOre_LB,    // 退出储存-左后
    TakeInOre_LB,       // 取出-左后
    TakeinOreLB_end,    // 取出结束-左后
    // 储矿槽 — 右后 (RB)
    StoreOre_RB,        // 储存-右后
    ExitStoreOre_RB,    // 退出储存-右后
    TakeInOre_RB,       // 取出-右后
    TakeinOreRB_end,    // 取出结束-右后

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
void ActionGroup_WithJoint(TrajectoryPointArray _tpa1[7], float _a[][8], int _Num);
void ActionGroup_Choose(TrajectoryPointArray _tpa1[7], ActionGroup_enum _action_enum, uint8_t _Add_onlyOnce);
void ActionGroup_Execute(Joint _j[7], TrajectoryPointArray _tpa1[7], int Joint_num, uint8_t &_Add_onlyOnce);

/*关节坐标系下*/
extern float ResetActionNoOre_Joint[2][8];
extern float ResetActionOre_Joint[3][8];

extern float MobileNoOreAction_Joint[3][8];
extern float MobileOreAction_Joint[2][8];
extern float MobileTunnelAction_Joint[4][8];

extern float TakeLandOre_prepare[2][8]; // 取地矿
extern float TakeLandOre_ing[1][8];
extern float TakeLandOre_end[1][8];

extern float TestAction[9][8];  // J1测试动作
extern float ShieldArmor[5][8]; // J3测试动作

extern float TurnOre_prepare[2][8]; // 翻转矿石
extern float TurnOre_ing[5][8];
extern float TurnOre_end[1][8];

// 取能量单元
extern float TakeOre1_prepare[2][8];
extern float TakeOre2_prepare[2][8];
extern float TakeOre3_prepare[2][8];
extern float TakeOre4_prepare[2][8];
extern float TakeOre5_prepare[2][8];
extern float TakeOre6_prepare[2][8];

extern float TakeOre1_ing[2][8];
extern float TakeOre2_ing[2][8];
extern float TakeOre3_ing[2][8];
extern float TakeOre4_ing[2][8];
extern float TakeOre5_ing[2][8];
extern float TakeOre6_ing[2][8];

extern float TakeOre1_end[3][8];
extern float TakeOre2_end[3][8];
extern float TakeOre3_end[3][8];
extern float TakeOre4_end[3][8];
extern float TakeOre5_end[3][8];
extern float TakeOre6_end[3][8];

// 储存能量单元 — 4 储矿槽 (LF=左前, RF=右前, LB=左后, RB=右后)
extern float StoreOre_LF[2][8];
extern float StoreOre_RF[2][8];
extern float StoreOre_LB[2][8];
extern float StoreOre_RB[2][8];
extern float ExitStoreOre_LF[2][8];
extern float ExitStoreOre_RF[2][8];
extern float ExitStoreOre_LB[2][8];
extern float ExitStoreOre_RB[2][8];

// 取肚子内能量单元 — 4 储矿槽
extern float TakeInOre_LF[2][8];
extern float TakeInOre_RF[2][8];
extern float TakeInOre_LB[2][8];
extern float TakeInOre_RB[2][8];
extern float TakeinOreLF_end[2][8];
extern float TakeinOreRF_end[2][8];
extern float TakeinOreLB_end[2][8];
extern float TakeinOreRB_end[2][8];

/*世界坐标系下*/
extern float TempAction_World[4][8];
extern float TakeLandOre_end_World[12][8];        // 取地矿结束
extern float TakeSilverOre_end_World[12][8];      // 取左银矿结束
extern float TakeSilverOre_RightEnd_World[12][8]; // 取右银矿结束
#endif
