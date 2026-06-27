#include "MobileState.h"
#include "Keyboard.h"
#include "ActionGroup.h"

void Mobile_enterAction()
{
    g_top_state_id = TopStateId::Mobile;

    for (uint8_t i = 0; i < 7; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }
}

void Mobile_action(Event_t **pEvent)
{
    // 云台升降电机降到下限位 0° (每周期步进, 不跳变)
    if (gimbal_lift_init_done)
        Gimbal_Lift_StepToGoal(0.0f, 20.0f); // 降到 0° (下限位), 步长 0.5°

    static ActionGroup_enum action_enum = ActionGroup_enum::NuLLAction;
    static uint8_t Add_onlyOnce = 0;
    static uint8_t mobile_state = 0;
    WS_indicator[0] = WS_Blue;

    if ((DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED) || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
    {

        /*暂停真正执行的动作组*/
        if ((DR16.GetS1() == SW_DOWN && DR16.GetS2() == SW_UP) || VT13.GetPause())
        {
            for (uint8_t i = 0; i < 7; i++)
            {
                joint[i].stopTrajectoryPlanner();
            }
        }

        // 机动状态下，可以为前4个不需要复位的电机移动到一个合适的位置
        if ((DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)) && Add_onlyOnce == 0)
        {
            action_enum = ActionGroup_enum::Mobile_NoOre;
            Add_onlyOnce = 1;
            mobile_state = 1; // 微抬视野状态
            // UI_Mode = 1;
        }
        // else if ((DR16.IsKeyPress(DR16_KEY_C) || VT13.IsKeyPress(VT13_KEY_C)) && Add_onlyOnce == 0)
        // {
        //     action_enum = ActionGroup_enum::Mobile_Ore;
        //     Add_onlyOnce = 1;
        //     mobile_state = 2; // 有矿机动状态
        //     UI_Mode = 1;
        // }
        else if ((DR16.IsKeyPress(DR16_KEY_Z) || VT13.IsKeyPress(VT13_KEY_Z)) && Add_onlyOnce == 0)
        {
            action_enum = ActionGroup_enum::Mobile_Tunnel;
            Add_onlyOnce = 1;
            mobile_state = 3; // 折叠压缩状态
            // UI_Mode = 1;
        }

        /*选择动作组*/
        ActionGroup_Choose(joint_trajectory_point_array, action_enum, Add_onlyOnce);

        /* 动作组执行 */
        ActionGroup_Execute(joint, joint_trajectory_point_array, 7, Add_onlyOnce);
    }

    for (int i = 0; i < 6; i++)  // 硬件仅6个灯珠，对应motor[0]~motor[5]
    {
        if (motor[i].getStatus() != MotorStatus::ready)
        {
            WS_indicator[i] = WS_Red;
        }
        else if (i == 0)
        {
            if (toBottomdata.ChassisState == 1)
            {
                WS_indicator[i] = WS_Green;
            }
            else if (toBottomdata.ChassisState == 3)
            {
                WS_indicator[i] = WS_Cyan;
            }
            else if (toBottomdata.ChassisState == 4)
            {
                WS_indicator[i] = WS_Purple;
            }
            else if (toBottomdata.ChassisState == 0)
            {
                WS_indicator[i] = WS_Yellow;
            }
        }
        else
        {
            WS_indicator[i] = WS_Blue;
        }
    }

    /* 底盘移动 */
    if(VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if(VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_MoveCtrl(toBottomdata);
        }
        else if(!VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_KeyBoard_MoveCtrl(toBottomdata);
        }

    }
    if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == 2)
        {
            DR16_MoveCtrl(toBottomdata);
        }
        else
        {
            KeyBoard_MoveCtrl(toBottomdata);
        }

        // 使用动作组开启轨迹规划器后手动控制关节会被覆盖
        // DR16_Manipulater_JointCtrl();
        Keyboard_Grippers();
        // Keyboard_Store(toBottomdata);

    }
}

void Mobile_exitAction()
{
    for (uint8_t i = 0; i < 7; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }

    // UI_Mode = 0;
}