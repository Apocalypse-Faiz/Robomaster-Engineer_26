#include "EmergencyState.h"
#include "internal.h"
#include "KeyBoard.h"
#include "ResetState.h"

static bool emergencyTorqueResetActive = false;
static bool emergencyTorqueResetDone = false;
static TopStateId emergency_entry_state = TopStateId::None;
static bool emergency_use_energy_unit_move_ctrl = false;

void Emergency_enterAction()
{    
    emergency_entry_state = g_top_state_id;
    g_top_state_id = TopStateId::Emergency;
    emergency_use_energy_unit_move_ctrl = (emergency_entry_state == TopStateId::GetEnergyUnit);

    emergencyTorqueResetActive = false;
    emergencyTorqueResetDone = false;
    Robot_Prepared = 0;
    // 清除所有轨迹点
    for (uint8_t i = 0; i < 6; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }
    gripper.emergencyStop();

    joint[4].setJointParam(-360, 360, -10, 10); // J5关节在力矩复位后还要重新再限位一下
	joint[5].setJointParam(-360, 360, -10, 10);

    Manipulator_TorqueReset_Params_Reset();
}

void Emergency_action(Event_t **pEvent)
{
    const uint32_t nowTick = xTaskGetTickCount();

    // 急停闪烁指示
    if (((nowTick / 200U) % 2U) == 0U)
    {
        WS_indicator[0] = WS_Red;
    }
    else
    {
        WS_indicator[0] = WS_Black;
    }

    // 链路可用判定
    const bool dr16Online = (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED);
    const bool vt13Online = (VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED && VT13.GetModeSwitch() != MS_Status_Typedef::MS_C);

    // 两路都不可用则转 Lost
    if (!dr16Online && !vt13Online)
    {
        TopStateMachine.pSonEvent = &ToLost;
    }



    // 单片机复位：沿用 Reset 模式组合键（Z + CTRL + SHIFT）
    if ((DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL) && DR16.IsKeyPress(DR16_KEY_SHIFT)) ||
        (VT13.IsKeyPress(VT13_KEY_Z) && VT13.IsKeyPress(VT13_KEY_CTRL) && VT13.IsKeyPress(VT13_KEY_SHIFT)))
    {
        vTaskDelay(1000);
        __set_FAULTMASK(1);
        NVIC_SystemReset();
    }

    // 力矩复位触发条件：沿用 Reset 模式
    const bool torqueResetTrigger = (((DR16.GetS1() == 1 && DR16.GetS2() == 1) || DR16.IsKeyPress(DR16_KEY_F) || (VT13.GetModeSwitch() == MS_Status_Typedef::MS_N && VT13.GetTrigger())) && (Robot_Prepared == 0));

    // 一旦触发，持续执行到完成，避免中途松键导致流程中断
    if (torqueResetTrigger)
    {
        emergencyTorqueResetActive = true;
    }

    if (emergencyTorqueResetActive && !emergencyTorqueResetDone)
    {
        WS_indicator[1] = WS_Yellow;
        emergencyTorqueResetDone = Manipulator_TorqueReset();
    }
    else
    {
        WS_indicator[1] = WS_Blue;
    }

    // 手动控制机械臂（在未执行力矩复位时允许）
    if (!emergencyTorqueResetActive)
    {
        if (vt13Online)
        {
            VT13_Manipulater_JointCtrl();
        }

        if (dr16Online)
        {
            DR16_Manipulater_JointCtrl();
        }
    }

    if (vt13Online)
    {
        if (VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_MoveCtrl(toBottomdata);
        }
        else if (!VT13_IsButtonActive(ButtonType::FN1))
        {
            if (emergency_use_energy_unit_move_ctrl)
            {
                VT13_KeyBoard_MoveCtrl_Custom(toBottomdata);
            }
            else
            {
                VT13_KeyBoard_MoveCtrl(toBottomdata);
            }
        }
    }
    if (dr16Online)
    {
        if (DR16.GetS1() == 2)
        {
            DR16_MoveCtrl(toBottomdata);
        }
        else
        {
            if (emergency_use_energy_unit_move_ctrl)
            {
                KeyBoard_MoveCtrl_Custom(toBottomdata);
            }
            else
            {
                KeyBoard_MoveCtrl(toBottomdata);
            }
        }
    }
    
    // 力矩复位完成后的状态转移
    if (emergencyTorqueResetDone)
    {
        Robot_Prepared = 1;
        Emergency_exitAction();
    }
}

void Emergency_exitAction()
{
    emergencyTorqueResetActive = false;
    emergencyTorqueResetDone = false;

    // 退出前再次做清理，避免残留轨迹
    for (uint8_t i = 0; i < 6; i++)
    {
        joint[i].stopTrajectoryPlanner();
        joint_trajectory_point_array[i].clear();
    }

    gripper.emergencyStop();

    Event_t *resume_event = nullptr;
    if (emergency_entry_state == TopStateId::Mobile)
    {
        resume_event = &ToMobile;
    }
    else if (emergency_entry_state == TopStateId::Lost)
    {
        resume_event = &ToLost;
    }
    else if (emergency_entry_state == TopStateId::Reset)
    {
        resume_event = &ToReset;
    }
    else if (emergency_entry_state == TopStateId::GetEnergyUnit)
    {
        resume_event = &ToGetEnergyUnit;
    }
    else if (emergency_entry_state == TopStateId::ExchangeEnergyUnit)
    {
        resume_event = &ToExchangeEnergyUnit;
    }

    if (resume_event != nullptr)
    {
        TopStateMachine.pSonEvent = resume_event;
    }
    else if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
    {
        TopStateMachine.pSonEvent = &ToMobile;
    }
    else
    {
        TopStateMachine.pSonEvent = &ToLost;
    }
}