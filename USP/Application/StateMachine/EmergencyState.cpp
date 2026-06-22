#include "EmergencyState.h"
#include "internal.h"
#include "KeyBoard.h"

static TopStateId emergency_entry_state = TopStateId::None;
static bool emergency_use_energy_unit_move_ctrl = false;

void Emergency_enterAction()
{    
    emergency_entry_state = g_top_state_id;
    g_top_state_id = TopStateId::Emergency;
    emergency_use_energy_unit_move_ctrl = (emergency_entry_state == TopStateId::GetEnergyUnit);

    // 清除所有轨迹点
    for (uint8_t i = 0; i < 7; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }
    gripper.emergencyStop();

    // 紧急状态下放宽J5/J6限位，便于手动恢复
    joint[4].setJointParam(-360, 360, -10, 10);
	joint[5].setJointParam(-360, 360, -10, 10);

}

void Emergency_action(Event_t **pEvent)
{
    const uint32_t nowTick = xTaskGetTickCount();

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

    // DM电机无需力矩复位，紧急状态下直接允许手动控制
    if (vt13Online)
    {
        VT13_Manipulater_JointCtrl();
    }

    if (dr16Online)
    {
        DR16_Manipulater_JointCtrl();
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

    for (int i = 0; i < 7; i++)
    {
        if (motor[i].getStatus() != MotorStatus::ready)
        {
            WS_indicator[i] = WS_Red;
        }
        else
        {
            WS_indicator[i] = WS_Yellow;
        }
    }
}

void Emergency_exitAction()
{
    // 退出前再次做清理，避免残留轨迹
    for (uint8_t i = 0; i < 7; i++)
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