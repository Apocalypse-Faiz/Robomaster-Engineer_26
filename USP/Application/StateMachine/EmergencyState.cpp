#include "EmergencyState.h"
#include "internal.h"
#include "KeyBoard.h"
#include "ResetState.h"

static bool emergencyTorqueResetActive = false;
static bool emergencyTorqueResetDone = false;

void Emergency_enterAction()
{    
    Robot_Prepared = 0;
    // 清除所有轨迹点
    for (uint8_t i = 0; i < 6; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }
    gripper.emergencyStop();

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
        if(VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_MoveCtrl(toBottomdata);
        }
        else if(!VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_KeyBoard_MoveCtrl(toBottomdata);
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
            KeyBoard_MoveCtrl(toBottomdata);
        }
    }
    
    // 力矩复位完成后的状态转移
    if (emergencyTorqueResetDone)
    {
        static uint8_t joints_reset_done = 0;
        static uint8_t j4_reset_done, j5_reset_done, j6_reset_done = 0;

        if (joints_reset_done)
        {
            Robot_Prepared = 1;
            Emergency_exitAction();
        }

        if (motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready)
        {
            float j5_current_angle = joint[4].getCurrentAngle();
            float j5_angle_diff = j5_current_angle - 0.0f;    

            float j6_current_angle = joint[5].getCurrentAngle();
            float j6_angle_diff = j6_current_angle - 0.0f;    

            float j4_current_angle = joint[3].getCurrentAngle();
            float j4_angle_diff = j4_current_angle - 0.0f;    

            if (j4_reset_done == 0)
            {
                float j4_current_angle = joint[3].getCurrentAngle();
                float angle_diff = j4_current_angle - 0.0f;    

                if (abs(angle_diff) > 1.0f)
                {
                    if (angle_diff > 0)
                    {
                        joint[3].ajustJointTarget(0.1f);
                    }
                    else
                    {
                        joint[3].ajustJointTarget(-0.1f);
                    }
                }
                else
                {
                    j4_reset_done = 1;
                }
            }

            if (j5_reset_done == 0 && j4_reset_done == 1)
            {
                if (abs(j5_angle_diff) > 1.0f)
                {
                    if (j5_angle_diff > 0)
                    {
                        joint[4].ajustJointTarget(-5.0f);
                    }
                    else
                    {
                        joint[4].ajustJointTarget(5.0f);
                    }
                }
                else                
                {
                    j5_reset_done = 1;
                }
            }

            // if (j6_reset_done == 0 && j5_reset_done == 1 && j4_reset_done == 1)
            // {
            //     if (abs(j6_angle_diff) > 1.0f)
            //     {
            //         if (j6_angle_diff > 0)
            //         {
            //             joint[5].ajustJointTarget(-5.0f);
            //         }
            //         else
            //         {
            //             joint[5].ajustJointTarget(5.0f);
            //         }
            //     }
            //     else                
            //     {
            //         j6_reset_done = 1;
            //     }
            // }

            if (j4_reset_done == 1 && j5_reset_done == 1 /*&& j6_reset_done == 1*/)
            {
                joints_reset_done = 1;
            }
        }
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

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
        TopStateMachine.pSonEvent = &ToMobile;
    else
    {
        TopStateMachine.pSonEvent = &ToLost;
    }
}