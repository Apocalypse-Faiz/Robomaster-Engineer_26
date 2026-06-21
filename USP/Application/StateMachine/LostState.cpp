#include "LostState.h"
#include "internal.h"

void Lost_enterAction()
{
    g_top_state_id = TopStateId::Lost;

    if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready && motor[6].getStatus() == MotorStatus::ready)
    {
        for (uint8_t i = 0; i < 7; i++)
        {
            joint_trajectory_point_array[i].clear();
            joint[i].stopTrajectoryPlanner();
        }
    }

    LostState.action();
}

void Lost_action(Event_t **pEvent)
{


    if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready && motor[6].getStatus() == MotorStatus::ready)
    {
        // 关节失能
        for (uint8_t i = 0; i < 7; i++)
        {
            joint[i].setMode(JointMode::disabled);
        }
    }
    // 底盘失能

    toBottomdata.ChassisState = 0; // 失能底盘控制
    toBottomdata.chassis_control_struct.x = 0;
    toBottomdata.chassis_control_struct.y = 0;
    toBottomdata.chassis_control_struct.z = 0;
    // toBottomdata.L_Store = 0; // 夹爪待命
    // toBottomdata.R_Store = 0; // 夹爪待命

    for (int i = 0; i < 7; i++)
    {
        if (motor[i].getStatus() != MotorStatus::ready)
        {
            WS_indicator[i] = WS_Red;
        }
        else
        {
            WS_indicator[i] = WS_White;
        }
    }

    // UI
    // UI_Mode = 0;

    gripper.emergencyStop(); // 夹爪失能
    gripper.setState(GripperState::IDLE);

    /* 如果重新建立联系*/
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
    {
        if (Robot_Prepared == 1)
        {
            TopStateMachine.pSonEvent = &ToMobile; // 转到机动状态
        }
        else
        {
            TopStateMachine.pSonEvent = &ToReset;
        }
    }
}

void Lost_exitAction()
{
    // if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready)
    // {
    //     for (uint8_t i = 0; i < 6; i++)
    //     {
    //         joint[i].setMode(JointMode::angle);
    //     }
    // }

    for (uint8_t i = 0; i < 7; i++)
    {
        if (motor[i].getStatus() == MotorStatus::ready)
        {
            joint[i].setMode(JointMode::angle);
        }
    }

    toBottomdata.ChassisState = 1; // 使能底盘控制
    
}
