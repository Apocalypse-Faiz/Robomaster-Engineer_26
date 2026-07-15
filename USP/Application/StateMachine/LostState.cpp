#include "LostState.h"
#include "internal.h"

// 调试模式: 进 Lost 时仅失能 7 个关节, 底盘保持原 ChassisState 与 chassis_height
//   置 1 = 调试 (底盘不失能), 置 0 = 正常 (底盘失能)
#define LOST_DEBUG_KEEP_CHASSIS 0

// 记录进入 Lost 前的状态, 重连后恢复到该状态
static TopStateId lost_entry_state = TopStateId::None;

void Lost_enterAction()
{
    lost_entry_state = g_top_state_id; // 记录进入 Lost 前的状态 (此时还未改为 Lost)
    g_top_state_id = TopStateId::Lost;

    // Lost 必须无条件触发全臂安全停机，不能等待所有电机 ready。
    ArmSafety_Trip();

    LostState.action();
}

void Lost_action(Event_t **pEvent)
{

    // 持续压制其他任务可能写入的目标/模式，Lost 内绝不允许自动使能。
    ArmSafety_Trip();

   

#if !LOST_DEBUG_KEEP_CHASSIS
    // 底盘失能 (正常模式)
    toBottomdata.ChassisState = 0; // 失能底盘控制
    toBottomdata.chassis_control_struct.x = 0;
    toBottomdata.chassis_control_struct.y = 0;
    toBottomdata.chassis_control_struct.z = 0;

    if (Gimbal_Lift.getStatus() == MotorStatus::ready)
    {
        Gimbal_Lift.setMode(MotorMode::disabled);
    }
    // toBottomdata.L_Store = 0; // 夹爪待命
    // toBottomdata.R_Store = 0; // 夹爪待命
#else
    // 调试模式: 底盘保持原 ChassisState 与 chassis_height, 不失能
    //   仅清运动指令, 防止失联时还在按上一帧速度移动
    toBottomdata.chassis_control_struct.x = 0;
    toBottomdata.chassis_control_struct.y = 0;
    toBottomdata.chassis_control_struct.z = 0;

    if (gimbal_lift_init_done)
    Gimbal_Lift_StepToGoal(gimbal_lift_max_angle, 20.0f); 
#endif

    for (int i = 0; i < 6; i++)  // 硬件仅6个灯珠，对应motor[0]~motor[5]
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

    /* 如果重新建立联系: 恢复到进入 Lost 前的状态 */
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
    {
        switch (lost_entry_state)
        {
        case TopStateId::GetEnergyUnit:
            TopStateMachine.pSonEvent = &ToGetEnergyUnit;
            break;
        case TopStateId::ExchangeEnergyUnit:
            TopStateMachine.pSonEvent = &ToExchangeEnergyUnit;
            break;
        case TopStateId::Mobile:
        default: // 初次开机(None)等情况默认转机动
            TopStateMachine.pSonEvent = &ToMobile;
            break;
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

    // 退出 Lost 不直接使能。链路正常后由 CheckMotorStatus()
    // 自动清错、启动，并在 7 个 DM 连续 ready 20 ms 后安全恢复。

#if !LOST_DEBUG_KEEP_CHASSIS
    toBottomdata.ChassisState = 1; // 使能底盘控制 (正常模式)
#endif

}
