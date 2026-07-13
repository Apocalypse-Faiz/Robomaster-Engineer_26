#include "Keyboard.h"
#include "GetEnergyUnit.h"
#include "Claw.h"
#include "CustomControl.h"
#include "ActionGroupController.h"
#include "VisualControl.h"

/* 控制模式枚举 */
enum class ControlMode {
    ACTION_GROUP,       // 动作组模式
    CUSTOM_CONTROL,     // 自定义控制模式
    VISION_CONTROL      // 视觉控制模式
};

static ControlMode current_mode = ControlMode::ACTION_GROUP;
static CustomData custom_control_data;            // 自定义控制数据
static VisionData vision_control_data;            // 视觉控制数据
static constexpr float get_energy_unit_default_chassis_height_m = 0.357f;

// 模式切换状态
static bool mode_changing = false;
static uint32_t mode_change_time = 0;
static bool vision_takeover_requested = false;
static bool vision_takeover_acknowledged = false;
static bool last_vt13_z_state = false;

/**
 * @brief 初始化所有控制器
 */
void initControllers() 
{
    // 初始化自定义控制数据
    custom_control_data.enable = false;
    custom_control_data.frequency = 60; // 60ms控制频率
    
    // 初始化视觉控制数据
    vision_control_data.enable = false;
    vision_control_data.frequency = 33; // 30Hz控制频率
    // vision_control_data.move_duration = 500; // 500ms移动时间
    
    // 重置所有控制器
    custom_controller.reset();
    vision_controller.reset();
    action_group_controller.reset();
    
    // 设置为当前位置
    for (int i = 0; i < 7; i++) 
    {
        if (joint[i].isCalibrated()) 
        {
            custom_control_data.joint_angle[i] = joint[i].getCurrentAngle();
            vision_control_data.joint_angle[i] = joint[i].getCurrentAngle();
        }
    }
}

/**
 * @brief 切换控制模式
 */
void switchControlMode(ControlMode new_mode) 
{
    if (current_mode == new_mode) return;

     // 停止所有关节轨迹规划
     for (uint8_t i = 0; i < 7; i++) 
     {
         joint[i].stopTrajectoryPlanner();
         joint_trajectory_point_array[i].clear();
     }
    
    // 标记正在切换模式
    mode_changing = true;
    mode_change_time = HAL_GetTick();
    
    // 退出当前模式
    switch (current_mode) 
    {
        case ControlMode::ACTION_GROUP:
            // 重置动作组控制器
            action_group_controller.reset();
            break;
            
        case ControlMode::CUSTOM_CONTROL:
            custom_control_data.enable = false;
            custom_controller.reset();
            break;
            
        case ControlMode::VISION_CONTROL:
            vision_control_data.enable = false;
            vision_controller.reset();
            vision_takeover_requested = false;
            vision_takeover_acknowledged = false;
            vision_comm_flag_override = 0U;
            vision_comm_flag_override_valid = false;
            break;
    }
    
    // 设置新模式
    current_mode = new_mode;
    
    // 进入新模式
    switch (new_mode) 
    {
        case ControlMode::ACTION_GROUP:
            // 动作组模式无需特殊初始化
            break;
            
        case ControlMode::CUSTOM_CONTROL:
            custom_control_data.enable = true;
            break;
            
        case ControlMode::VISION_CONTROL:
            vision_control_data.enable = true;
            vision_takeover_requested = false;
            vision_takeover_acknowledged = false;
            vision_comm_flag_override = 0U;
            vision_comm_flag_override_valid = false;
            break;
    }
}

static bool SwitchModeFromCustomData()
{
    if (VT13.GetStatus() != VT13_ESTABLISHED)
    {
        return false;
    }

    static uint8_t last_mode_command = 0;
    uint8_t mode_command = VT13.Datapack_custom.mode & 0x03u; // 只关注最低两位

    if (mode_command == last_mode_command)
    {
        return false; // 没有新的模式切换命令
    }

    last_mode_command = mode_command;

    switch (mode_command)
    {
        case 1:
            switchControlMode(ControlMode::ACTION_GROUP);
            return true;

        case 2:
            if (current_mode == ControlMode::ACTION_GROUP && !action_group_controller.isActionGroupCompleted())
            {
                return false; // 动作组未完成，不切换
            }
            switchControlMode(ControlMode::CUSTOM_CONTROL);
            return true;

        case 3:
            if (current_mode == ControlMode::ACTION_GROUP && !action_group_controller.isActionGroupCompleted())
            {
                return false; // 动作组未完成，不切换
            }
            switchControlMode(ControlMode::VISION_CONTROL);
            return true;

        default:
            return false;
    }
}

/**
 * @brief 处理模式切换按键
 */
void handleModeSwitchInput() 
{
    if (SwitchModeFromCustomData())
    {
        return;
    }

    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 防抖处理
    if (current_time - last_press_time < 500) return;
    
    // G键切换模式：ACTION_GROUP -> CUSTOM_CONTROL -> VISION_CONTROL -> ACTION_GROUP
    if (DR16.IsKeyPress(DR16_KEY_G) || VT13.IsKeyPress(VT13_KEY_G)) 
    {
        last_press_time = current_time;
        
        ControlMode next_mode;
        switch (current_mode) 
        {
            case ControlMode::ACTION_GROUP:
                if (!action_group_controller.isActionGroupCompleted()) return;
                next_mode = ControlMode::CUSTOM_CONTROL;
                // next_mode = ControlMode::VISION_CONTROL;      // 视觉调试用
                break;
            case ControlMode::CUSTOM_CONTROL:
                next_mode = ControlMode::VISION_CONTROL;
                break;
            case ControlMode::VISION_CONTROL:
                next_mode = ControlMode::ACTION_GROUP;
                break;
        }
        
        switchControlMode(next_mode);
    }
}

/**
 * @brief 执行动作组模式
 */
void executeActionGroupMode() 
{
    // 使用动作组控制器处理所有动作组逻辑
    action_group_controller.update(joint, joint_trajectory_point_array, gripper, toBottomdata, reBottomdata);
}

/**
 * @brief 执行自定义控制模式
 */
void executeCustomControlMode() 
{
    // 检查图传连接状态
    if (VT13.Check_CustomController_Link()) 
    {
        // 接收自定义控制器数据
        for (int i = 0; i < 7; i++) 
        {
            custom_control_data.joint_angle[i] = VT13.Datapack_custom.joint_angle[i];
        }
        
        // 更新自定义控制器
        custom_controller.update(custom_control_data, joint, joint_trajectory_point_array, 7);
    }
    
    VT13_KeyBoard_Grip();
    VT13_KeyboardStoreGrippers(toBottomdata);
}

/**
 * @brief 执行视觉控制模式
 */
void executeVisionControlMode(bool vision_data_ready) 
{
    // 检查是否收到小电脑数据
    if (vision_data_ready)
    {
        for (int i = 0; i < 7; i++)
        {
            vision_control_data.joint_angle[i] = reVisiondata.joint_target[i];
        }

        // 更新视觉控制
        vision_controller.update(vision_control_data, joint, joint_trajectory_point_array, 7);
        SetTargetChassisHeight(reVisiondata.chassis_height_target);
    }
}

/**
 * @brief 执行关节控制接管
 */
void executeJointTakeover()
{
    uint32_t current_time = HAL_GetTick();

    // 所有模式都需要执行关节接管
    for (uint8_t i = 0; i < 7; i++)
    {
        joint[i].takeOver(current_time);

        // 轨迹速度/加速度前馈 (与 ActionGroup_Execute 一致)
        float v_des = 0.0f, a_des = 0.0f;
        joint[i].getTrajectoryDerivative(current_time + 1, &v_des, &a_des);

        const float k = joint[i].mapping.k;
        if (k != 0.0f)
        {
            const float deg2rad = 3.14159265f / 180.0f;
            float v_motor = v_des / k * deg2rad;
            float a_motor = a_des / k * deg2rad;

            motor[i].setSpeed_feedforward(v_motor * speed_ff_gain[i]);
            motor[i].setAcceleration_feedforward(a_motor * acc_ff_gain[i]);
        }
    }
}

void GetEnergyUnit_enterAction()
{
    g_top_state_id = TopStateId::GetEnergyUnit;

    for (uint8_t i = 0; i < 7; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }

    // 初始化所有控制器
    initControllers();

    // 设置为动作组模式
    current_mode = ControlMode::ACTION_GROUP;

    mode_changing = false;
    vision_takeover_requested = false;
    vision_takeover_acknowledged = false;
    vision_comm_flag_override = 0U;
    vision_comm_flag_override_valid = false;

    // 进入取能量单元状态时，默认将底盘高度目标设置为 0.35m
    toBottomdata.ChassisState = 2;
    SetTargetChassisHeight(get_energy_unit_default_chassis_height_m);

}

void GetEnergyUnit_action(Event_t **pEvent)
{
    // 云台升降电机升到上限位 (步进, 不跳变)
    if (gimbal_lift_init_done)
        Gimbal_Lift_StepToGoal(gimbal_lift_max_angle, 20.0f); 

    bool vision_data_ready = false;
    uint32_t vision_notify_value = 0U;
    if (current_mode == ControlMode::VISION_CONTROL)
    {
        if (xTaskNotifyWait(0U, VISION_NOTIFY_BIT, &vision_notify_value, 0U) == pdTRUE)
        {
            vision_data_ready = ((vision_notify_value & VISION_NOTIFY_BIT) != 0U);
        }
    }


    if (current_mode == ControlMode::VISION_CONTROL)
    {
        const bool dr16_ready = (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED);
        const bool vt13_ready = (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED);
        bool z_triggered = false;

        if (dr16_ready && IsKeyPressOnce(DR16_KEY_Z))
        {
            z_triggered = true;
        }

        if (vt13_ready)
        {
            const bool vt13_z_pressed = VT13.IsKeyPress(VT13_KEY_Z);
            if (vt13_z_pressed && !last_vt13_z_state)
            {
                z_triggered = true;
            }
            last_vt13_z_state = vt13_z_pressed;
        }
        else
        {
            last_vt13_z_state = false;
        }

        if (z_triggered)
        {
            vision_takeover_requested = true;
            vision_takeover_acknowledged = false;
            vision_comm_flag_override = 9U;
            vision_comm_flag_override_valid = true;
        }

        if (vision_takeover_requested && reVisiondata.comm_flag == 8U)
        {
            vision_takeover_requested = false;
            vision_takeover_acknowledged = true;
            vision_comm_flag_override = 0U;
            vision_comm_flag_override_valid = true;
        }
    }

    // 根据当前模式执行不同逻辑
    if ((DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED) || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)) 
    {
        switch (current_mode) 
        {
            case ControlMode::ACTION_GROUP:
                ArmMode = 0;
                executeActionGroupMode();

                break;
                
            case ControlMode::CUSTOM_CONTROL:
                ArmMode = 1;
                executeCustomControlMode();
                executeJointTakeover();
                break;
                
            case ControlMode::VISION_CONTROL:
                ArmMode = 2;
                executeVisionControlMode(vision_data_ready);
                executeJointTakeover();
                break;
        }
        
    }

    for (int i = 0; i < 6; i++)  // 硬件仅6个灯珠，对应motor[0]~motor[5]
    {
        if (motor[i].getStatus() != MotorStatus::ready)
        {
            WS_indicator[i] = WS_Red;
        }
        else
        {
            if (current_mode == ControlMode::ACTION_GROUP)
            {
                if (i == action_group_controller.getSelectedOre())
                {
                    WS_indicator[i] = WS_Yellow;
                }
                else
                {
                    WS_indicator[i] = WS_Green;
                }
            }
            else if (current_mode == ControlMode::CUSTOM_CONTROL)
            {
                if (VT13.Check_CustomController_Link())
                {
                    WS_indicator[i] = WS_Blue;
                }
                else
                {
                    WS_indicator[i] = WS_Yellow; 
                }
            }
            else if (current_mode == ControlMode::VISION_CONTROL)
            {
                if (vision_takeover_requested)
                {
                    if (i == 0)
                    WS_indicator[i] = WS_Blue;
                }
                else if (vision_takeover_acknowledged)
                {
                    if (i == 0)
                    WS_indicator[i] = WS_Green;
                }
                else if (vision_data_ready)
                {
                    WS_indicator[i] = WS_Purple;
                }
                else
                {
                    WS_indicator[i] = WS_Yellow; 
                }
            }
        }
    }

    switch (current_mode)
    {
        case ControlMode::ACTION_GROUP :
            if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                if (DR16.GetS1() == 2)
                {
                    DR16_MoveCtrl(toBottomdata);
                }
                else
                {
                    KeyBoard_MoveCtrl_Custom(toBottomdata);
                }
            }
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                if(VT13_IsButtonActive(ButtonType::FN1))
                {
                    VT13_MoveCtrl(toBottomdata);
                }
                else if(!VT13_IsButtonActive(ButtonType::FN1))
                {
                    VT13_KeyBoard_MoveCtrl_Custom(toBottomdata);
                }
            }
            break;
        case ControlMode::CUSTOM_CONTROL :
            if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                if (DR16.GetS1() == 2)
                {
                    DR16_MoveCtrl(toBottomdata);
                }
                else
                {
                    KeyBoard_MoveCtrl_Custom(toBottomdata);
                }
            }
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                if(VT13_IsButtonActive(ButtonType::FN1))
                {
                    VT13_MoveCtrl(toBottomdata);
                }
                else if(!VT13_IsButtonActive(ButtonType::FN1))
                {
                    VT13_KeyBoard_MoveCtrl_Custom(toBottomdata);
                }
            }
            break;
        case ControlMode::VISION_CONTROL :
            if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                if (DR16.GetS1() == 2)
                {
                    DR16_MoveCtrl(toBottomdata);
                }
                else
                {
                    KeyBoard_MoveCtrl_Custom(toBottomdata);
                }
            }
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                if(VT13_IsButtonActive(ButtonType::FN1))
                {
                    VT13_MoveCtrl(toBottomdata);
                }
                else if(!VT13_IsButtonActive(ButtonType::FN1))
                {
                    VT13_KeyBoard_MoveCtrl_Custom(toBottomdata);
                }
            }
            break;        
        default:
            break;
    }

    handleModeSwitchInput(); 

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        Keyboard_Grippers();
        // Keyboard_Store(toBottomdata);
    }
    if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        VT13_KeyBoard_Grip();
        // VT13_KeyboardStoreGrippers(toBottomdata);
    }

}

void GetEnergyUnit_exitAction()
{
     // 停止轨迹
     for (uint8_t i = 0; i < 7; i++)
     {
         joint_trajectory_point_array[i].clear();
         joint[i].stopTrajectoryPlanner();
     }

     // 重置所有控制器
     custom_control_data.enable = false;
     vision_control_data.enable = false;

     custom_controller.reset();
     vision_controller.reset();
     action_group_controller.reset();

     // 重置模式状态
     current_mode = ControlMode::ACTION_GROUP;
     mode_changing = false;
    vision_takeover_requested = false;
    vision_takeover_acknowledged = false;
    vision_comm_flag_override = 0U;
    vision_comm_flag_override_valid = false;
}
