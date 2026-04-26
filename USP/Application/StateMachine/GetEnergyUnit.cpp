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
static constexpr float get_energy_unit_default_chassis_height_m = 0.441f;

// 模式切换状态
static bool mode_changing = false;
static uint32_t mode_change_time = 0;

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
    for (int i = 0; i < 6; i++) 
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
     for (uint8_t i = 0; i < 6; i++) 
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
    if (VT13.GetStatus() == VT13_ESTABLISHED) 
    {
        // 接收自定义控制器数据
        for (int i = 0; i < 6; i++) 
        {
            custom_control_data.joint_angle[i] = VT13.Datapack_custom.joint_angle[i];
        }
        
        // 更新自定义控制器
        custom_controller.update(custom_control_data, joint, joint_trajectory_point_array, 6);
    }
    
    VT13_KeyBoard_Grip();
    VT13_KeyboardStoreGrippers(toBottomdata);
}

/**
 * @brief 执行视觉控制模式
 */
void executeVisionControlMode() 
{
    // 检查是否收到小电脑数据
    if (ulTaskNotifyTake(pdTRUE, 0) == 55)
    {
        for (int i = 0; i < 6; i++)
        {
            vision_control_data.joint_angle[i] = reVisiondata.joint_target[i];
        }

        // 更新视觉控制
        vision_controller.update(vision_control_data, joint, joint_trajectory_point_array, 6);
    }

}

/**
 * @brief 执行关节控制接管
 */
void executeJointTakeover() 
{
    uint32_t current_time = HAL_GetTick();
    
    // 所有模式都需要执行关节接管
    for (uint8_t i = 0; i < 6; i++) 
    {
        joint[i].takeOver(current_time);
    }
}

void GetEnergyUnit_enterAction()
{
    for (uint8_t i = 0; i < 6; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }

    // 初始化所有控制器
    initControllers();
    
    // 设置为动作组模式
    current_mode = ControlMode::ACTION_GROUP;
    mode_changing = false;

    // 进入取能量单元状态时，默认将底盘高度目标设置为 0.441m
    SetTargetChassisHeight(get_energy_unit_default_chassis_height_m);

}

void GetEnergyUnit_action(Event_t **pEvent)
{
    // 根据当前模式执行不同逻辑
    if ((DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED) || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)) 
    {
        switch (current_mode) 
        {
            case ControlMode::ACTION_GROUP:
                executeActionGroupMode();

                break;
                
            case ControlMode::CUSTOM_CONTROL:
                executeCustomControlMode();
                executeJointTakeover();
                break;
                
            case ControlMode::VISION_CONTROL:
                executeVisionControlMode();
                executeJointTakeover();
                break;
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

    // 统一在状态机中处理底盘高度微调，避免分散在各自定义移动函数中
    UpdateTargetChassisHeight();

    handleModeSwitchInput(); 

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        Keyboard_Grippers();
        Keyboard_Store(toBottomdata);
    }
    if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        VT13_KeyBoard_Grip();
        VT13_KeyboardStoreGrippers(toBottomdata);
    }

}

void GetEnergyUnit_exitAction()
{
     // 重置所有控制器
     custom_control_data.enable = false;
     vision_control_data.enable = false;
     
     custom_controller.reset();
     vision_controller.reset();
     action_group_controller.reset();
     
     // 重置模式状态
     current_mode = ControlMode::ACTION_GROUP;
     mode_changing = false;
}
