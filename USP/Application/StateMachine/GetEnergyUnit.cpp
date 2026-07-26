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
static constexpr float vision_start_chassis_height_m = 0.350f;

// 模式切换状态
static bool mode_changing = false;
static uint32_t mode_change_time = 0;
static TickType_t last_vision_rx_tick = 0U;
static bool vision_frame_received = false;
static constexpr uint32_t vision_link_timeout_ms = 100U;

static float joint_takeover_speed_ff[7] = {0.0f};
static float joint_takeover_acc_ff[7] = {0.0f};
static uint32_t joint_takeover_ff_last_time = 0U;
static ControlMode joint_takeover_ff_last_mode = ControlMode::ACTION_GROUP;

static float clampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void resetVisionLinkState()
{
    last_vision_rx_tick = 0U;
    vision_frame_received = false;
}

static void resetJointTakeoverFeedforward()
{
    for (uint8_t i = 0; i < 7; i++)
    {
        joint_takeover_speed_ff[i] = 0.0f;
        joint_takeover_acc_ff[i] = 0.0f;
        motor[i].setSpeed_feedforward(0.0f);
        motor[i].setAcceleration_feedforward(0.0f);
    }
}

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
    
    // 重置所有控制器
    custom_controller.reset();
    vision_controller.reset();
    action_group_controller.reset();
    resetJointTakeoverFeedforward();

    // Set target to current joint position.
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
            resetVisionLinkState();
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
            resetVisionLinkState();
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
    static bool last_f_key_state = false;
    const bool f_key_pressed =
        DR16.IsKeyPress(DR16_KEY_F) || VT13.IsKeyPress(VT13_KEY_F);
    const bool f_key_rising_edge = f_key_pressed && !last_f_key_state;

    // F 键上升沿触发起始动作组；触发不依赖小电脑是否有新数据。
    if (f_key_rising_edge)
    {
        vision_controller.startSoftStart(joint, joint_trajectory_point_array, 7);
        SetTargetChassisHeight(vision_start_chassis_height_m);
    }
    last_f_key_state = f_key_pressed;

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
    const bool smooth_feedforward =
        current_mode == ControlMode::CUSTOM_CONTROL ||
        current_mode == ControlMode::VISION_CONTROL;

    if (joint_takeover_ff_last_mode != current_mode ||
        current_time < joint_takeover_ff_last_time ||
        joint_takeover_ff_last_time == 0U)
    {
        resetJointTakeoverFeedforward();
    }

    float dt = (current_time - joint_takeover_ff_last_time) * 1e-3f;
    dt = clampFloat(dt, 0.001f, 0.05f);

    constexpr float deg2rad = 3.14159265f / 180.0f;
    constexpr float custom_speed_ff_lpf_alpha = 0.150f;
    constexpr float custom_speed_ff_max_abs = 10.0f;   // rad/s
    constexpr float custom_speed_ff_max_slew = 25.0f; // rad/s^2
    constexpr float custom_acc_ff_lpf_alpha = 1.0f;
    constexpr float custom_acc_ff_max_abs = 3.0f;      // torque command
    constexpr float custom_acc_ff_max_slew = 30.0f;   // torque command/s

    // Joint takeover is required in all modes.
    for (uint8_t i = 0; i < 7; i++)
    {
        joint[i].takeOver(current_time);

        float speed_ff = 0.0f;
        float acc_ff = 0.0f;
        float v_des = 0.0f, a_des = 0.0f;
        const float k = joint[i].mapping.k;

        if (k != 0.0f && joint[i].getTrajectoryDerivative(current_time + 1, &v_des, &a_des))
        {
            const float v_motor = v_des / k * deg2rad;
            const float a_motor = a_des / k * deg2rad;

            speed_ff = v_motor * speed_ff_gain[i];
            acc_ff = a_motor * acc_ff_gain[i];

            if (smooth_feedforward)
            {
                speed_ff = clampFloat(speed_ff, -custom_speed_ff_max_abs, custom_speed_ff_max_abs);
                speed_ff = joint_takeover_speed_ff[i] +
                           custom_speed_ff_lpf_alpha * (speed_ff - joint_takeover_speed_ff[i]);

                const float max_step = custom_speed_ff_max_slew * dt;
                speed_ff = joint_takeover_speed_ff[i] +
                           clampFloat(speed_ff - joint_takeover_speed_ff[i], -max_step, max_step);
            }
        }

        if (smooth_feedforward)
        {
            // Short-cycle replanning makes desired acceleration discontinuous. Filter and
            // rate-limit the torque-level acceleration feedforward before applying it.
            // When the trajectory ends, the same path also ramps the previous FF back to zero.
            acc_ff = clampFloat(acc_ff, -custom_acc_ff_max_abs, custom_acc_ff_max_abs);
            acc_ff = joint_takeover_acc_ff[i] +
                     custom_acc_ff_lpf_alpha * (acc_ff - joint_takeover_acc_ff[i]);

            const float acc_max_step = custom_acc_ff_max_slew * dt;
            acc_ff = joint_takeover_acc_ff[i] +
                     clampFloat(acc_ff - joint_takeover_acc_ff[i], -acc_max_step, acc_max_step);
        }

        joint_takeover_speed_ff[i] = speed_ff;
        joint_takeover_acc_ff[i] = acc_ff;
        motor[i].setSpeed_feedforward(speed_ff);
        motor[i].setAcceleration_feedforward(acc_ff);
    }

    joint_takeover_ff_last_time = current_time;
    joint_takeover_ff_last_mode = current_mode;
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
    resetVisionLinkState();

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
    bool vision_link_alive = false;
    uint32_t vision_notify_value = 0U;
    const TickType_t current_tick = xTaskGetTickCount();

    /* Always consume the bit so data received in other modes cannot become stale. */
    if ((xTaskNotifyWait(0U, VISION_NOTIFY_BIT, &vision_notify_value, 0U) == pdTRUE) &&
        ((vision_notify_value & VISION_NOTIFY_BIT) != 0U) &&
        (current_mode == ControlMode::VISION_CONTROL))
    {
        vision_data_ready = true;
        vision_frame_received = true;
        last_vision_rx_tick = current_tick;
    }

    if ((current_mode == ControlMode::VISION_CONTROL) && vision_frame_received)
    {
        vision_link_alive =
            ((current_tick - last_vision_rx_tick) < pdMS_TO_TICKS(vision_link_timeout_ms));
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
                // 手动储取矿模式: LED1~LED4 对应 LF、RF、LB、RB；
                // 自动模式: 保持原有 LED1~LED6 显示取矿编号的逻辑。
                const bool show_belly_slot =
                    action_group_controller.isManualSlotMode();
                const uint8_t selected_indicator =
                    show_belly_slot
                        ? action_group_controller.getSelectedBellySlot()
                        : action_group_controller.getSelectedOre();

                if (i == selected_indicator)
                {
                    // 储矿槽索引用青色，原有取矿编号继续使用黄色。
                    WS_indicator[i] = show_belly_slot ? WS_Purple : WS_Yellow;
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
                if (vision_link_alive)
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
     resetVisionLinkState();
}
