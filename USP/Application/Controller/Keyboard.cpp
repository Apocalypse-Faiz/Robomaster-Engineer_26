#include "srml_std_lib.h"
#include "Keyboard.h"

bool VT13_ButtonStates[3] = {false, false, false}; // 存储每个按钮的触发状态
static bool lastButtonState[3] = {false, false, false};
bool GripperStates[3] = {false, false, false}; // 存储每个夹爪的触发状态 （二代车的储矿换成了电磁铁，但本质没变，继续用这个标志位）
static bool lastGripperState[3] = {false, false, false};
// 单按钮处理函数
bool handleButtonToggle(ButtonType buttonType, bool isPressed) 
{
    int index = static_cast<int>(buttonType);
    bool changed = false;

    // 检测上升沿：之前未按下，现在按下
    if (isPressed && !lastButtonState[index]) 
    {
        // 翻转内部触发状态
        VT13_ButtonStates[index] = !VT13_ButtonStates[index];
        changed = true;
    }

    // 更新上一次物理状态
    lastButtonState[index] = isPressed;

    return changed;
}

// 主处理函数
void VT13_ProcessButtons() 
{
     // 检查VT13连接状态，如果断连则清零所有按键状态
    if (VT13.GetStatus() != LinkageStatus_Typedef::ESTABLISHED)
    {
        for (int i = 0; i < 3; i++) 
        {
            VT13_ButtonStates[i] = false;
            lastButtonState[i] = false;
        }
        return;
    }
    handleButtonToggle(ButtonType::FN1, VT13.GetFN1());
    handleButtonToggle(ButtonType::FN2, VT13.GetFN2());
    handleButtonToggle(ButtonType::TRIGGER, VT13.GetTrigger());
}

// 获取特定按钮当前状态的辅助函数
bool VT13_IsButtonActive(ButtonType buttonType) 
{
    return VT13_ButtonStates[static_cast<int>(buttonType)];
}

// max
float max(float a, float b)
{
    return ((a >= b) ? (a) : (b));
}
// min
float min(float a, float b)
{
    return ((a < b) ? (a) : (b));
}
// constrain
float constrain(float val, float lb, float rb)
{
    return ::min(::max(val, lb), rb);
}
// linear mapping
float map(float x, float in_lb, float in_rb, float out_lb, float out_rb)
{
    return (::constrain(x, in_lb, in_rb) - in_lb) * (out_rb - out_lb) / (in_rb - in_lb) + out_lb;
}
// sign
float sgn(float val)
{
    return (val < 0) ? (-1.0f) : ((val > 0) ? (1.0f) : (0.0f));
}
// static deadband
float deadband(float val, float center, float radius)
{
    return (fabsf(val - center) >= radius ? (val - ::sgn(val - center) * radius) : center);
}

// inline constexpr float chassis_max_resultant_speed = 3.5f;   // max resultant speed of chassis
// inline constexpr float chassis_max_translation_speed = 6.5f; // max translation speed of chassis
// inline constexpr float chassis_max_spinning_speed = 8;       // max spinning speed of chassis

constexpr float chassis_max_resultant_speed = 3.5f;   // 底盘最大合速度 m/s
constexpr float chassis_max_translation_speed = 6.5f; // 底盘最大平移速度 m/s
constexpr float chassis_max_spinning_speed = 8;  

void SetTargetChassisHeight(float target_height)
{
    if (toBottomdata.ChassisState != 2)
    {
        return;
    }

    target_chassis_height = constrain(target_height, chassis_height_min, chassis_height_max);
    target_height_initialized = true;
    align_to_target_active = true;
}

void UpdateTargetChassisHeight()
{
    if (toBottomdata.ChassisState != 2)
    {
        return;
    }

    const bool dr16_established = (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED);
#if USE_SRML_VT13
    const bool vt13_established = (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED);
#else
    const bool vt13_established = false;
#endif

    const bool dr16_custom_height_enabled = dr16_established && (DR16.GetS1() != 2);
#if USE_SRML_VT13
    const bool vt13_custom_height_enabled = vt13_established && !VT13_IsButtonActive(ButtonType::FN1);
#else
    const bool vt13_custom_height_enabled = false;
#endif

    if (!dr16_established && !vt13_established)
    {
        return;
    }

    float updated_target_chassis_height = target_chassis_height;
    bool has_manual_command = false;

    if (dr16_custom_height_enabled)
    {
        if (DR16.IsKeyPress(DR16_KEY_R) && DR16.IsKeyPress(DR16_KEY_CTRL))
        {
            updated_target_chassis_height = current_chassis_height - 0.1f;
            has_manual_command = true;
        }
        else if (DR16.IsKeyPress(DR16_KEY_R) && DR16.IsKeyPress(DR16_KEY_SHIFT))
        {
            updated_target_chassis_height = current_chassis_height + 0.1f;
            has_manual_command = true;
        }
    }

#if USE_SRML_VT13
    if (!has_manual_command && vt13_custom_height_enabled)
    {
        if (VT13.IsKeyPress(VT13_KEY_R) && VT13.IsKeyPress(VT13_KEY_CTRL))
        {
            updated_target_chassis_height = current_chassis_height - 0.1f;
            has_manual_command = true;
        }
        else if (VT13.IsKeyPress(VT13_KEY_R) && VT13.IsKeyPress(VT13_KEY_SHIFT))
        {
            updated_target_chassis_height = current_chassis_height + 0.1f;
            has_manual_command = true;
        }
    }
#endif

    // 手动控制优先；无手动输入时若处于进状态自动对位阶段则继续对位，否则保持当前位置。
    if (has_manual_command)
    {
        align_to_target_active = false;
    }
    else if (align_to_target_active)
    {
        updated_target_chassis_height = target_chassis_height;
        if (fabsf(current_chassis_height - target_chassis_height) < 2e-3f)
        {
            align_to_target_active = false;
        }
    }
    else
    {
        updated_target_chassis_height = current_chassis_height;
    }

    target_chassis_height = constrain(updated_target_chassis_height,
                                      chassis_height_min,
                                      chassis_height_max);
}

// 关于遥控器：1为上，2为下，3为中

bool keyPressedFlag[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/**
 * @brief 判断按键是否只按下了一次（不论按的时间长短）
 *
 * @param _key
 * @return true
 * @return false
 */
bool IsKeyPressOnce(int _key)
{
    if (!keyPressedFlag[_key] && DR16.IsKeyPress(_key)) // 如果按键没有按下的记录
    {
        keyPressedFlag[_key] = true; // 记录按键被按下
        return true;
    }
    else if (!DR16.IsKeyPress(_key)) // 按键松开，清除标志
    {
        keyPressedFlag[_key] = false;
        return false;
    }
    else
        return false;
}

#if USE_SRML_VT13
static bool ChassisDisableToggle()
{
    static bool chassis_disabled = false;
    static bool last_combo_pressed = false;

    const bool combo_pressed = VT13.IsKeyPress(VT13_KEY_SHIFT) && VT13.IsKeyPress(VT13_KEY_CTRL);
    if (combo_pressed && !last_combo_pressed)
    {
        chassis_disabled = !chassis_disabled;
    }

    last_combo_pressed = combo_pressed;
    return chassis_disabled;
}
#endif

#if USE_SRML_VT13
void VT13_KeyBoard_MoveCtrl(ToBottomDataStructdef &_ToBottomData)
{
    float temp_vx = 0.0f;
    float temp_vy = 0.0f;
    float temp_wz = 0.0f;

    float temp_Speed = 1.0f;

    static float increas_buffer[4] = {0, 0, 0, 0};
    static int commanded_chassis_state = 1;
    static int last_rebottom_chassis_state = 1;
    static bool last_f_key_state = false;
    static bool last_b_key_state = false;

    ChassisControlStruct chassis_control_struct = {};
    chassis_control_struct.rfid_height = _ToBottomData.chassis_control_struct.rfid_height;
    chassis_control_struct.chassis_height = _ToBottomData.chassis_control_struct.chassis_height;

    if (ChassisDisableToggle())
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
        return;
    }

    if (VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {
        const bool f_key_pressed = VT13.IsKeyPress(VT13_KEY_F);
        const bool b_key_pressed = VT13.IsKeyPress(VT13_KEY_C);
        const bool f_rising_edge = f_key_pressed && !last_f_key_state;
        const bool b_rising_edge = b_key_pressed && !last_b_key_state;

        // if (VT13.IsKeyPress(VT13_KEY_SHIFT))
        // {
        //     temp_Speed = 1.5;
        // }
        // else if (VT13.IsKeyPress(VT13_KEY_CTRL))
        // {
        //     temp_Speed = 0.5;
        // }
        // else
        // {
            temp_Speed = 3.75f;
        // }

        if (commanded_chassis_state == 1)
        {
            if (f_rising_edge && !b_key_pressed) 
            {
                commanded_chassis_state = 3;
            }
            else if (b_rising_edge && !f_key_pressed)
            {
                commanded_chassis_state = 4;
            }
        }
        else if (last_rebottom_chassis_state == 3 && reBottomdata.ChassisState == 1)
        {
            commanded_chassis_state = 1;
        }
        else if (last_rebottom_chassis_state == 4)
        {
            if (VT13.IsKeyPress(VT13_KEY_X))
            {
                commanded_chassis_state = 1;
            }
            else if (reBottomdata.ChassisState == 1)
            {
                commanded_chassis_state = 1;
            }
        }

        if (commanded_chassis_state == 4)
        {
            if (last_rebottom_chassis_state == 0)
            commanded_chassis_state = 1;
        }

        _ToBottomData.ChassisState = commanded_chassis_state;

        last_rebottom_chassis_state = reBottomdata.ChassisState;
        last_f_key_state = f_key_pressed;
        last_b_key_state = b_key_pressed;

        if (VT13.IsKeyPress(VT13_KEY_D))
        {
            increas_buffer[0] += 0.05;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            temp_vy = 0.8 * temp_Speed * increas_buffer[0];
        }
        else if (VT13.IsKeyPress(VT13_KEY_A))
        {
            increas_buffer[1] += 0.05;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            temp_vy = -0.8 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0.5;
            increas_buffer[1] = 0.5;
            temp_vy = 0;
        }

        if (VT13.IsKeyPress(VT13_KEY_S))
        {
            increas_buffer[2] += 0.05;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);

            temp_vx = -0.8 * temp_Speed * increas_buffer[2];
        }
        else if (VT13.IsKeyPress(VT13_KEY_W))
        {
            increas_buffer[3] += 0.05;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);

            temp_vx = 0.8 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0.5;
            increas_buffer[3] = 0.5;
            temp_vx = 0;
        }

        temp_wz = -10 * temp_Speed * VT13.Get_MouseX_Norm();

        // read remote data
        const float x = temp_vx;
        const float y = temp_vy;

        const float z = constrain(temp_wz, -chassis_max_spinning_speed, chassis_max_spinning_speed);

        const float magxy = sqrtf(x * x + y * y);
        // chassis control struct
        if (fabsf(magxy) < 1e-5)
        {
            chassis_control_struct.x = 0;
            chassis_control_struct.y = 0;
        }
        else
        {
            chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
        }
        chassis_control_struct.z = z;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
    else
    {
        // _ToBottomData.ChassisState = 0;
        commanded_chassis_state = 1;
        last_rebottom_chassis_state = reBottomdata.ChassisState;
        last_f_key_state = false;
        last_b_key_state = false;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

void VT13_MoveCtrl(ToBottomDataStructdef &_ToBottomData)
{
    ChassisControlStruct chassis_control_struct = {};
    float temp_wz = 0.0f;

    if (VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {

        // yaw_target_angle += map(deadband(VT13.GetCh0(), 1024, 50), 364, 1684, 0.3, -0.3);
        // if (VT13.GetTrigger() == 1)
        // {
        //     Chassis_flag = 0; // 小陀螺模式
        //     temp_wz = 4;
        // }
        // else
        // {
        //     Chassis_flag = 1; // 底盘跟随模式
        // }

        _ToBottomData.ChassisState = 1;

        // read remote data
        const float x = map(deadband(VT13.GetCh2(), 1024, 50), 364, 1684, -chassis_max_translation_speed, chassis_max_translation_speed);
        const float y = map(deadband(VT13.GetCh3(), 1024, 50), 364, 1684, -chassis_max_translation_speed, chassis_max_translation_speed);
        const float z = -map(deadband(VT13.GetCh0(), 1024, 50), 364, 1684, -chassis_max_spinning_speed, chassis_max_spinning_speed);
        const float magxy = sqrtf(x * x + y * y);
        // const float z = constrain(temp_wz, -chassis_max_spinning_speed, chassis_max_spinning_speed);

        // chassis control struct
        if (fabsf(magxy) < 1e-5)
        {
            chassis_control_struct.x = 0;
            chassis_control_struct.y = 0;
        }
        else
        {
            chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
        }
        chassis_control_struct.z = z;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
    else
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

void VT13_KeyBoard_MoveCtrl_Custom(ToBottomDataStructdef &_ToBottomData)
{
    // 自定义控制器模式下的按键控制底盘移动
    float temp_vx = 0.0f;
    float temp_vy = 0.0f;  
    float temp_wz = 0.0f;

    float temp_Speed = 1.0f;

    static float increas_buffer[4] = {0, 0, 0, 0};
    ChassisControlStruct chassis_control_struct = {};
    chassis_control_struct.rfid_height = _ToBottomData.chassis_control_struct.rfid_height;
    chassis_control_struct.chassis_height = _ToBottomData.chassis_control_struct.chassis_height;

    if (ChassisDisableToggle())
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
        return;
    }

    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {
        _ToBottomData.ChassisState = 2;

        // if (VT13.IsKeyPress(VT13_KEY_SHIFT))
        // {
        //     temp_Speed = 1;
        // }
        // else if (VT13.IsKeyPress(VT13_KEY_CTRL))
        // {
        //     temp_Speed = 0.5;
        // }
        // else
        // {
            temp_Speed = 0.8f;
        // }

        if (VT13.IsKeyPress(VT13_KEY_D))
        {
            increas_buffer[0] += 0.05;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            temp_vy = 0.5 * temp_Speed * increas_buffer[0];
        }
        else if (VT13.IsKeyPress(VT13_KEY_A))
        {
            increas_buffer[1] += 0.05;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            temp_vy = -0.5 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0.5;
            increas_buffer[1] = 0.5;
            temp_vy = 0;
        }

        if (VT13.IsKeyPress(VT13_KEY_S))
        {
            increas_buffer[2] += 0.05;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);

            temp_vx = -0.5 * temp_Speed * increas_buffer[2];
        }
        else if (VT13.IsKeyPress(VT13_KEY_W))
        {
            increas_buffer[3] += 0.05;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);

            temp_vx = 0.5 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0.5;
            increas_buffer[3] = 0.5;
            temp_vx = 0;
        }

        if (VT13.IsKeyPress(VT13_KEY_Q) && VT13.IsKeyPress(VT13_KEY_CTRL))
        {
            temp_wz = temp_Speed; // 左转
        }
        else if (VT13.IsKeyPress(VT13_KEY_Q) && VT13.IsKeyPress(VT13_KEY_SHIFT))
        {
            temp_wz = -temp_Speed; // 右转
        }
        else
        {
            temp_wz = -temp_Speed * VT13.Get_MouseX_Norm(); // 无按键时保持原有鼠标控制
        }
        
        // read remote data
        const float x = temp_vx;
        const float y = temp_vy;
        const float z = constrain(temp_wz, -chassis_max_spinning_speed, chassis_max_spinning_speed);
        const float magxy = sqrtf(x * x + y * y);
        // chassis control struct
        if (fabsf(magxy) < 1e-5)
        {
            chassis_control_struct.x = 0;
            chassis_control_struct.y = 0;
        }
        else
        {
            chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
        }
        chassis_control_struct.z = z;
        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
    else
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }

}

void VT13_Manipulater_JointCtrl()
{
    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_S)
    {
        if(VT13_IsButtonActive(ButtonType::FN1) && !VT13_IsButtonActive(ButtonType::FN2))
        {
            if (VT13.GetCh2() > 1324)
            {
                joint[0].ajustJointTarget(0.05);
            }
            if (VT13.GetCh2() < 724)
            {
                joint[0].ajustJointTarget(-0.05);
            }

            if (VT13.GetCh3() > 1324)
            {
                joint[1].ajustJointTarget(-0.01);
            }
            if (VT13.GetCh3() < 724)
            {
                joint[1].ajustJointTarget(0.01);
            }

            if (VT13.GetCh1() > 1324)
            {
                joint[2].ajustJointTarget(-0.02);
            }
            if (VT13.GetCh1() < 724)
            {
                joint[2].ajustJointTarget(0.02);
            }

            if(VT13.GetCh0() > 1324)
            {
                gripper.open();
            }
            else if(VT13.GetCh0() < 724)
            {
                gripper.close();
            }
            else
            {
                gripper.emergencyStop();
            }
        }
        else if(VT13_IsButtonActive(ButtonType::FN2) && !VT13_IsButtonActive(ButtonType::FN1))
        {
            if (VT13.GetCh2() > 1324)
            {
                joint[3].ajustJointTarget(0.1);
            }
            if (VT13.GetCh2() < 724)
            {
                joint[3].ajustJointTarget(-0.1);
            }

            if (VT13.GetCh3() > 1324)
            {
                joint[4].ajustJointTarget(-1.5);
            }
            if (VT13.GetCh3() < 724)
            {
                joint[4].ajustJointTarget(1.5);
            }

            if (VT13.GetCh1() > 1324)
            {
                joint[5].ajustJointTarget(1.5);
            }
            if (VT13.GetCh1() < 724)
            {
                joint[5].ajustJointTarget(-1.5);
            }

            if(VT13.GetCh0() > 1324)
            {
                gripper.open();
            }
            else if(VT13.GetCh0() < 724)
            {
                gripper.close();
            }
            else
            {
                gripper.emergencyStop();
            }
        }

    }
}

void VT13_Grip()
{
    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_S)
    {
        if(VT13_IsButtonActive(ButtonType::FN2) && !VT13_IsButtonActive(ButtonType::FN1))
        {
            if(VT13.GetCh0() > 1324)
            {
                gripper.open();
            }
            else if(VT13.GetCh0() < 724)
            {
                gripper.close();
            }
            else
            {
                gripper.emergencyStop();
            }
        }
      
    }
}

void VT13_Store(ToBottomDataStructdef &_ToBottomData)
{
    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_S)
    {
        if(VT13.GetTrigger())
        {
           if(VT13.GetDial() > 1324)
           {
                _ToBottomData.L_Store = 1;
           }
           else if(VT13.GetDial() < 724)
           {
                _ToBottomData.L_Store = 0;
           }
           else
           {
                // _ToBottomData.L_Store = 0;
           }
          
        }
        else if(!VT13.GetTrigger())
        {
            if(VT13.GetDial() > 1324)
           {
                _ToBottomData.R_Store = 1;
           }
           else if(VT13.GetDial() < 724)
           {
                _ToBottomData.R_Store = 0;
           }
           else
           {
                // _ToBottomData.R_Store = 0;
           }
          
        }
    }
    
}

void VT13_KeyBoard_Grip()
{
    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {
        bool gripperchanged = false;
        static uint8_t last_gripper_button_state = 0; // 0: 未按下，1: 按下
        uint8_t current_gripper_button_state = VT13.Datapack_custom.pick & 0x01; // 取最低位作为夹爪按钮状态
        uint8_t gripper_button_state_changed = current_gripper_button_state ^ last_gripper_button_state;

        if ((VT13.IsKeyPress(VT13_KEY_E) && !lastGripperState[0]) || gripper_button_state_changed)
        {
            GripperStates[0] = !GripperStates[0];
            gripperchanged = true;
        }

        lastGripperState[0] = VT13.IsKeyPress(VT13_KEY_E);
        last_gripper_button_state = current_gripper_button_state;

        if (gripperchanged)
        {
            if (gripper.getState() == GripperState::CLOSING)
            {
                gripper.open(); // 打开
            }
            else if (gripper.getState() == GripperState::OPENING || gripper.getState() == GripperState::IDLE)
            {
                gripper.close(); // 夹紧
            }
        }
    }
    else if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_C)
    {
        gripper.emergencyStop(); // 紧急停止
    }
}

void VT13_KeyboardStoreGrippers(ToBottomDataStructdef &_ToBottomData)
{
    // VT13储存能量单元
    if (VT13.GetModeSwitch() == MS_Status_Typedef::MS_S)
    {
        bool leftgripperchanged = false;
        bool rightgripperchanged = false;
        static uint8_t last_left_store_button_state = 0; // 0: 未按下，1: 按下
        static uint8_t last_right_store_button_state = 0; // 0: 未按下，1: 按下
        uint8_t current_left_store_button_state = VT13.Datapack_custom.L_Storage & 0x01; // 取最低位作为左侧储存按钮状态
        uint8_t current_right_store_button_state = VT13.Datapack_custom.R_Storage & 0x01; // 取最低位作为右侧储存按钮状态
        uint8_t left_store_button_state_changed = current_left_store_button_state ^ last_left_store_button_state;
        uint8_t right_store_button_state_changed = current_right_store_button_state ^ last_right_store_button_state;

        if (VT13.IsKeyPress(VT13_KEY_E) && !lastGripperState[2] || right_store_button_state_changed)
        {
            GripperStates[2] = !GripperStates[2];
            rightgripperchanged = true;
        }

        lastGripperState[2] = VT13.IsKeyPress(VT13_KEY_E);
        last_right_store_button_state = current_right_store_button_state;
        
        if (rightgripperchanged)
        {
            if (reBottomdata.R_Store == 0)
            {
                _ToBottomData.R_Store = 1; // 打开电磁铁
            }
            else if (reBottomdata.R_Store == 1)
            {
                _ToBottomData.R_Store = 0; // 关闭电磁铁
            }
        }

        if (VT13.IsKeyPress(VT13_KEY_Q) && !lastGripperState[1] || left_store_button_state_changed)
        {
            GripperStates[1] = !GripperStates[1];
            leftgripperchanged = true;
        }

        lastGripperState[1] = VT13.IsKeyPress(VT13_KEY_Q);
        last_left_store_button_state = current_left_store_button_state;

        if (leftgripperchanged)
        {
            if (reBottomdata.L_Store == 0)
            {
                _ToBottomData.L_Store = 1; // 打开电磁铁
            }
            else if (reBottomdata.L_Store == 1)
            {
                _ToBottomData.L_Store = 0; // 关闭电磁铁
            }
        }
    }
    else if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_C)
    {
        _ToBottomData.L_Store = 0;
        _ToBottomData.R_Store = 0;
    }
}

#endif

void KeyBoard_MoveCtrl(ToBottomDataStructdef &_ToBottomData)
{
    float temp_vx = 0.0f;
    float temp_vy = 0.0f;
    float temp_wz = 0.0f;

    float temp_Speed = 3.0f;

    static float increas_buffer[4] = {0, 0, 0, 0};
    static int commanded_chassis_state = 1;
    static int last_rebottom_chassis_state = 1;
    static bool last_f_key_state = false;
    static bool last_b_key_state = false;

    ChassisControlStruct chassis_control_struct = {};
    chassis_control_struct.rfid_height = _ToBottomData.chassis_control_struct.rfid_height;
    chassis_control_struct.chassis_height = _ToBottomData.chassis_control_struct.chassis_height;

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        const bool f_key_pressed = DR16.IsKeyPress(DR16_KEY_F);
        const bool b_key_pressed = DR16.IsKeyPress(DR16_KEY_B);
        const bool f_rising_edge = f_key_pressed && !last_f_key_state;
        const bool b_rising_edge = b_key_pressed && !last_b_key_state;

        if (commanded_chassis_state == 1)
        {
            if (f_rising_edge)
            {
                commanded_chassis_state = 3;
            }
            else if (b_rising_edge)
            {
                commanded_chassis_state = 4;
            }
        }
        else if ((last_rebottom_chassis_state == 3 || last_rebottom_chassis_state == 4) && reBottomdata.ChassisState == 1)
        {
            commanded_chassis_state = 1;
        }

        _ToBottomData.ChassisState = commanded_chassis_state;
        last_rebottom_chassis_state = reBottomdata.ChassisState;
        last_f_key_state = f_key_pressed;
        last_b_key_state = b_key_pressed;

        // if (DR16.IsKeyPress(DR16_KEY_SHIFT))
        // {
        //     temp_Speed = 3;
        // }
        // else if (DR16.IsKeyPress(DR16_KEY_CTRL))
        // {
        //     temp_Speed = 0.5;
        // }
        // else
        // {
            temp_Speed = 5;
        // }

        if (DR16.IsKeyPress(DR16_KEY_W))
        {
            increas_buffer[0] += 0.1;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            // increas_buffer[0] = 0.75;

            temp_vy = 1.0 * temp_Speed * increas_buffer[0];
        }
        else if (DR16.IsKeyPress(DR16_KEY_S))
        {
            increas_buffer[1] += 0.1;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            // increas_buffer[1] = 0.75;
            temp_vy = -1.0 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0;
            increas_buffer[1] = 0;
            temp_vy = 0;
        }

        if (DR16.IsKeyPress(DR16_KEY_D))
        {
            increas_buffer[2] += 0.1;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);
            // increas_buffer[2] = 0.75;

            temp_vx = +1.0 * temp_Speed * increas_buffer[2];
        }
        else if (DR16.IsKeyPress(DR16_KEY_A))
        {
            increas_buffer[3] += 0.1;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);
            //  increas_buffer[3] = 0.75;

            temp_vx = -1.0 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0;
            increas_buffer[3] = 0;
            temp_vx = 0;
        }

        temp_wz = -temp_Speed * DR16.Get_MouseX_Norm(); // 旋转速度

        // read remote data
        const float x = temp_vx;
        const float y = temp_vy;

        const float z = constrain(temp_wz, -chassis_max_spinning_speed, chassis_max_spinning_speed);

        const float magxy = sqrtf(x * x + y * y);
        // chassis control struct
        if (fabsf(magxy) < 1e-5)
        {
            chassis_control_struct.x = 0;
            chassis_control_struct.y = 0;
        }
        else
        {
            chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
        }
        chassis_control_struct.z = z;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
    else
    {
        _ToBottomData.ChassisState = 0;
        commanded_chassis_state = 1;
        last_rebottom_chassis_state = reBottomdata.ChassisState;
        last_f_key_state = false;
        last_b_key_state = false;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

/**
 * @brief 按键控制底盘移动，在自定义控制器状态下
 *
 */
void KeyBoard_MoveCtrl_Custom(ToBottomDataStructdef &_ToBottomData)
{
    float temp_vx = 0.0f;
    float temp_vy = 0.0f;
    float temp_wz = 0.0f;

    float temp_Speed = 1.0f;

    static float increas_buffer[4] = {0, 0, 0, 0};

    ChassisControlStruct chassis_control_struct = {};
    chassis_control_struct.rfid_height = _ToBottomData.chassis_control_struct.rfid_height;
    chassis_control_struct.chassis_height = _ToBottomData.chassis_control_struct.chassis_height;

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        _ToBottomData.ChassisState = 2;

        if (DR16.IsKeyPress(DR16_KEY_CTRL))
        {
            temp_Speed = 0.5;
        }
        else
        {
            temp_Speed = 1.1;
        }

        // Chassis_flag = 1; // 底盘跟随

        if (DR16.IsKeyPress(DR16_KEY_W))
        {
            increas_buffer[0] += 0.1;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            temp_vy = 1.0 * temp_Speed * increas_buffer[0];
        }
        else if (DR16.IsKeyPress(DR16_KEY_S))
        {
            increas_buffer[1] += 0.1;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            temp_vy = -1.0 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0.5;
            increas_buffer[1] = 0.5;
            temp_vy = 0;
        }

        if (DR16.IsKeyPress(DR16_KEY_D))
        {
            increas_buffer[2] += 0.1;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);

            temp_vx = +1.0 * temp_Speed * increas_buffer[2];
        }
        else if (DR16.IsKeyPress(DR16_KEY_A))
        {
            increas_buffer[3] += 0.1;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);

            temp_vx = -1.0 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0.5;
            increas_buffer[3] = 0.5;
            temp_vx = 0;
        }

        temp_vy = -temp_Speed * DR16.Get_MouseX_Norm(); // 旋转速度

        // read remote data
        const float x = temp_vx;
        const float y = temp_vy;

        const float z = constrain(temp_wz, -chassis_max_spinning_speed, chassis_max_spinning_speed);

        const float magxy = sqrtf(x * x + y * y);
        // chassis control struct
        if (fabsf(magxy) < 1e-5)
        {
            chassis_control_struct.x = 0;
            chassis_control_struct.y = 0;
        }
        else
        {
            chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
        }
        chassis_control_struct.z = z;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
    else
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

void KeyBoard_MoveCtrl_Ore(ToBottomDataStructdef &_ToBottomData)
{
    float temp_vx = 0.0f;
    float temp_vy = 0.0f;
    float temp_wz = 0.0f;

    static float increas_buffer[4] = {0, 0, 0, 0};

    float temp_Speed = 1.0f;

    ChassisControlStruct chassis_control_struct = {};

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        _ToBottomData.ChassisState = 1;

        if (DR16.IsKeyPress(DR16_KEY_CTRL))
        {
            temp_Speed = 0.5;
        }
        else
        {
            temp_Speed = 1.5;
        }

        if (DR16.IsKeyPress(DR16_KEY_W))
        {
            increas_buffer[0] += 0.1;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            temp_vy = 0.9 * temp_Speed * increas_buffer[0];
        }
        else if (DR16.IsKeyPress(DR16_KEY_S))
        {
            increas_buffer[1] += 0.1;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            temp_vy = -0.9 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0.5;
            increas_buffer[1] = 0.5;
            temp_vy = 0;
        }

        if (DR16.IsKeyPress(DR16_KEY_D))
        {
            increas_buffer[2] += 0.1;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);

            temp_vx = +0.9 * temp_Speed * increas_buffer[2];
        }
        else if (DR16.IsKeyPress(DR16_KEY_A))
        {
            increas_buffer[3] += 0.1;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);

            temp_vx = -0.9 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0.5;
            increas_buffer[3] = 0.5;
            temp_vx = 0;
        }

        temp_wz = -temp_Speed * DR16.Get_MouseX_Norm(); // 旋转速度

        // read remote data
        const float x = temp_vx;
        const float y = temp_vy;

        const float z = constrain(temp_wz, -chassis_max_spinning_speed, chassis_max_spinning_speed);

        const float magxy = sqrtf(x * x + y * y);
        // chassis control struct
        if (fabsf(magxy) < 1e-5)
        {
            chassis_control_struct.x = 0;
            chassis_control_struct.y = 0;
        }
        else
        {
            chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
        }
        chassis_control_struct.z = z;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
    else
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

void DR16_Manipulater_JointCtrl()
{
    static LinkageStatus_Typedef last_link_status = LinkageStatus_Typedef::LOST;

    /*具体执行*/
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {

        /*机械臂的Control*/
        if (last_link_status == LinkageStatus_Typedef::LOST)
        {
            // 使能
            for (uint8_t i = 0; i < 6; i++)
            {
                joint[i].setMode(JointMode::angle);
            }
        }

        if (DR16.GetS1() == 1 && DR16.GetS2() == 2) // 左上右下
        {
            if (DR16.GetCh2() > 1324)
            {
                joint[0].ajustJointTarget(0.05);
            }
            if (DR16.GetCh2() < 724)
            {
                joint[0].ajustJointTarget(-0.05);
            }

            if (DR16.GetCh3() > 1324)
            {
                joint[1].ajustJointTarget(-0.1);
            }
            if (DR16.GetCh3() < 724)
            {
                joint[1].ajustJointTarget(0.1);
            }

            if (DR16.GetCh1() > 1324)
            {
                joint[2].ajustJointTarget(-0.1);
            }
            if (DR16.GetCh1() < 724)
            {
                joint[2].ajustJointTarget(0.1);
            }

            if (DR16.GetCh0() > 1324)
            {
                gripper.open();
            }
            if (DR16.GetCh0() < 724)
            {
                gripper.close();
            }
        }
        if (DR16.GetS1() == 1 && DR16.GetS2() == 3)
        {
            if (DR16.GetCh2() > 1324)
            {
                joint[3].ajustJointTarget(0.1);
            }
            if (DR16.GetCh2() < 724)
            {
                joint[3].ajustJointTarget(-0.1);
            }

            if (DR16.GetCh3() > 1324)
            {
                joint[4].ajustJointTarget(-1.5);
            }
            if (DR16.GetCh3() < 724)
            {
                joint[4].ajustJointTarget(1.5);
            }

            if (DR16.GetCh1() > 1324)
            {
                joint[5].ajustJointTarget(1.5);
            }
            if (DR16.GetCh1() < 724)
            {
                joint[5].ajustJointTarget(-1.5);
            }

            if (DR16.GetCh0() > 1324)
            {
                gripper.open();
            }
            if (DR16.GetCh0() < 724)
            {
                gripper.close();
            }
        }
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            joint[i].setMode(JointMode::disabled);
        }
    }

    last_link_status = DR16.GetStatus(); // 更新
}

/**
 * @brief 用于测试电机pid
 */
void DR16_Debug_Ctrl()
{

    static LinkageStatus_Typedef last_link_status = LinkageStatus_Typedef::LOST;
    static uint8_t one = 0;

    // motor[4].setPidParameters({speed_kp1, -speed_Out_Max1, speed_Out_Max1, angle_kp1, angle_ki1, angle_kd1, -angle_Out_Max1, angle_Out_Max1, 0, angle_I_Max1});
    // motor[5].setPidParameters({speed_kp, -speed_Out_Max, speed_Out_Max, angle_kp, angle_ki, angle_kd, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
    // motor[3].setPidParameters({speed_kp, -speed_Out_Max, speed_Out_Max, angle_kp, angle_ki, angle_kd, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
    // motor[2].setPidParameters({speed_kp, -speed_Out_Max, speed_Out_Max, angle_kp, angle_ki, angle_kd, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
    // motor[1].setPidParameters({speed_kp, -speed_Out_Max, speed_Out_Max, angle_kp, angle_ki, angle_kd, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
    // motor[0].setPidParameters({speed_kp, -speed_Out_Max, speed_Out_Max, angle_kp, angle_ki, angle_kd, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
    /*具体执行*/
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {

        /*机械臂的Control*/
        if (last_link_status == LinkageStatus_Typedef::LOST)
        {
            // 使能
            for (uint8_t i = 0; i < 6; i++)
            {
                joint[i].setMode(JointMode::angle);
            }
        }

        if (DR16.GetS1() == 1 && DR16.GetS2() == 2) // 左上右下
        {
            if (DR16.GetCh2() > 1324)
            {
                joint[0].ajustJointTarget(0.05);
            }
            if (DR16.GetCh2() < 724)
            {
                joint[0].ajustJointTarget(-0.05);
            }

            if (DR16.GetCh3() > 1324)
            {
                joint[1].ajustJointTarget(-0.01);
            }
            if (DR16.GetCh3() < 724)
            {
                joint[1].ajustJointTarget(0.01);
            }

            if (DR16.GetCh1() > 1324)
            {
                joint[2].ajustJointTarget(-0.02);
            }
            if (DR16.GetCh1() < 724)
            {
                joint[2].ajustJointTarget(0.02);
            }
        }
        if (DR16.GetS1() == 1 && DR16.GetS2() == 3)
        {
            if (DR16.GetCh2() > 1324)
            {
                joint[3].ajustJointTarget(0.1);
            }
            if (DR16.GetCh2() < 724)
            {
                joint[3].ajustJointTarget(-0.1);
            }

            if (DR16.GetCh3() > 1324)
            {
                joint[4].ajustJointTarget(-1.5);
            }
            if (DR16.GetCh3() < 724)
            {
                joint[4].ajustJointTarget(1.5);
            }

            if (DR16.GetCh1() > 1324)
            {
                joint[5].ajustJointTarget(1.5);
            }
            if (DR16.GetCh1() < 724)
            {
                joint[5].ajustJointTarget(-1.5);
            }
        }

        // joint[5].setTargetAngle(debug_target_angle);
        joint[4].setTargetAngle(debug_target_angle);
        // joint[3].setTargetAngle(debug_target_angle);
        // joint[2].setTargetAngle(debug_target_angle);
        // joint[1].setTargetAngle(debug_target_angle);
        // joint[0].setTargetAngle(debug_target_angle);
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            joint[i].setMode(JointMode::disabled);
        }

        // if (motor[5].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = joint[5].getCurrentAngle();
        // }
        if (motor[5].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready)
        {
            debug_target_angle = joint[4].getCurrentAngle();
        }
        // if (motor[3].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = joint[3].getCurrentAngle();
        // }
        // if (motor[2].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = joint[2].getCurrentAngle();
        // }
        // if (motor[1].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = joint[1].getCurrentAngle();
        // }
        // if (motor[0].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = joint[0].getCurrentAngle();
        // }
    }

    last_link_status = DR16.GetStatus(); // 更新
}

void DR16_MoveCtrl(ToBottomDataStructdef &_ToBottomData)
{
    ChassisControlStruct chassis_control_struct = {};
    static float target_chassis_height_m = chassis_height_min;

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == SW_DOWN && DR16.GetS2() == SW_DOWN) // S1下，S2下
        {
            _ToBottomData.ChassisState = 1;

            // UpdateChassisCurrentHeight();

            // if (DR16.GetCh1() > 1324)
            // {
            //     target_chassis_height_m += 0.01f;
            // }
            // else if (DR16.GetCh1() < 724)
            // {
            //     target_chassis_height_m -= 0.01f;
            // }

            // target_chassis_height_m = constrain(target_chassis_height_m, chassis_height_min, chassis_height_max);
            // ControlChassisHeight(target_chassis_height_m, chassis_control_struct);

            // read remote data
            const float y = map(deadband(DR16.GetCh3(), 1024, 50), 364, 1684, -chassis_max_translation_speed, chassis_max_translation_speed);
            const float x = map(deadband(DR16.GetCh2(), 1024, 50), 364, 1684, -chassis_max_translation_speed, chassis_max_translation_speed);
            const float z = map(deadband(DR16.GetCh0(), 1024, 50), 364, 1684, chassis_max_spinning_speed, -chassis_max_spinning_speed);
            const float magxy = sqrtf(x * x + y * y);
            // chassis control struct
            if (fabsf(magxy) < 1e-5)
            {
                chassis_control_struct.x = 0;
                chassis_control_struct.y = 0;
            }
            else
            {
                chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
                chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            }
            chassis_control_struct.z = z;
            _ToBottomData.chassis_control_struct = chassis_control_struct;
        }
        else
        {
            // UpdateChassisCurrentHeight();
            // target_chassis_height_m = current_chassis_height;
        }
        
    }
    else
    {
        // UpdateChassisCurrentHeight();
        // target_chassis_height_m = current_chassis_height;
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;
        // chassis_control_struct.chassis_height = current_chassis_height;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

void DR16_MoveCtrl_Ore(ToBottomDataStructdef &_ToBottomData)
{
    ChassisControlStruct chassis_control_struct = {};

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == 2 && DR16.GetS2() == 2) // S1中，S2上
        {
            _ToBottomData.ChassisState = 1;

            // read remote data
            const float x = map(deadband(DR16.GetCh3(), 1024, 50), 364, 1684, -1.0f, 1.0f);
            const float y = map(deadband(DR16.GetCh2(), 1024, 50), 364, 1684, 1.0f, -1.0f);
            const float z = map(deadband(DR16.GetCh0(), 1024, 50), 364, 1684, chassis_max_spinning_speed, -chassis_max_spinning_speed);
            const float magxy = sqrtf(x * x + y * y);
            // chassis control struct
            if (fabsf(magxy) < 1e-5)
            {
                chassis_control_struct.x = 0;
                chassis_control_struct.y = 0;
            }
            else
            {
                chassis_control_struct.x = x * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
                chassis_control_struct.y = y * constrain(chassis_max_translation_speed / magxy, 0.0f, 1.0f);
            }
            chassis_control_struct.z = z;

            _ToBottomData.chassis_control_struct = chassis_control_struct;
        }
    }
    else
    {
        _ToBottomData.ChassisState = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

        _ToBottomData.chassis_control_struct = chassis_control_struct;
    }
}

void DR16_Grip()
{
    if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if ((DR16.GetS1() == 3 && DR16.GetS2() == 1)) // 开夹爪 S1中，S2上
        {
            gripper.open();
        }
        else if ((DR16.GetS1() == 3 && DR16.GetS2() == 3)) // 停止夹爪 S1中，S2中
        {
            gripper.close();
        }
        else if(DR16.GetS1() == 3 && DR16.GetS2() == 2) // 关夹爪 S1中，S2下
        {
            gripper.close();
        }
    }
    else
    {
        gripper.emergencyStop();
    }
}

void Keyboard_Grippers()
{
    if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        bool gripperchanged = false;
        if (DR16.IsKeyPress(DR16_KEY_E) && !lastGripperState[0])
        {
            GripperStates[0] = !GripperStates[0];
            gripperchanged = true;
        }

        lastGripperState[0] = DR16.IsKeyPress(DR16_KEY_E);

        if (gripperchanged)
        {
            if (gripper.getState() == GripperState::CLOSING || gripper.getState() == GripperState::IDLE)
            {
                gripper.open(); // 打开
            }
            else if (gripper.getState() == GripperState::OPENING)
            {
                gripper.close(); // 关闭
            }
        }
    }
    else
    {
        gripper.emergencyStop();
    }
}

/**
 * @brief DR16储存能量单元
 */
void DR16_Store(ToBottomDataStructdef &_ToBottomData)
{
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == 2 && DR16.GetS2() == 3)  // 左下右中
        {
            if (DR16.GetCh3() > 1324)
            {
                _ToBottomData.L_Store = 0; // 左电磁铁关
            }
            else if (DR16.GetCh3() < 724)
            {
                _ToBottomData.L_Store = 1; // 左电磁铁开
            }
            else
            {
                // _ToBottomData.L_Store = 0;
            }

            if (DR16.GetCh1() > 1324)
            {
                _ToBottomData.R_Store = 0; // 右电磁铁关
            }
            else if (DR16.GetCh1() < 724)
            {
                _ToBottomData.R_Store = 1; // 右电磁铁开
            }
            else
            {
                // _ToBottomData.R_Store = 0;
            }

        }
        else
        {
            // _ToBottomData.L_Store = 0;
            // _ToBottomData.R_Store = 0;
        }
    }
    else
    {
        // _ToBottomData.L_Store = 0;
        // _ToBottomData.R_Store = 0;
    }
}

void Keyboard_Store(ToBottomDataStructdef &_ToBottomData)
{
    if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == 2 && DR16.GetS2() == 3)  // 左下右中
        {
            bool leftgripperchanged = false;
            bool rightgripperchanged = false;
            if (DR16.IsKeyPress(DR16_KEY_E) && !lastGripperState[2])
            {
                GripperStates[2] = !GripperStates[2];
                rightgripperchanged = true;
            }

            lastGripperState[2] = DR16.IsKeyPress(DR16_KEY_E);
            
            if (rightgripperchanged)
            {
                if (reBottomdata.R_Store == 0)
                {
                    _ToBottomData.R_Store = 1; // 打开电磁铁
                }
                else if (reBottomdata.R_Store == 1)
                {
                    _ToBottomData.R_Store = 0; // 关闭电磁铁
                }
            }

            if (DR16.IsKeyPress(DR16_KEY_Q) && !lastGripperState[1])
            {
                GripperStates[1] = !GripperStates[1];
                leftgripperchanged = true;
            }

            lastGripperState[1] = DR16.IsKeyPress(DR16_KEY_Q);

            if (leftgripperchanged)
            {
                if (reBottomdata.L_Store == 0)
                {
                    _ToBottomData.L_Store = 1; // 打开电磁铁
                }
                else if (reBottomdata.L_Store == 1)
                {
                    _ToBottomData.L_Store = 0; // 关闭电磁铁
                }
            }
        }    
        
    }
    else
    {
        // _ToBottomData.L_Store = 0;
        // _ToBottomData.R_Store = 0;
    }
}
