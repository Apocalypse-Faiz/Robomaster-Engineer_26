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
void VT13_KeyBoard_MoveCtrl(ToBottomDataStructdef &_ToBottomData)
{
    float temp_vx = 0.0f;
    float temp_vy = 0.0f;
    float temp_wz = 0.0f;

    float temp_Speed = 5.0f;

    static float increas_buffer[4] = {0, 0, 0, 0};

    ChassisControlStruct chassis_control_struct = {};

    if (VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {
        _ToBottomData.is_ctrl = 1;

        if (VT13.IsKeyPress(VT13_KEY_SHIFT))
        {
            temp_Speed = 1.5;
        }
        else if (VT13.IsKeyPress(VT13_KEY_CTRL))
        {
            temp_Speed = 0.5;
        }
        else
        {
            temp_Speed = 5.0f;
        }

        // if (VT13.IsKeyPress(VT13_KEY_R)) // 变速小陀螺
        // {
        //     Chassis_flag = 0;
        //     temp_wz = 4;
        // }
        // else // 底盘跟随
        // {
        //     Chassis_flag = 1;
        // }

        if (VT13.IsKeyPress(VT13_KEY_W))
        {
            increas_buffer[0] += 0.1;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            temp_vy = 1.3 * temp_Speed * increas_buffer[0];
        }
        else if (VT13.IsKeyPress(VT13_KEY_S))
        {
            increas_buffer[1] += 0.1;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            temp_vy = -1.3 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0.5;
            increas_buffer[1] = 0.5;
            temp_vy = 0;
        }

        if (VT13.IsKeyPress(VT13_KEY_A))
        {
            increas_buffer[2] += 0.1;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);

            temp_vx = -1.3 * temp_Speed * increas_buffer[2];
        }
        else if (VT13.IsKeyPress(VT13_KEY_D))
        {
            increas_buffer[3] += 0.1;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);

            temp_vx = 1.3 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0.5;
            increas_buffer[3] = 0.5;
            temp_vx = 0;
        }

        temp_wz = -temp_Speed * VT13.Get_MouseX_Norm();

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
        _ToBottomData.is_ctrl = 0;
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

        _ToBottomData.is_ctrl = 1;

        // read remote data
        const float x = map(deadband(VT13.GetCh3(), 1024, 50), 364, 1684, -chassis_max_translation_speed, chassis_max_translation_speed);
        const float y = map(deadband(VT13.GetCh2(), 1024, 50), 364, 1684, -chassis_max_translation_speed, chassis_max_translation_speed);
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
        _ToBottomData.is_ctrl = 0;
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

    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {
        _ToBottomData.is_ctrl = 1;

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
            temp_Speed = 1.5;
        // }

        if (VT13.IsKeyPress(VT13_KEY_W))
        {
            increas_buffer[0] += 0.1;
            increas_buffer[0] = constrain(increas_buffer[0], 0.5, 1);
            temp_vy = 1.3 * temp_Speed * increas_buffer[0];
        }
        else if (VT13.IsKeyPress(VT13_KEY_S))
        {
            increas_buffer[1] += 0.1;
            increas_buffer[1] = constrain(increas_buffer[1], 0.5, 1);
            temp_vy = -1.3 * temp_Speed * increas_buffer[1];
        }
        else
        {
            increas_buffer[0] = 0.5;
            increas_buffer[1] = 0.5;
            temp_vy = 0;
        }

        if (VT13.IsKeyPress(VT13_KEY_A))
        {
            increas_buffer[2] += 0.1;
            increas_buffer[2] = constrain(increas_buffer[2], 0.5, 1);

            temp_vx = -1.3 * temp_Speed * increas_buffer[2];
        }
        else if (VT13.IsKeyPress(VT13_KEY_D))
        {
            increas_buffer[3] += 0.1;
            increas_buffer[3] = constrain(increas_buffer[3], 0.5, 1);

            temp_vx = 1.3 * temp_Speed * increas_buffer[3];
        }
        else
        {
            increas_buffer[2] = 0.5;
            increas_buffer[3] = 0.5;
            temp_vx = 0;
        }

        if (VT13.IsKeyPress(VT13_KEY_R))
        {
            temp_wz = -temp_Speed;
        }
        else if (VT13.IsKeyPress(VT13_KEY_R) && VT13.IsKeyPress(VT13_KEY_SHIFT))
        {
            temp_wz = temp_Speed;
        }
        else 
        {
            temp_wz = -temp_Speed * VT13.Get_MouseX_Norm();
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
        _ToBottomData.is_ctrl = 0;
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
                joint[1].ajustJointTarget(-10);
            }
            if (VT13.GetCh3() < 724)
            {
                joint[1].ajustJointTarget(10);
            }

            if (VT13.GetCh1() > 1324)
            {
                joint[2].ajustJointTarget(-10);
            }
            if (VT13.GetCh1() < 724)
            {
                joint[2].ajustJointTarget(10);
            }

            if(VT13.GetCh0() > 1324)
            {
                gripper.open(4500.0f);
            }
            else if(VT13.GetCh0() < 724)
            {
                gripper.close(4500.0f);
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
                gripper.open(4500.0f);
            }
            else if(VT13.GetCh0() < 724)
            {
                gripper.close(4500.0f);
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
                gripper.open(4500.0f);
            }
            else if(VT13.GetCh0() < 724)
            {
                gripper.close(4500.0f);
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
                _ToBottomData.L_Store = 2;
           }
           else
           {
                _ToBottomData.L_Store = 0;
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
                _ToBottomData.R_Store = 2;
           }
           else
           {
                _ToBottomData.R_Store = 0;
           }
          
        }
    }
    
}

void VT13_KeyBoard_Grip()
{
    if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_N)
    {
        bool gripperchanged = false;
        if (VT13.IsKeyPress(VT13_KEY_E) && !lastGripperState[0])
        {
            GripperStates[0] = !GripperStates[0];
            gripperchanged = true;
        }

        lastGripperState[0] = VT13.IsKeyPress(VT13_KEY_E);

        if (gripperchanged)
        {
            if (gripper.getState() == GripperState::CLOSING || gripper.getState() == GripperState::IDLE)
            {
                gripper.open(4500.0f); // 打开
            }
            else if (gripper.getState() == GripperState::OPENING)
            {
                gripper.close(4500.0f); // 夹紧
            }
        }
    }
    else if(VT13.GetModeSwitch() == MS_Status_Typedef::MS_C)
    {
        gripper.emergencyStop(); // 停止
    }
}

void VT13_KeyboardStoreGrippers(ToBottomDataStructdef &_ToBottomData)
{
    // VT13储存能量单元
    if (VT13.GetModeSwitch() == MS_Status_Typedef::MS_S)
    {
        bool leftgripperchanged = false;
        bool rightgripperchanged = false;
        if (VT13.IsKeyPress(VT13_KEY_E) && !lastGripperState[2])
        {
            GripperStates[2] = !GripperStates[2];
            rightgripperchanged = true;
        }

        lastGripperState[2] = VT13.IsKeyPress(VT13_KEY_E);
        
        if (rightgripperchanged)
        {
            if (reBottomdata.R_Store == 2 || reBottomdata.R_Store == 0)
            {
                _ToBottomData.R_Store = 1; // 打开
            }
            else if (reBottomdata.R_Store == 1)
            {
                _ToBottomData.R_Store = 2; // 夹紧
            }
        }

        if (VT13.IsKeyPress(VT13_KEY_Q) && !lastGripperState[1])
        {
            GripperStates[1] = !GripperStates[1];
            leftgripperchanged = true;
        }

        lastGripperState[1] = VT13.IsKeyPress(VT13_KEY_Q);

        if (leftgripperchanged)
        {
            if (reBottomdata.L_Store == 2 || reBottomdata.L_Store == 0)
            {
                _ToBottomData.L_Store = 1; // 打开
            }
            else if (reBottomdata.L_Store == 1)
            {
                _ToBottomData.L_Store = 2; // 夹紧
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

    ChassisControlStruct chassis_control_struct = {};

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        _ToBottomData.is_ctrl = 1;

        if (DR16.IsKeyPress(DR16_KEY_SHIFT))
        {
            temp_Speed = 3;
        }
        else if (DR16.IsKeyPress(DR16_KEY_CTRL))
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
        _ToBottomData.is_ctrl = 0;
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

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        _ToBottomData.is_ctrl = 1;

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
        _ToBottomData.is_ctrl = 0;
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
        _ToBottomData.is_ctrl = 1;

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
        _ToBottomData.is_ctrl = 0;
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
                joint[1].ajustJointTarget(-10);
            }
            if (DR16.GetCh3() < 724)
            {
                joint[1].ajustJointTarget(10);
            }

            if (DR16.GetCh1() > 1324)
            {
                joint[2].ajustJointTarget(-10);
            }
            if (DR16.GetCh1() < 724)
            {
                joint[2].ajustJointTarget(10);
            }

            if (DR16.GetCh0() > 1324)
            {
                gripper.open(5000);
            }
            if (DR16.GetCh0() < 724)
            {
                gripper.close(5000);
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
                gripper.open(5000);
            }
            if (DR16.GetCh0() < 724)
            {
                gripper.close(5000);
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
 * @brief 用于调试电机pid
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
    // motor[1].setPidParameters({0, -15000, 15000, 0, 0, 0, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
    // motor[2].setPidParameters({0, -15000, 15000, 0, 0, 0, -angle_Out_Max, angle_Out_Max, 0, angle_I_Max});
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

        if (DR16.GetS1() == 1 && DR16.GetS2() == 2) // 全为上
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

        // motor[4].setTarget(debug_target_angle1);
        // motor[5].setTarget(debug_target_angle);
        // motor[3].setTarget(debug_target_angle);
        motor[2].setTarget(debug_target_angle);
        // motor[1].setTarget(debug_target_angle);
        // motor[0].setTarget(debug_target_angle);
    }
    else
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            joint[i].setMode(JointMode::disabled);
        }

        // if (motor[4].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle1 = motor[4].getAngle();
        // }
        // if (motor[5].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = motor[5].getAngle();
        // }
        // if (motor[3].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = motor[3].getAngle();
        // }
        if (motor[2].getStatus() == MotorStatus::ready)
        {
            debug_target_angle = motor[2].getAngle();
        }
        // if (motor[1].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = motor[1].getAngle();
        // }
        // if (motor[0].getStatus() == MotorStatus::ready)
        // {
        //     debug_target_angle = motor[0].getAngle();
        // }
    }

    last_link_status = DR16.GetStatus(); // 更新
}

void DR16_MoveCtrl(ToBottomDataStructdef &_ToBottomData)
{
    ChassisControlStruct chassis_control_struct = {};

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == SW_DOWN && DR16.GetS2() == SW_DOWN) // S1下，S2下
        {
            _ToBottomData.is_ctrl = 1;

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
        
    }
    else
    {
        _ToBottomData.is_ctrl = 0;
        chassis_control_struct.x = 0;
        chassis_control_struct.y = 0;
        chassis_control_struct.z = 0;

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
            _ToBottomData.is_ctrl = 1;

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
        _ToBottomData.is_ctrl = 0;
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
            gripper.open(4500.0f);
        }
        else if ((DR16.GetS1() == 3 && DR16.GetS2() == 3)) // 停止夹爪 S1中，S2中
        {
            gripper.close(0.0f);
        }
        else if(DR16.GetS1() == 3 && DR16.GetS2() == 2) // 关夹爪 S1中，S2下
        {
            gripper.close(5000.0f);
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
                gripper.open(4500.0f); // 打开
            }
            else if (gripper.getState() == GripperState::OPENING)
            {
                gripper.close(4500.0f); // 夹紧
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
                _ToBottomData.L_Store = 2; // 左夹爪抓紧
            }
            else if (DR16.GetCh3() < 724)
            {
                _ToBottomData.L_Store = 1; // 左夹爪松开
            }
            else
            {
                _ToBottomData.L_Store = 0;
            }

            if (DR16.GetCh1() > 1324)
            {
                _ToBottomData.R_Store = 2;
            }
            else if (DR16.GetCh1() < 724)
            {
                _ToBottomData.R_Store = 1;
            }
            else
            {
                _ToBottomData.R_Store = 0;
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
        _ToBottomData.L_Store = 0;
        _ToBottomData.R_Store = 0;
    }
}

void Keyboard_Store(ToBottomDataStructdef &_ToBottomData)
{
    if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
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
            if (reBottomdata.R_Store == 2 || reBottomdata.R_Store == 0)
            {
                _ToBottomData.R_Store = 1; // 打开
            }
            else if (reBottomdata.R_Store == 1)
            {
                _ToBottomData.R_Store = 2; // 夹紧
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
            if (reBottomdata.L_Store == 2 || reBottomdata.L_Store == 0)
            {
                _ToBottomData.L_Store = 1; // 打开
            }
            else if (reBottomdata.L_Store == 1)
            {
                _ToBottomData.L_Store = 2; // 夹紧
            }
        }
        
    }
    else
    {
        _ToBottomData.L_Store = 0;
        _ToBottomData.R_Store = 0;
    }
}
