#include "Claw.h"

void Gripper::init(float grip_current_threshold, float max_grip_torque, float reset_torque, 
                   float reduction_ratio, float bias, float max_position, float min_position) 
{
    this->grip_current_threshold = grip_current_threshold;
    this->max_grip_torque = max_grip_torque;
    this->reset_torque = reset_torque;
    this->reduction_ratio = reduction_ratio;
    this->bias = bias;
    this->max_position = max_position;
    this->min_position = min_position;
    this->reseted = false;
}

void Gripper::update() 
{
    if(!motor || motor->getStatus() != MotorStatus::ready) 
    {
        state = GripperState::ERROR;
        return;
    }
    
    // 只更新电机传感器数据，不执行控制计算
    motor->update();
    updateCurrentPosition();
}

void Gripper::updateControl() 
{
    if(state == GripperState::ERROR) 
    {
        calculated_torque = 0.0f;
        new_torque_available = true;
        return;
    }
    
    // 检查机械限位（如果已复位）
    if(reseted && checkMechanicalLimit()) 
    {
        emergencyStop();
        calculated_torque = 0.0f;
        new_torque_available = true;
        return;
    }
    
    // 根据状态计算控制力矩
    switch(state) 
    {
        case GripperState::OPENING:
        case GripperState::CLOSING:
            // 如果已复位，可以进行物体夹持检测
            if(reseted && checkObjectGripped()) 
            {
                state = GripperState::GRIPPED;
                calculated_torque = target_torque * 0.8f;  // 夹住后减小力矩保持
            } 
            else 
            {
                // 根据是否已复位选择不同的力矩计算方法
                if(reseted) 
                {
                    calculated_torque = calculateControlTorque();  // 已复位：使用完整控制
                }
                else 
                {
                    calculated_torque = calculateControlTorqueUncalibrated();  // 未复位：使用简化控制
                }
            }
            break;
            
        case GripperState::RESETTING:
            // 检查是否到达复位位置
            if(abs(motor->current_torque) >= reset_torque) 
            {
                state = GripperState::IDLE;
                calculated_torque = 0.0f;
                calibrate();
                motor->setMode(MotorMode::torque);
                reseted = true;
            } 
            else 
            {
                if(motor->getMode() != MotorMode::angle)
                {
                    motor->setMode(MotorMode::angle);
                    reset_start_angle_offset = motor->getAngle();
                }
                motor->setTarget(reset_start_angle_offset += 15.0f);
            }
            break;
            
        case GripperState::GRIPPED:
            // 保持夹持力矩（只有在已复位时才能进入此状态）
            calculated_torque = target_torque * 0.8f;
            break;
            
        case GripperState::IDLE:
        default:
            calculated_torque = 0.0f;
            break;
    }
    
    // 限制力矩范围
    calculated_torque = std_lib::constrain(calculated_torque, -max_grip_torque, max_grip_torque);
    new_torque_available = true;
}

void Gripper::executeOutput() 
{
    if(!motor || motor->getStatus() != MotorStatus::ready) 
    {
        return;
    }
    
    if (motor->getMode() == MotorMode::angle)
    {
        motor->execute();
    }
    else if (motor->getMode() == MotorMode::torque)
    {
        if(new_torque_available) 
        {
            motor->setTarget(calculated_torque);
            motor->execute();
            new_torque_available = false;
        }
    }
}

void Gripper::open(float torque) 
{
    if(state == GripperState::ERROR) return;
    
    // 如果当前正在关闭且有物体，需要先释放
    if(state == GripperState::GRIPPED) 
    {
        object_detected = false;
        grip_confirmation_count = 0;
    }
    
    target_torque = std::abs(torque);  // 正力矩表示打开方向
    state = GripperState::OPENING;
}

void Gripper::close(float torque) 
{
    if(state == GripperState::ERROR) return;
    
    target_torque = -std::min(std::abs(torque), max_grip_torque);  // 负力矩表示关闭方向
    state = GripperState::CLOSING;
    
    // 重置夹持检测计数器
    grip_confirmation_count = 0;
}

void Gripper::torqueReset() 
{
    if(state == GripperState::ERROR) return;
    
    state = GripperState::RESETTING;
    object_detected = false;
    grip_confirmation_count = 0;
}

void Gripper::emergencyStop() 
{
    target_torque = 0.0f;
    calculated_torque = 0.0f;
    state = GripperState::IDLE;
    new_torque_available = true;
}

float Gripper::calculateControlTorque() 
{
    // 已复位状态下的力矩控制
    float torque = target_torque;
    
    // 添加位置软限位保护
    float position_margin = 0.1f; // 10%的裕量
    if(current_position < min_position + (max_position - min_position) * position_margin && torque > 0) 
    {
        torque *= 0.5f; // 关夹爪时接近下限位时减小关闭力矩
    }
    if(current_position > max_position - (max_position - min_position) * position_margin && torque < 0) 
    {
        torque *= 0.5f; // 开夹爪时接近上限位时减小打开力矩
    }
    
    return torque;
}

float Gripper::calculateControlTorqueUncalibrated() 
{
    // 未复位状态下的简化力矩控制
    // 仅使用基本的力矩限制，不进行位置保护
    return target_torque;
}

void Gripper::setGripCurrentThreshold(float threshold) 
{
    grip_current_threshold = std::abs(threshold);
}

GripperState Gripper::getState() const 
{
    return state;
}

bool Gripper::isObjectGripped() const 
{
    return object_detected;
}

float Gripper::getCurrentPosition() const 
{
    return current_position;
}

float Gripper::getCurrentCurrent() const 
{
    return motor ? motor->getTorque() : 0.0f;
}

void Gripper::clearError() 
{
    if(state == GripperState::ERROR) 
    {
        state = GripperState::IDLE;
    }
}

bool Gripper::checkMechanicalLimit() 
{
    // 未复位时不检查机械限位
    if(!reseted) return false;
    
    if(current_position <= min_position && target_torque < 0) 
    {
        return true;  // 达到最小限位但仍试图关闭
    }
    if(current_position >= max_position && target_torque > 0) 
    {
        return true;  // 达到最大限位但仍试图打开
    }
    return false;
}

bool Gripper::checkObjectGripped() 
{
    // 未复位时不进行物体夹持检测
    if(!reseted) return false;
    
    float current_current = getCurrentCurrent();
    
    // 如果电流超过阈值，认为可能夹住物体
    if(std::abs(current_current) > grip_current_threshold && target_torque > 0) 
    {
        grip_confirmation_count++;
        // 连续多次检测到高电流才确认夹住
        if(grip_confirmation_count >= 3) 
        {
            object_detected = true;
            return true;
        }
    } 
    else 
    {
        grip_confirmation_count = 0;
    }
    
    return false;
}

void Gripper::updateCurrentPosition() 
{
    if(motor) 
    {
        // 根据减速比转换电机角度到夹爪角度
        current_position = motor->getAngle() * reduction_ratio + bias;
    }
}

void Gripper::calibrate()
{
    if (motor)
    {
        motor->calibrate();
    }
}

void Gripper::setState(GripperState state)
{
    this->state = state;
}
