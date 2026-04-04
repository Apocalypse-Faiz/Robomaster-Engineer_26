#include "Motor.h"

void Motor::setPidParameters(MotorPidParameters param)
{
    a_pid.SetPIDParam(param.a_Kp, param.a_Ki, param.a_Kd, param.a_IntMax, param.a_Max);
    v_pid.SetPIDParam(param.v_Kp, 0.0f, 0.0f, 0.0f, param.v_Max);
}

/**
 * @brief 设置重力前馈值
 */
void Motor::setGravity_feedforward(float _data)
{
    Gravity_feedforward = _data;
}

/**
 * @brief 设置反电动势前馈值
 */
void Motor::setBackEMF_feedforward(float _data)
{
    BackEMF_feedforward = _data;
}

/**
 * @brief 设置库伦摩擦前馈值
 */
void Motor::setCoulomb_feedforward(float _data)
{
    Coulomb_feedforward = _data;
}

/**
 * @brief 设置黏性摩擦前馈值
 */
void Motor::setViscous_feedforward(float _data)
{
    Viscous_feedforward = _data;
}

/**
 * @brief 设置转动惯量补偿前馈值
 */
void Motor::setInertia_feedforward(float _data)
{
    Inertia_feedforward = _data;
}

void Motor::update()
{
    specific_motor->update((*this));
}
/**
 * @brief 获得角度
 *
 * @note 必须在电机ready 后才能调用，不然会进入死循环
 */
float Motor::getAngle()
{
    if (getStatus() != MotorStatus::ready)
        for (;;)
            ;
    return (current_angle - calibration); // 如果标定了，那返回的减掉了标定
}

float Motor::getSpeed()
{
    if (getStatus() != MotorStatus::ready)
        for (;;)
            ;
    return current_speed;
}

float Motor::getTorque()
{
    if (getStatus() != MotorStatus::ready)
        for (;;)
            ;
    return current_torque;
}

/**
 * @brief 获取电机当前状态
 *
 * @note
 */
const MotorStatus &Motor::getStatus() const
{
    return specific_motor->status;
}

/**
 * @brief 获取电机当前模式
 *
 * @note
 */
const MotorMode &Motor::getMode() const
{
    return this->mode;
}

/**
 * @brief 设置电机状态
 *
 * @note
 */
void Motor::setMode(MotorMode mode)
{
    if (getStatus() != MotorStatus::ready)
        return;
    switch (mode)
    {
    case MotorMode::unset:
        return;
        break;
    case MotorMode::torque:
        target_torque = 0.0f;
        break;
    // 从其他模式更改为angle时，保持目标角度与当前角度一致
    case MotorMode::angle:
        target_angle = getAngle();
        a_pid.clean_intergral();
        v_pid.clean_intergral(); // 重置a_pid、v_pid
        break;
    case MotorMode::disabled:
        break;
    default:
        for (;;)
            ;
        break;
    }
    this->mode = mode;
}

void Motor::forceSetMode(MotorMode mode)
{
    switch (mode)
    {
    case MotorMode::unset:
        return;
        break;
    case MotorMode::torque:
        target_torque = 0.0f;
        break;
    case MotorMode::angle:
        target_angle = (current_angle - calibration);
        a_pid.clean_intergral();
        v_pid.clean_intergral();
        break;
    case MotorMode::disabled:
        break;
    default:
        for (;;)
            ;
        break;
    }
    this->mode = mode;
}

/**
 * @brief 设置目标值 力矩/角度
 *
 * @note
 */
void Motor::setTarget(float target_value)
{
    if (getStatus() != MotorStatus::ready)
        return;
    switch (mode)
    {
    case MotorMode::unset:
        return;
        break;
    case MotorMode::torque:
        target_torque = target_value;
        break;
    case MotorMode::angle:
        target_angle = target_value;
        break;
    case MotorMode::disabled:
        return;
        break;
    default:
        for (;;)
            ;
        break;
    }
}

/**
 * @brief 电机真正执行目标，进行PID计算
 *
 * @note
 */
void Motor::execute()
{
    // 非ready时仍下发0力矩，避免断联/重连阶段保留旧目标导致风险
    if (getStatus() != MotorStatus::ready)
    {
        specific_motor->setTorque(0.0f);
        return;
    }
    // static LPF_SecondOrder_Classdef lpf_speed(50, 1000); // 尝试使用滤波对速度进行
    // float temp_speed = 0;
    float True_Out;

    switch (mode)
    {
    case MotorMode::unset:
        specific_motor->setTorque(0.0f);
        break;
    case MotorMode::torque:
        specific_motor->setTorque(target_torque);
        break;
    case MotorMode::angle:

        // 滤波
        // temp_speed = lpf_speed.f(getSpeed());

        a_pid.Target = target_angle;
        a_pid.Current = getAngle();

        // 使用电机速度作为角度环微分项
        a_pid.Adjust_importDiff(getSpeed());
        v_pid.Target = a_pid.Out;

        v_pid.Current = getSpeed();
        v_pid.Adjust();

        True_Out = v_pid.Out + Gravity_feedforward + BackEMF_feedforward + Coulomb_feedforward + Viscous_feedforward + Inertia_feedforward;                         // 速度环输出再加上前馈
        True_Out = std_lib::constrain(True_Out, -v_pid.Out_Max, v_pid.Out_Max); // 总输出再加个限制

        specific_motor->setTorque(True_Out);
        break;
    case MotorMode::disabled:
        specific_motor->setTorque(0.0f);
        break;
    default:
        for (;;)
            ;
        break;
    }
}

void Motor::calibrate()
{
    if (getStatus() != MotorStatus::ready)
        return;
    calibration = current_angle;
    target_angle = getAngle();
}
