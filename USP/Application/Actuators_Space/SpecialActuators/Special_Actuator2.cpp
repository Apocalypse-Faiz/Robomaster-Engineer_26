#include "Special_Actuator2.h"

/**
 * @brief 将执行器角度转换为电机角度
 *
 * @param actuator_angle 执行器角度
 * @param motor_angle 电机角度
 */
void Special_Actuator2::resolve_angle_actuator2motor(float actuator_angle[2], float motor_angle[2])
{
    motor_angle[0] = actuator_angle[1] * (1 / Differential_reduction_ratio) + actuator_angle[0]; //*2为齿轮减速比带来的
    motor_angle[1] = actuator_angle[1] * (1 / Differential_reduction_ratio) - actuator_angle[0];
}

/**
 * @brief 将电机角度转换为执行器角度
 *
 * @param motor_angle 电机角度
 * @param actuator_angle 执行器角度
 */
void Special_Actuator2::resolve_angle_motor2actuator(float motor_angle[2], float actuator_angle[2])
{
    actuator_angle[0] = 0.5f * (motor_angle[0] + (-motor_angle[1]));                                  //  两个电机的方向是相反的，那么我们让左5电机方向为正
    actuator_angle[1] = 0.5f * (Differential_reduction_ratio) * (motor_angle[0] - (-motor_angle[1])); // 前面的0.5为齿轮减速比带来的，只有J6才有齿轮减速比的效果
    // actuator_angle[0] = 0.5 * (motor_angle[0] + (-motor_angle[1]));
    // actuator_angle[1] = 1.5 * (motor_angle[0] - (-motor_angle[1]));
}

/**
 * @brief 将执行器力矩转换为电机力矩
 *
 * @param actuator_torque 执行器力矩
 * @param motor_torque 电机力矩
 */
void Special_Actuator2::resolve_torque_actuator2motor(float actuator_torque[2], float motor_torque[2])
{
   // J5 (Pitch) 轴力矩由两电机反向出力提供，且经过 0.5 减速比（力矩放大 2 倍）
    // 所以电机端只需输出 0.5 * 0.5 = 0.25 倍的执行器力矩
    motor_torque[0] = 0.5f * Differential_reduction_ratio * actuator_torque[0] + 0.5f * actuator_torque[1];
    motor_torque[1] = -0.5f * Differential_reduction_ratio * actuator_torque[0] + 0.5f * actuator_torque[1];
}

/**
 * @brief 将电机力矩转换为执行器力矩
 *
 * @param motor_torque 电机力矩
 * @param actuator_torque 执行器力矩
 */
void Special_Actuator2::resolve_torque_motor2actuator(float motor_torque[2], float actuator_torque[2])
{
    // J5 (Pitch): 两电机力矩之差，由于大齿轮 2:1 减速，末端力矩被放大
    // 物理表现：(m0 - m1) * (1 / 0.5)
    actuator_torque[0] = (motor_torque[0] - motor_torque[1]) * (1.0f / Differential_reduction_ratio);

    // J6 (Roll): 两电机力矩之和，无额外齿轮减速
    actuator_torque[1] = (motor_torque[0] + motor_torque[1]);
}

/**
 * @brief 将电机速度转换为执行器速度
 *
 * @param motor_speed 电机速度
 * @param actuator_speed 执行器速度
 */
void Special_Actuator2::resolve_speed_motor2actuator(float motor_speed[2], float actuator_speed[2])
{
    actuator_speed[0] = 0.5f * (motor_speed[0] + motor_speed[1]);
    actuator_speed[1] = 0.5f * (motor_speed[0] - motor_speed[1]);
}

/**
 * @brief 设置模式
 *
 * @param mode 模式
 */
void Special_Actuator2::setMode(ActuatorMode mode)
{
    float motor_target_angle[2];
    this->mode = mode;
    switch (this->mode)
    {
    case ActuatorMode::unset:
        abort();
        break;
    case ActuatorMode::torque:
        motor[0]->setMode(MotorMode::torque);
        motor[1]->setMode(MotorMode::torque);
        actuator_target[0] = 0.0f;
        actuator_target[1] = 0.0f;
        break;
    case ActuatorMode::angle:
        motor[0]->setMode(MotorMode::angle);
        motor[1]->setMode(MotorMode::angle);
        motor_target_angle[0] = motor[0]->target_angle;
        motor_target_angle[1] = motor[1]->target_angle;
        resolve_angle_motor2actuator(motor_target_angle, actuator_target);
        break;
    case ActuatorMode::disabled:
        motor[0]->setMode(MotorMode::disabled);
        motor[1]->setMode(MotorMode::disabled);
        break;
    default:
        abort();
        break;
    }
}
/**
 * @brief 设置目标值
 *
 * @param joint_index 关节索引
 * @param target_value 目标值
 */
void Special_Actuator2::setTarget(uint8_t joint_index, float target_value)
{
    actuator_target[joint_index] = target_value;
}
/**
 * @brief 执行
 */
void Special_Actuator2::set()
{
    float motor_target[2];
    switch (mode)
    {
    case ActuatorMode::angle:
        resolve_angle_actuator2motor(actuator_target, motor_target); // 将执行器目标角度转换为电机角度再进行下发目标
        motor[0]->setTarget(motor_target[0]);
        motor[1]->setTarget(motor_target[1]);
        break;
    case ActuatorMode::torque:
        resolve_torque_actuator2motor(actuator_target, motor_target); // 将执行器目标力矩转换为电机力矩
        motor[0]->setTarget(motor_target[0]);
        motor[1]->setTarget(motor_target[1]);
        break;
    default:
        break;
    }
}
/**
 * @brief 获取目标值
 *
 * @param joint_index 关节索引
 * @return 目标值
 */
float Special_Actuator2::getTarget(uint8_t joint_index)
{
    float motor_target_angle[2];
    float motor_target_torque[2];
    switch (mode)
    {
    case ActuatorMode::angle:
        motor_target_angle[0] = motor[0]->target_angle;
        motor_target_angle[1] = motor[1]->target_angle;
        resolve_angle_motor2actuator(motor_target_angle, actuator_target);
        return actuator_target[joint_index]; // 返回的是驱动器层的目标值
        break;
    case ActuatorMode::torque:
        motor_target_torque[0] = motor[0]->target_torque;
        motor_target_torque[1] = motor[1]->target_torque;
        resolve_torque_motor2actuator(motor_target_torque, actuator_target);
        return actuator_target[joint_index];
        break;
    default:
        return 0; // risky
        break;
    }
}
float Special_Actuator2::getTorque(uint8_t joint_index)
{
    float motor_torque[2] = {motor[0]->getTorque(), motor[1]->getTorque()};
    float actuator_torque[2];
    resolve_torque_motor2actuator(motor_torque, actuator_torque);
    return actuator_torque[joint_index];
}
float Special_Actuator2::getAngle(uint8_t joint_index)
{
    float motor_angle[2] = {motor[0]->getAngle(), motor[1]->getAngle()};
    float actuator_angle[2];
    resolve_angle_motor2actuator(motor_angle, actuator_angle);
    return actuator_angle[joint_index];
}
float Special_Actuator2::getSpeed(uint8_t joint_index)
{
    float motor_speed[2] = {motor[0]->getSpeed(), motor[1]->getSpeed()};
    float actuator_speed[2];
    resolve_speed_motor2actuator(motor_speed, actuator_speed);
    return actuator_speed[joint_index];
}
void Special_Actuator2::calibrate()
{
    motor[0]->calibrate();
    motor[1]->calibrate();
    calibrated = true;
}
bool Special_Actuator2::isCalibrationTriggered()
{
    if (getTorque(0) >= 800.0f)
        return true;
    else
        return false;
}
