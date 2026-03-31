#include "JointActuator.h"

/**
 * @brief 更新关节数据
 *
 * @param joint 关节
 * @param joint_index 关节索引
 */
void JointActuator::updateJointData(Joint &joint, uint8_t joint_index)
{
    if (mode == JointMode::angle)
        joint.data.target_angle = specific_actuator->getTarget(joint_index) * joint.mapping.k + joint.mapping.b;
    else
        joint.data.target_angle = 0.0f;
    if (mode == JointMode::torque)
        joint.data.target_torque = specific_actuator->getTarget(joint_index) / joint.mapping.k;
    else
        joint.data.target_torque = 0.0f;
    joint.data.current_torque = specific_actuator->getTorque(joint_index) / joint.mapping.k;
    joint.data.current_angle = specific_actuator->getAngle(joint_index) * joint.mapping.k + joint.mapping.b;
    joint.data.current_speed = specific_actuator->getSpeed(joint_index) * joint.mapping.k;
}

/**
 * @brief 设置模式
 *
 * @param mode 模式
 */
void JointActuator::setMode(JointMode mode)
{
    this->mode = mode;
    switch (this->mode)
    {
    case JointMode::unset:
        return;
        break;
    case JointMode::angle:
        specific_actuator->setMode(ActuatorMode::angle);
        break;
    case JointMode::torque:
        specific_actuator->setMode(ActuatorMode::torque);
        break;
    case JointMode::disabled:
        specific_actuator->setMode(ActuatorMode::disabled);
        break;
    default:
        for (;;)
            ;
        break;
    }
}
/**
 * @brief 设置关节目标值
 *
 * @param joint_index 关节索引
 * @param target_value 目标值
 * @param set 是否执行
 */
void JointActuator::setJointTarget(uint8_t joint_index, float target_value, bool set = true)
{
    specific_actuator->setTarget(joint_index, target_value);
    if (set)
    {
        specific_actuator->set();
    }
}

/**
 * @brief 是否触发标定
 *
 * @return 是否触发标定
 */
bool JointActuator::isCalibrationTriggered()
{
    return specific_actuator->isCalibrationTriggered();
}

/**
 * @brief 标定
 */
void JointActuator::calibrate()
{
    specific_actuator->calibrate();
}
/**
 * @brief 是否标定
 *
 * @return 是否标定
 */
bool JointActuator::isCalibrated()
{
    return specific_actuator->calibrated;
}
