#include "Special_Actuator3.h"

void Special_Actuator3::setMode(ActuatorMode mode)
{
    switch (mode)
    {
    case ActuatorMode::unset:
        return;
        break;
    case ActuatorMode::torque:
        motor->setMode(MotorMode::torque);
        actuator_target[0] = 0.0f;
        break;
    case ActuatorMode::angle:
        motor->setMode(MotorMode::angle);
        actuator_target[0] = motor->target_angle;
        break;
    case ActuatorMode::disabled:
        motor->setMode(MotorMode::disabled);
        break;
    default:
        for (;;)
            ;
        break;
    }
    this->mode = mode;
}
void Special_Actuator3::setTarget(uint8_t joint_index, float target_value)
{
    actuator_target[joint_index] = target_value;
}
void Special_Actuator3::set()
{
    switch (mode)
    {
    case ActuatorMode::angle:
        motor->setTarget(actuator_target[0]);
        break;
    case ActuatorMode::torque:
        motor->setTarget(actuator_target[0]);
        break;
    default:
        break;
    }
}
float Special_Actuator3::getTarget(uint8_t joint_index)
{
    switch (mode)
    {
    case ActuatorMode::angle:
        return motor->target_angle;
        break;
    case ActuatorMode::torque:
        return motor->target_torque;
        break;
    default:
        return 0; // a bit risky
        break;
    }
}
float Special_Actuator3::getTorque(uint8_t joint_index)
{
    return motor->getTorque();
}
float Special_Actuator3::getAngle(uint8_t joint_index)
{
    return motor->getAngle();
}
float Special_Actuator3::getSpeed(uint8_t joint_index)
{
    return motor->getSpeed();
}
void Special_Actuator3::calibrate()
{
    // 该菜品不需要calibrate，直接判定
    calibrated = true;
}
bool Special_Actuator3::isCalibrationTriggered()
{
    // if (getTorque(0) >= 800.0f)
    //     return true;
    // else
    //     return false;

    return calibrated;
}
