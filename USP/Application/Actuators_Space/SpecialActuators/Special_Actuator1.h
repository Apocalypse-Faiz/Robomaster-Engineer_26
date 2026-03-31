#ifndef __SPECIAL_ACTUATOR1_H__
#define __SPECIAL_ACTUATOR1_H__

#include "JointActuator.h"
#include "Motor.h"

/**
 * @brief 特殊执行器1
 * @note 该执行器是只有一个电机类型的执行器，是最普遍的一种驱动器
 */
class Special_Actuator1 : public JointActuator_
{
private:
    Motor *motor = nullptr;
    float actuator_target[1];

public:
    explicit Special_Actuator1(Motor *motor)
    {
        joint_number = 1;
        calibrated = false;
        mode = ActuatorMode::unset;
        if (motor)
        {
            this->motor = motor;
        }
    }
    void setMode(ActuatorMode mode) override; // 设置模式
    void setTarget(uint8_t joint_index, float target_value) override; // 设置目标值
    void set() override; // 执行
    float getTarget(uint8_t joint_index) override; // 获取目标值
    float getTorque(uint8_t joint_index) override; // 获取力矩
    float getAngle(uint8_t joint_index) override; // 获取角度
    float getSpeed(uint8_t joint_index) override; // 获取速度
    void calibrate() override; // 标定
    bool isCalibrationTriggered() override; // 是否触发标定
    ~Special_Actuator1()
    {
    }
};

#endif
