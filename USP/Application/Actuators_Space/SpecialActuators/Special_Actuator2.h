/*
<reference>
Actuator1::
    void setMode(ActuatorMode mode) override; 设置驱动器控制模式（力控、位控、失能）
    void setTarget(uint8_t joint_index, float target_value) override; 设置驱动器控制目标值
    void set() override; 将目标值更新到电机（解算）
    float getTarget() override; 获取目标值（是力矩还是角度，视控制模式而定）
    float getTorque(uint8_t joint_index) override; 获取驱动器力矩
    float getAngle(uint8_t joint_index) override; 获取驱动器角度
    float getSpeed(uint8_t joint_index) override; 获取驱动器速度
    void calibrate() override; 标定驱动器
    bool isCalibrationTriggered() override; 标定条件
    uint8_t joint_number; 驱动器包含的电机数量
    ActuatorMode mode; 驱动器模式
    bool calibrated; 驱动器标定状态
*/

#ifndef __ACTUATOR1_H__
#define __ACTUATOR1_H__

#include "Motor.h"
#include "JointActuator.h"

/**
 * @brief 特殊执行器2
 * @note 该执行器是由三个齿轮构成的差速器，旁边齿轮到中间齿轮的减速比为2 ：1，电机驱动的是两旁的齿轮，所以下面的解算直接用到了该减速比
 */
class Special_Actuator2 : public JointActuator_
{
private:
    Motor *motor[2] = {nullptr, nullptr};
    float actuator_target[2];
    float Differential_reduction_ratio = 0.5;
    void resolve_angle_actuator2motor(float actuator_angle[2], float motor_angle[2]);    // 从执行器角度转换为电机角度
    void resolve_angle_motor2actuator(float motor_angle[2], float actuator_angle[2]);    // 从电机角度转换为执行器角度
    void resolve_torque_actuator2motor(float actuator_torque[2], float motor_torque[2]); // 从执行器力矩转换为电机力矩
    void resolve_torque_motor2actuator(float motor_torque[2], float actuator_torque[2]); // 从电机力矩转换为执行器力矩
    void resolve_speed_motor2actuator(float motor_speed[2], float actuator_speed[2]);    // 从电机速度转换为执行器速度

public:
    explicit Special_Actuator2(Motor *motor1, Motor *motor2)
    {
        joint_number = 2;
        calibrated = false;
        mode = ActuatorMode::unset;

        if (motor1 && motor2)
        {
            this->motor[0] = motor1;
            this->motor[1] = motor2;
        }
        else
            abort(); // 报错
    }
    void setMode(ActuatorMode mode) override;                         // 设置模式
    void setTarget(uint8_t joint_index, float target_value) override; // 设置目标值
    void set() override;                                              // 执行
    float getTarget(uint8_t joint_index) override;                    // 获取目标值
    float getTorque(uint8_t joint_index) override;                    // 获取力矩
    float getAngle(uint8_t joint_index) override;                     // 获取角度
    float getSpeed(uint8_t joint_index) override;                     // 获取速度
    void calibrate() override;                                        // 标定
    bool isCalibrationTriggered() override;                           // 是否触发标定
    ~Special_Actuator2() override
    {
    }
};

#endif
