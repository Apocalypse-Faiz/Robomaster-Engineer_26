#ifndef __JOINT_ACTUATOR_H__
#define __JOINT_ACTUATOR_H__

#include "Joint.h"

/**
 * @brief 执行器模式
 */
enum class ActuatorMode
{
    unset,   // 未设置
    torque,  // 力矩
    angle,   // 角度
    disabled // 禁用
};
class JointActuator;

/**
 * @brief 关节执行器接口
 */
struct JointActuator_
{
    friend JointActuator;

public:
    virtual void setMode(ActuatorMode) = 0;                              // 设置模式
    virtual void setTarget(uint8_t joint_index, float target_value) = 0; // 设置目标值
    virtual void set() = 0;                                              // 执行
    virtual float getTarget(uint8_t joint_index) = 0;                    // 获取目标值
    virtual float getTorque(uint8_t joint_index) = 0;                    // 获取力矩
    virtual float getAngle(uint8_t joint_index) = 0;                     // 获取角度
    virtual float getSpeed(uint8_t joint_index) = 0;                     // 获取速度
    virtual void calibrate() = 0;                                        // 标定
    virtual bool isCalibrationTriggered() = 0;                           // 是否触发标定
    virtual ~JointActuator_() {}

protected:
    uint8_t joint_number; // 关节数量
    ActuatorMode mode;    // 模式
    bool calibrated;      // 是否标定
};

/**
 * @brief 关节执行器
 */
class JointActuator : public Joint_
{
private:
    JointActuator_ *specific_actuator = nullptr; // 具体执行器

public:
    explicit JointActuator(JointActuator_ *specific_actuator)
    {
        if (specific_actuator)
        {
            this->specific_actuator = specific_actuator;
            this->mode = JointMode::unset;
        }
    }
    ~JointActuator() override
    {
    }
    void updateJointData(Joint &joint, uint8_t joint_index) override;                // 更新关节数据
    void setMode(JointMode mode) override;                                           // 设置模式
    void setJointTarget(uint8_t joint_index, float target_value, bool set) override; // 设置关节目标值
    bool isCalibrationTriggered() override;                                          // 是否触发标定
    void calibrate() override;                                                       // 标定
    bool isCalibrated() override;                                                    // 是否标定
};

#endif
