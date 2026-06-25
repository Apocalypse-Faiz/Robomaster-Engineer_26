#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "SRML.h"

class Motor;
enum class MotorStatus
{
    ready,
    error
};
enum class MotorMode
{
    unset, // 未设置
    torque,
    angle,
    disabled // 失能
};
struct MotorPidParameters
{
    float v_Kp;
    float v_Min;
    float v_Max;
    float a_Kp;
    float a_Ki;
    float a_Kd;
    float a_Min;
    float a_Max;
    float a_IntSep;
    float a_IntMax;
};
struct Motor_
{
    friend Motor; // 友元类  Motor 类可以直接访问 Motor_ 类中的私有成员，比如 status 变量

public:
    virtual void update(Motor &motor) = 0;    // 供上层调用，更新Motor的各种状态
    virtual void setTorque(float torque) = 0; // 设置目标力矩
    virtual ~Motor_() {}

protected:
    MotorStatus status; // 电机状态
};
class Motor
{
private:
    Motor_ *specific_motor = nullptr;
    MotorMode mode;
    float calibration; // 标定的角度

public:
    float Gravity_feedforward = 0; // 重力前馈
    float BackEMF_feedforward = 0; // 反电动势前馈
    float Coulomb_feedforward = 0; // 库仑摩擦前馈
    float Viscous_feedforward = 0; // 黏性摩擦前馈
    float Inertia_feedforward = 0; // 转动惯量前馈
    float Speed_feedforward = 0;        // 速度前馈: 注入速度环 Target (单位同 getSpeed)
    float Acceleration_feedforward = 0; // 加速度前馈: 力矩级, 注入力矩输出 (调用方已乘等效惯量/增益)

    float target_angle; // 标定后的目标角度 ~ (current_angle - calibration)
    float target_torque;
    float current_angle; // 电机实际的角度，是未经标定
    float current_speed;
    float current_torque;
    myPID a_pid; // 角度PID
    myPID v_pid; // 速度PID
    explicit Motor(Motor_ *specific_motor)
    {
        if (specific_motor)
        {
            this->specific_motor = specific_motor;
            mode = MotorMode::unset;
            calibration = 0.0f;
            target_angle = 0.0f;
            target_torque = 0.0f;
            current_angle = 0.0f;
            current_speed = 0.0f;
            current_torque = 0.0f;
        }
    }

    void setPidParameters(MotorPidParameters param); // 设置pid参数

    void setGravity_feedforward(float _data);        // 设置重力前馈

    void setBackEMF_feedforward(float _data);       // 设置电流前馈

    void setCoulomb_feedforward(float _data);       // 设置库仑摩擦前馈

    void setViscous_feedforward(float _data);       // 设置黏性摩擦前馈

    void setInertia_feedforward(float _data);       // 设置转动惯量前馈

    void setSpeed_feedforward(float _data);        // 设置速度前馈 (注入速度环 Target)

    void setAcceleration_feedforward(float _data); // 设置加速度前馈 (力矩级)

    void update(); // 更新电机状态

    float getAngle(); // 获取角度

    float getSpeed(); // 获取速度

    float getTorque(); // 获取力矩

    const MotorStatus &getStatus() const; // 获取电机状态

    const MotorMode &getMode() const; // 获取电机模式

    void setMode(MotorMode mode); // 设置电机模式

    void forceSetMode(MotorMode mode); // 强制设置模式（用于掉线保护）

    void setTarget(float target_value); // 设置目标值

    void execute(); // 执行PID计算

    void calibrate(); // 标定

    ~Motor()
    {
    }
};

#endif
