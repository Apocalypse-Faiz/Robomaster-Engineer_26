#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include "Servo.h"

enum class GripperState 
{
    IDLE,           // 空闲状态
    OPENING,        // 正在打开
    CLOSING,        // 正在关闭
    ERROR           // 错误状态
};

class Gripper 
{
private:
    Servo_Classdef* servo;                  // 舵机对象
    GripperState state;                     // 夹爪状态

    // 舵机角度参数
    float min_position;                     // 最小机械限位
    float max_position;                     // 最大机械限位
    float angle_step;                       // 每周期步进角度

    // 状态变量
    float current_position;                 // 当前位置
    float target_position;                  // 目标位置
    bool pwm_started;                       // PWM是否已启动

    // 内部方法
    bool checkMechanicalLimit();
    void updateCurrentPosition();
    void moveTowardsTarget();

public:
    /**
     * @brief 夹爪构造函数
     * @param servo 舵机对象
     * @param reduction_ratio 减速比
     * @param min_limit 最小机械限位
     * @param max_limit 最大机械限位
     */
    Gripper(Servo_Classdef* servo): servo(servo)
    {
        state = GripperState::IDLE;
        min_position = 0.0f;
        max_position = 76.0f;
        angle_step = 3.0f;
        current_position = 0.0f;
        target_position = 0.0f;
        pwm_started = false;
    }
    
    /**
     * @brief 更新夹爪传感器数据（高速循环中调用）
     * @note 只读取传感器数据，不发送控制指令
     */
    void update();
    
    /**
     * @brief 更新夹爪控制逻辑（中速循环中调用）
     * @note 根据传感器数据计算控制量，但不发送给电机
     */
    void updateControl();
    
    /**
     * @brief 执行电机输出（定时循环中调用）
     * @note 将计算好的控制量发送给电机
     */
    void executeOutput();
    
    /**
     * @brief 打开夹爪
     */
    void open();
    
    /**
     * @brief 关闭夹爪（位控）
     */
    void close();
    
    /**
     * @brief 紧急停止
     */
    void emergencyStop();
    
    /**
     * @brief 获取夹爪状态
     * @return 当前夹爪状态
     */
    GripperState getState() const;
    
    /**
     * @brief 获取当前位置
     * @return 当前位置 (rad)
     */
    float getCurrentPosition() const;
    
    /**
     * @brief 清除错误状态
     */
    void clearError();

    void setState(GripperState state);

};

#endif