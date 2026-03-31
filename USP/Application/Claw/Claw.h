#ifndef __GRIPPER_H__
#define __GRIPPER_H__

#include "Motor.h"
#include <cmath>

enum class GripperState 
{
    IDLE,           // 空闲状态
    OPENING,        // 正在打开
    CLOSING,        // 正在关闭
    GRIPPED,        // 已夹住物体
    RESETTING,      // 力矩复位中
    ERROR           // 错误状态
};

class Gripper 
{
private:
    Motor* motor;                           // 电机对象
    GripperState state;                     // 夹爪状态
    
    // 配置参数
    float reduction_ratio;                  // 减速比
    float bias;                             // 夹爪偏置
    float min_position;                     // 最小机械限位
    float max_position;                     // 最大机械限位
    float grip_current_threshold;           // 夹持电流阈值
    float reset_torque;                     // 复位力矩阈值
    float max_grip_torque;                  // 最大夹持力矩
    
    // 状态变量
    float target_torque;                    // 目标力矩
    float current_position;                 // 当前位置
    bool object_detected;                   // 物体检测标志
    bool reseted;
    uint32_t grip_confirmation_count;       // 夹持确认计数器
    
    // 控制变量
    float calculated_torque;                // 计算出的力矩值
    bool new_torque_available;              // 新力矩值可用标志
    float reset_start_angle_offset;         // 力矩复位开始时的电机角度 
    
    // 内部方法
    bool checkMechanicalLimit();
    bool checkObjectGripped();
    void updateCurrentPosition();
    float calculateControlTorque();
    float calculateControlTorqueUncalibrated();  // 未校准状态下的力矩计算

public:
    /**
     * @brief 夹爪构造函数
     * @param motor 电机对象
     * @param reduction_ratio 减速比
     * @param min_limit 最小机械限位
     * @param max_limit 最大机械限位
     */
    Gripper(Motor* motor): motor(motor)
    {
        
        state = GripperState::IDLE;
        target_torque = 0.0f;
        reset_torque = 0.0f;
        reduction_ratio = 0.0f;
        min_position = 0.0f;
        max_position = 0.0f;
        calculated_torque = 0.0f;
        current_position = 0.0f;
        object_detected = false;
        grip_confirmation_count = 0;
        new_torque_available = false;
        
        // 设置电机为力矩模式
        if(motor) 
        {
            motor->setMode(MotorMode::torque);
        }
    }
    
    /**
     * @brief 初始化夹爪
     * @param grip_current_threshold 夹持电流阈值
     * @param reset_torque 复位力矩
     * @param max_grip_torque 最大夹持力矩
     */
    void init(float grip_current_threshold, float max_grip_torque, float reset_torque, float reduction_ratio, float bias, float max_position, float mini_position);
    
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
     * @param torque 打开力矩
     */
    void open(float torque = 0.5f);
    
    /**
     * @brief 关闭夹爪（力控）
     * @param torque 夹持力矩
     */
    void close(float torque = 1.0f);
    
    /**
     * @brief 力矩复位（回到初始位置）
     */
    void torqueReset();
    
    /**
     * @brief 紧急停止
     */
    void emergencyStop();
    
    /**
     * @brief 设置夹持电流阈值
     * @param threshold 新的阈值
     */
    void setGripCurrentThreshold(float threshold);
    
    /**
     * @brief 获取夹爪状态
     * @return 当前夹爪状态
     */
    GripperState getState() const;
    
    /**
     * @brief 是否检测到物体
     * @return true: 已夹住物体, false: 未夹住
     */
    bool isObjectGripped() const;
    
    /**
     * @brief 获取当前位置
     * @return 当前位置 (rad)
     */
    float getCurrentPosition() const;
    
    /**
     * @brief 获取当前电机电流
     * @return 电流值
     */
    float getCurrentCurrent() const;
    
   /**
     * @brief 是否已复位
     * @return true: 已复位, false: 未复位
     */
    bool isReseted() const { return reseted; }
    
    /**
     * @brief 清除错误状态
     */
    void clearError();

    /**
     * @brief 校准夹爪
     */
    void calibrate();

    void setState(GripperState state);

};

#endif