#ifndef __JOINT_H__
#define __JOINT_H__

#include "Trajectory.h"
#include "main.h"

class Joint;

/**
 * @brief 关节模式
 */
enum JointMode
{
    unset,   // 未设置
    torque,  // 力矩模式
    angle,   // 角度模式
    disabled // 禁用
};

/**
 * @brief 关节参数
 */
struct JointParameters
{
    float min_angle;
    float max_angle;
    float min_speed;
    float max_speed;
    float min_torque;
    float max_torque;
};
/**
 * @brief 关节映射
 */
struct JointMapping
{
    float k;
    float b;
};
/**
 * @brief 关节数据
 */
struct JointData
{
    float target_angle;
    float target_torque;
    float current_angle;
    float current_speed;
    float current_torque;
};
/**
 * @brief 关节类的执行器接口，继承给JointActuator
 */
struct Joint_
{
    friend Joint;

public:
    virtual void updateJointData(Joint &joint, uint8_t joint_index) = 0;
    virtual void setMode(JointMode) = 0;
    virtual void setJointTarget(uint8_t joint_index, float target_value, bool set) = 0;
    virtual bool isCalibrationTriggered() = 0;
    virtual void calibrate() = 0;
    virtual bool isCalibrated() = 0;
    virtual ~Joint_() {}

protected:
    JointMode mode;
};
/**
 * @brief 关节
 */
class Joint
{
private:
    uint8_t joint_index;
    Joint_ *joint_;
    TrajectoryPlanning *trajectory_planning;
    TrajectoryPointArray *trajectory_point_array;
    float joint_output_angle; // 轨迹规划输出角度

public:
    JointParameters parameters; // 关节空间下的限位
    JointMapping mapping;       // 映射
    JointData data;

    explicit Joint(Joint_ *joint_, TrajectoryPlanning *trajectory_planning, uint8_t joint_index)
    {
        this->joint_ = joint_;
        this->trajectory_planning = trajectory_planning;
        this->joint_index = joint_index;
    }
    void setJointParam(float min_angle, float max_angle, float min_torque, float max_torque); // 设置关节参数
    void setJointMapping(float k, float b);                                                   // 设置关节映射
    void setMode(JointMode mode);                                                             // 设置关节模式
    const JointMode &getMode() const;                                                         // 获取关节模式
    bool isCalibrated();                                                                      // 是否标定
    void updateJointData();                                                                   // 更新关节数据
    bool isCalibrationTriggered();                                                            // 是否触发标定
    void calibrate();                                                                         // 标定
    float getTargetAngle();                                                                   // 获取目标角度
    float getTargetTorque();                                                                  // 获取目标力矩
    float getCurrentAngle();                                                                  // 获取当前角度
    float getCurrentSpeed();                                                                  // 获取当前速度
    float getCurrentTorque();                                                                 // 获取当前力矩

    void setTorque(float target_torque); // 设置力矩

    bool attach(TrajectoryPointArray *_trajectory_point_array); // 附加轨迹点数组

    void stopTrajectoryPlanner(); // 停止轨迹规划

    bool startTrajectoryPlanner(uint32_t tick); // 开始轨迹规划

    void takeOver(uint32_t t); // 轨迹规划接管

    // 获取 t 时刻轨迹的目标速度/加速度 (关节空间, 度/秒 与 度/秒²)
    bool getTrajectoryDerivative(uint32_t t, float *target_speed, float *target_acceleration);
    float getTrajectorySpeed(uint32_t t);        // 目标速度 (度/秒)
    float getTrajectoryAcceleration(uint32_t t); // 目标加速度 (度/秒²)

    float Trajectory_display(uint32_t t); // 轨迹显示

    bool isTrajectoryPlannerFree(); // 判断轨迹规划是否空闲

    void ajustJointTarget(float da); // 调整关节目标，用于简单驱动关节电机

    void setTargetAngle(float target_angle); //设置关节目标角度

    ~Joint()
    {
    }
};

#endif
