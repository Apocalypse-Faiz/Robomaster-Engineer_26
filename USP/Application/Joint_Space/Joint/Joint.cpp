#include "Joint.h"
#include "srml_std_lib.h"

void Joint::setJointParam(float min_angle, float max_angle, float min_torque, float max_torque)
{
    parameters.min_angle = min_angle;
    parameters.max_angle = max_angle;
    parameters.min_torque = min_torque;
    parameters.max_torque = max_torque;
}
void Joint::setJointMapping(float k, float b)
{
    mapping.k = k;
    mapping.b = b;
}
void Joint::setMode(JointMode mode)
{
    joint_->setMode(mode);
}
const JointMode &Joint::getMode() const
{
    return joint_->mode;
}
bool Joint::isCalibrated()
{
    return joint_->isCalibrated();
}
void Joint::updateJointData()
{
    joint_->updateJointData(*this, joint_index);
}
bool Joint::isCalibrationTriggered()
{
    return joint_->isCalibrationTriggered();
}
void Joint::calibrate()
{
    joint_->calibrate();
}
float Joint::getTargetAngle()
{
    return data.target_angle;
}
float Joint::getTargetTorque()
{
    return data.target_torque;
}
float Joint::getCurrentAngle()
{
    return data.current_angle;
}
float Joint::getCurrentSpeed()
{
    return data.current_speed;
}
float Joint::getCurrentTorque()
{
    return data.current_torque;
}

void Joint::setTorque(float target_torque)
{
    if (getMode() != JointMode::torque)
        return;
    updateJointData();
    joint_->setJointTarget(joint_index, target_torque * mapping.k, true);
}

bool Joint::attach(TrajectoryPointArray *_trajectory_point_array)
{
    this->trajectory_point_array = _trajectory_point_array;
    return trajectory_planning->attach(_trajectory_point_array); // 这里传入的应该不是参数，而是类内元素吧
}

void Joint::stopTrajectoryPlanner()
{
    if (getMode() != JointMode::angle)
        return;
    updateJointData(); // 实时更新数据
    trajectory_planning->reset(getTargetAngle());
}

bool Joint::startTrajectoryPlanner(uint32_t tick)
{
    if (getMode() != JointMode::angle || !isCalibrated())
        return false;
    updateJointData();
    return trajectory_planning->compute(tick, getTargetAngle());
}

/**
 * @brief 轨迹规划的真正执行
 *
 */
void Joint::takeOver(uint32_t t)
{
    updateJointData();
    if (getMode() == JointMode::disabled)
    {
        stopTrajectoryPlanner();
    }
    else if (getMode() == JointMode::angle)
    {
        if (!trajectory_planning->get(t + 1, &joint_output_angle)) // 确保一直在增加
            joint_output_angle = getTargetAngle();                 // 如果不成功，就为原来的值

        joint_output_angle = std_lib::constrain(joint_output_angle, parameters.min_angle, parameters.max_angle); // 关节空间下的角度限位，待测试

        joint_->setJointTarget(joint_index, (joint_output_angle - mapping.b) / mapping.k, true);
    }
    else if (getMode() == JointMode::torque)
    {
    }
}

/**
 * @brief 轨迹规划的验证显示
 * @note 在轨迹规划执行前，先验证一下是否正确
 */
float Joint::Trajectory_display(uint32_t t)
{
    updateJointData();
    if (getMode() == JointMode::disabled)
    {
        stopTrajectoryPlanner();
    }
    else if (getMode() == JointMode::angle)
    {
        if (!trajectory_planning->get(t + 1, &joint_output_angle)) // 确保一直在增加
            joint_output_angle = getTargetAngle();                 // 如果不成功，就为原来的值

        joint_output_angle = std_lib::constrain(joint_output_angle, parameters.min_angle, parameters.max_angle); // 关节空间下的角度限位，待测试
        return joint_output_angle;
    }

    return -999.0f;
}

bool Joint::isTrajectoryPlannerFree()
{

    return trajectory_planning->free();
}

void Joint::ajustJointTarget(float da)
{
    if (getMode() != JointMode::angle)
        return;
    updateJointData();
    float temp_output = getTargetAngle();                                                      // 只能说允许超一点da，也就只能把关节限位角度再往上保守一些
    temp_output = std_lib::constrain(temp_output, parameters.min_angle, parameters.max_angle); // 关节空间下的角度限位

    joint_->setJointTarget(joint_index, (temp_output - mapping.b) / mapping.k + da, true);
}

void Joint::setTargetAngle(float target_angle)
{
    if (getMode() != JointMode::angle)
        return;
    updateJointData();
    float temp_output = std_lib::constrain(target_angle, parameters.min_angle, parameters.max_angle); // 关节空间下的角度限位

    joint_->setJointTarget(joint_index, (temp_output - mapping.b) / mapping.k, true);
}
