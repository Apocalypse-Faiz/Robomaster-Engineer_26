#include "VisualControl.h"

/* 全局视觉控制器实例 */
VisionController vision_controller;

/**
 * @brief 检查目标是否变化
 */
bool VisionController::checkTargetsChanged(VisionData& data) 
{
    bool changed = false;
    const float tolerance = 0.05f; // 容忍度0.1度
    
    for (int i = 0; i < 7; i++) 
    {
        if (fabs(data.joint_angle[i] - previous_angles[i]) > tolerance) 
        {
            changed = true;
            break;
        }
    }
    
    if (changed)
    {
        memcpy(previous_angles, data.joint_angle, sizeof(previous_angles));
    }

    return changed;
}

/**
 * @brief 启动缓启动过程
 */
void VisionController::startSoftStart(Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count)
{
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < joint_count; i++) 
    {
        if (joints[i].isCalibrated()) 
        {
            // 记录起始角度
            // start_angles[i] = joints[i].getCurrentAngle();
            
            joints[i].stopTrajectoryPlanner();
            trajectory_arrays[i].clear();
            
            // 缓启动：500ms平滑移动到视觉起始位置
            // trajectory_arrays[i].add(600, data.joint_angle[i]);
            trajectory_arrays[i].add(500, start_angles[i]);
            
            joints[i].attach(&trajectory_arrays[i]);
            joints[i].startTrajectoryPlanner(current_time);
        }
    }

    // 起始动作独立于视觉数据状态，是否完成由 update() 单独检查。
    soft_start_active = true;
}

/**
 * @brief 检查目标是否都为 0
 */
bool VisionController::checkTargetsAllZero(VisionData& data)
{
    bool is_all_zero = false;
    uint8_t zero_count = 0;

    for (int i = 0; i < 7; i++)
    {
        if (data.joint_angle[i] == 0)
        zero_count++;
    }

    if (zero_count == 7)
    {
        is_all_zero = true;
    }

    return is_all_zero;
}

/**
 * @brief 更新视觉控制
 */
void VisionController::update(VisionData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count) 
{
    uint32_t current_time = HAL_GetTick();

    // F 键动作未完成时，不允许视觉目标覆盖 start_angles 轨迹。
    if (soft_start_active)
    {
        for (uint8_t i = 0; i < joint_count; i++)
        {
            if (joints[i].isCalibrated() && !joints[i].isTrajectoryPlannerFree())
            {
                return;
            }
        }

        soft_start_active = false;
    }
    
    switch (state) 
    {
        case VISION_DISABLED:
            if (data.enable) 
            {
                if (!first)
                {
                    // 记录当前位置作为起始点
                    for (int i = 0; i < 7; i++)
                    {
                        if (joints[i].isCalibrated())
                        {
                            previous_angles[i] = joints[i].getCurrentAngle();
                        }
                    }
                    first = true;
                }

                // startSoftStart(joints, trajectory_arrays, joint_count);

                // 以防万一，初次进入视觉时通信没连上
                if (data.joint_angle[1] != 0)
                {
                    state = VISION_STARTING;
                }
                else
                {
                    state = VISION_DISABLED;
                }

                last_update_time = current_time;
            }
            break;

        case VISION_STARTING:
            if (data.enable)
            {
                // 检查是否所有关节空闲
                bool all_free = true;
                for (uint8_t i = 0; i < joint_count; i++)
                {
                    if (joints[i].isCalibrated() && !joints[i].isTrajectoryPlannerFree())
                    {
                        all_free = false;
                        break;
                    }
                }

                if (all_free)
                {
                    // for (uint8_t i = 0; i < joint_count; i++)
                    // {
                    //     if (joints[i].isCalibrated())
                    //     {
                    //         joints[i].stopTrajectoryPlanner();
                    //         trajectory_arrays[i].clear();

                    //         // 缓启动：600ms对齐视觉上一次规划出的位置
                    //         trajectory_arrays[i].add(600, data.joint_angle[i]);

                    //         joints[i].attach(&trajectory_arrays[i]);
                    //         joints[i].startTrajectoryPlanner(current_time);
                    //     }
                    // }
                    state = VISION_RUNNING;
                    last_update_time = current_time;
                }
            }
            else
            {
                reset();
            }
            break;

        case VISION_RUNNING:
            if (data.enable) 
            {
                // 检查目标是否变化
                if (checkTargetsChanged(data) && (current_time - last_update_time) >= data.frequency /*&& !checkTargetsAllZero(data)*/)
                {
                    // 当没收到数据时，data里的joint_angle数据就会为 0，鉴于J2不可能会有运动到 0°的工况，所以加这个判定
                    // if (data.joint_angle[1] == 0)   
                    // {
                    //     state = VISION_DISABLED;
                    //     break;
                    // }
                    
                    // 执行新的轨迹规划
                    for (uint8_t i = 0; i < joint_count; i++)
                    {
                        if (joints[i].isCalibrated())
                        {
                            joints[i].stopTrajectoryPlanner();
                            trajectory_arrays[i].clear();
                            trajectory_arrays[i].add(data.frequency, data.joint_angle[i]);

                            joints[i].attach(&trajectory_arrays[i]);
                            joints[i].startTrajectoryPlanner(current_time);
                        }

                        previous_angles[i] = data.joint_angle[i];
                    }

                    last_update_time = current_time;
                }
            } 
            else 
            {
                reset();
            }
            break;
    }
}

/**
 * @brief 重置控制器状态
 */
void VisionController::reset() 
{
    state = VISION_DISABLED;
    last_update_time = 0;
    first = false;
    soft_start_active = false;
    memset(previous_angles, 0, sizeof(previous_angles));
    // memset(start_angles, 0, sizeof(start_angles));
}
