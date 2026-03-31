// #include "CustomControl.h"

/**
 * @brief 处理自定义控制器接入逻辑
 */
// void CustomControl_Controller(uint8_t Custom_enable, Joint j[6], TrajectoryPointArray tpa1[6], CustomConData CustomCONTROL_data)
// {
//     enum CustomState
//     {
//         disabled, // 未使能，刚使能用于进入缓启动
//         wait,     // 等待缓启动完成
//         running   // 自定义控制接管中
//     };
    
//     static CustomState custom_state = disabled; // 自定义控制器状态
    
//     switch (custom_state)
//     {
//     case disabled:
//         if (Custom_enable == 1)
//         {
//             for (uint8_t i = 0; i < 5; i++)
//             {
//                 if (j[i].isCalibrated() == true) // 关节映射是否完成
//                 {
//                     j[i].stopTrajectoryPlanner();
//                     tpa1[i].clear();
//                     tpa1[i].add(300, CustomCONTROL_data.joint_angle[i]); // 缓启动

//                     j[i].attach(&tpa1[i]);
//                     j[i].startTrajectoryPlanner(HAL_GetTick());
//                 }
//             }
//             custom_state = wait;
//         }
//         break;

//     case wait:
//         if (Custom_enable == 1)
//         {
//             int tmp = 0;
//             for (uint8_t i = 0; i < 5; i++)
//             {
//                 if (j[i].isCalibrated() == true)
//                 {
//                     if (j[i].isTrajectoryPlannerFree())
//                         tmp++;
//                 }
//             }
//             if (tmp == 5)
//             {
//                 custom_state = running;
//             }
//         }
//         else
//         {
//             for (uint8_t i = 0; i < 5; i++)
//             {
//                 if (j[i].isCalibrated() == true)
//                 {
//                     j[i].stopTrajectoryPlanner();
//                 }
//             }
//             custom_state = disabled;
//         }
//         break;

//     case running:
//         if (Custom_enable == 1)
//         {
//             float frequency = 60; // 自定义控制器执行间隔时间，单位为ms
            
//             for (uint8_t i = 0; i < 5; i++)
//             {
//                 if (j[i].isCalibrated() == true)
//                 {
//                     j[i].stopTrajectoryPlanner();
//                     tpa1[i].clear();
//                     tpa1[i].add(frequency, CustomCONTROL_data.joint_angle[i]);
//                     j[i].attach(&tpa1[i]);
//                     j[i].startTrajectoryPlanner(HAL_GetTick());
//                 }
//             }
//         }
//         else
//         {
//             for (uint8_t i = 0; i < 5; i++)
//             {
//                 if (j[i].isCalibrated() == true)
//                 {
//                     j[i].stopTrajectoryPlanner();
//                 }
//             }
//             custom_state = disabled;
//         }
//         break;
//     default:
//         break;
//     }
// }

// /* 全局自定义控制器实例 */
// CustomController custom_controller;

// /**
//  * @brief 自定义控制器更新函数
//  */
// void CustomController::update(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count) {
//     uint32_t current_time = HAL_GetTick();
    
//     switch (state) {
//         case DISABLED:
//             if (data.enable) {
//                 // 进入缓启动
//                 for (uint8_t i = 0; i < joint_count; i++) {
//                     if (i < 6 && joints[i].isCalibrated()) {
//                         joints[i].stopTrajectoryPlanner();
//                         trajectory_arrays[i].clear();
//                         // 缓启动：300ms移动到目标位置
//                         trajectory_arrays[i].add(300, data.joint_angle[i]);
                        
//                         joints[i].attach(&trajectory_arrays[i]);
//                         joints[i].startTrajectoryPlanner(current_time);
//                     }
//                 }
//                 state = WAIT_STARTUP;
//                 last_update_time = current_time;
//             }
//             break;
            
//         case WAIT_STARTUP:
//             if (data.enable) {
//                 // 检查所有关节是否完成缓启动
//                 bool all_free = true;
//                 for (uint8_t i = 0; i < joint_count; i++) {
//                     if (i < 6 && joints[i].isCalibrated()) {
//                         if (!joints[i].isTrajectoryPlannerFree()) {
//                             all_free = false;
//                             break;
//                         }
//                     }
//                 }
                
//                 if (all_free) {
//                     state = RUNNING;
//                     last_update_time = current_time;
//                 }
//             } else {
//                 // 外部禁用了自定义控制
//                 for (uint8_t i = 0; i < joint_count; i++) {
//                     if (i < 6 && joints[i].isCalibrated()) {
//                         joints[i].stopTrajectoryPlanner();
//                     }
//                 }
//                 state = DISABLED;
//             }
//             break;
            
//         case RUNNING:
//             if (data.enable) {
//                 // 按指定频率更新轨迹
//                 if ((current_time - last_update_time) >= data.frequency) {
//                     for (uint8_t i = 0; i < joint_count; i++) {
//                         if (i < 6 && joints[i].isCalibrated()) {
//                             joints[i].stopTrajectoryPlanner();
//                             trajectory_arrays[i].clear();
//                             trajectory_arrays[i].add(data.frequency, data.joint_angle[i]);
                            
//                             joints[i].attach(&trajectory_arrays[i]);
//                             joints[i].startTrajectoryPlanner(current_time);
//                         }
//                     }
//                     last_update_time = current_time;
//                 }
//             } else {
//                 // 外部禁用了自定义控制
//                 for (uint8_t i = 0; i < joint_count; i++) {
//                     if (i < 6 && joints[i].isCalibrated()) {
//                         joints[i].stopTrajectoryPlanner();
//                     }
//                 }
//                 state = DISABLED;
//             }
//             break;
//     }
// }

// /**
//  * @brief 重置控制器状态
//  */
// void CustomController::reset() {
//     state = DISABLED;
//     last_update_time = 0;
// }

#include "CustomControl.h"

/* 全局自定义控制器实例 */
CustomController custom_controller;

/**
 * @brief 启动缓启动过程
 */
void CustomController::startSoftStart(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count) 
{
    uint32_t current_time = HAL_GetTick();
    
    for (uint8_t i = 0; i < joint_count; i++) 
    {
        if (joints[i].isCalibrated()) 
        {
            // 记录起始角度
            start_angles[i] = joints[i].getCurrentAngle();
            
            joints[i].stopTrajectoryPlanner();
            trajectory_arrays[i].clear();
            
            // 缓启动：600ms平滑移动到目标位置
            trajectory_arrays[i].add(600, data.joint_angle[i]);
            
            joints[i].attach(&trajectory_arrays[i]);
            joints[i].startTrajectoryPlanner(current_time);
        }
    }
}

/**
 * @brief 执行正常控制
 */
void CustomController::executeControl(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count) 
{
    uint32_t current_time = HAL_GetTick();
    
    // 按指定频率更新轨迹
    if ((current_time - last_update_time) >= data.frequency) 
    {
        for (uint8_t i = 0; i < joint_count; i++) 
        {
            if (joints[i].isCalibrated()) 
            {
                joints[i].stopTrajectoryPlanner();
                trajectory_arrays[i].clear();
                trajectory_arrays[i].add(data.frequency, data.joint_angle[i]);
                
                if(joints[i].isTrajectoryPlannerFree() == true)
                {
                    joints[i].attach(&trajectory_arrays[i]);
                    joints[i].startTrajectoryPlanner(current_time);
                }
            }
        }
        last_update_time = current_time;
    }
}

/**
 * @brief 更新自定义控制器
 */
void CustomController::update(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count) 
{
    uint32_t current_time = HAL_GetTick();
    
    switch (state) {
        case CUSTOM_DISABLED:
            if (data.enable) 
            {
                startSoftStart(data, joints, trajectory_arrays, joint_count);
                state = CUSTOM_STARTING;
                last_update_time = current_time;
            }
            break;
            
        case CUSTOM_STARTING:
            if (data.enable) 
            {
                // 检查所有关节是否完成缓启动
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
                    state = CUSTOM_RUNNING;
                }
            } 
            else 
            {
                reset();
            }
            break;
            
        case CUSTOM_RUNNING:
            if (data.enable) 
            {
                executeControl(data, joints, trajectory_arrays, joint_count);
            } 
            else 
            {
                reset();
            }
            break;
    }
}

/**
 * @brief 获取关节当前目标角度
 */
void CustomController::getCurrentTargets(float targets[6]) const 
{
    // 这里可以根据需要返回当前目标角度
    // 在实际应用中，可以从最新的CustomData中获取
}

/**
 * @brief 重置控制器状态
 */
void CustomController::reset() 
{
    state = CUSTOM_DISABLED;
    last_update_time = 0;
    memset(start_angles, 0, sizeof(start_angles));
}