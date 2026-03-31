// /* 自定义控制数据结构 */
// typedef struct {
//     float joint_angle[6];      // 目标关节角度
//     bool enable;               // 自定义控制使能
//     uint32_t frequency;        // 控制频率（ms）
// } CustomData;

// /* 自定义控制器类 */
// class CustomController {
// private:
//     enum CustomState {
//         DISABLED,
//         WAIT_STARTUP,
//         RUNNING
//     };
    
//     CustomState state;
//     uint32_t last_update_time;
    
// public:
//     CustomController() : state(DISABLED), last_update_time(0) {}
    
//     /**
//      * @brief 处理自定义控制器逻辑
//      * @param data 自定义控制数据
//      * @param joints 关节数组
//      * @param trajectory_arrays 轨迹数组
//      * @param joint_count 关节数量
//      */
//     void update(CustomData& data, Joint joints[], 
//                 TrajectoryPointArray trajectory_arrays[], 
//                 uint8_t joint_count);
    
//     /**
//      * @brief 重置控制器状态
//      */
//     void reset();
    
//     /**
//      * @brief 获取当前状态
//      */
//     bool isEnabled() const { return state != DISABLED; }
// };

// extern "C"
// {
//     /* 全局自定义控制器实例 */
//     extern CustomController custom_controller;
    
// }

#ifndef _CUSTOMCONTROL_H
#define _CUSTOMCONTROL_H

#ifdef __cplusplus
#include "internal.h"
#include "SRML.h"
#include "Joint.h"
#include "Trajectory.h"

/* 自定义控制数据结构 */
typedef struct {
    float joint_angle[6];      // 目标关节角度
    bool enable;               // 自定义控制使能
    uint32_t frequency;        // 控制频率（ms）
} CustomData;

/* 自定义控制器状态 */
enum CustomState {
    CUSTOM_DISABLED,
    CUSTOM_STARTING,
    CUSTOM_RUNNING
};

/* 纯净的自定义控制器类 - 只负责关节角度控制 */
class CustomController {
private:
    CustomState state;
    uint32_t last_update_time;
    float start_angles[6];      // 缓启动起始角度
    
public:
    CustomController() : state(CUSTOM_DISABLED), last_update_time(0) 
    {
        memset(start_angles, 0, sizeof(start_angles));
    }
    
    /**
     * @brief 处理自定义控制器逻辑
     * @param data 自定义控制数据
     * @param joints 关节数组
     * @param trajectory_arrays 轨迹数组
     * @param joint_count 关节数量
     */
    void update(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count);
    
    /**
     * @brief 重置控制器状态
     */
    void reset();
    
    /**
     * @brief 获取当前状态
     */
    CustomState getState() const { return state; }
    
    /**
     * @brief 是否使能
     */
    bool isEnabled() const { return state != CUSTOM_DISABLED; }
    
    /**
     * @brief 是否运行中
     */
    bool isRunning() const { return state == CUSTOM_RUNNING; }
    
    /**
     * @brief 获取关节当前目标角度
     */
    void getCurrentTargets(float targets[6]) const;
    
private:
    /**
     * @brief 启动缓启动过程
     */
    void startSoftStart(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count);
    
    /**
     * @brief 执行正常控制
     */
    void executeControl(CustomData& data, Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count);
};

extern "C" {
    /* 全局自定义控制器实例 */
    extern CustomController custom_controller;
}

#endif
#endif