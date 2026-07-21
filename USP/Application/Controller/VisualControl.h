#ifndef __VISIONCONTROL_H__
#define __VISIONCONTROL_H__

#ifdef __cplusplus
#include "internal.h"
#include "SRML.h"
#include "Joint.h"
#include "Trajectory.h"

/* 视觉控制数据结构 */
typedef struct {
    float joint_angle[7];      // 目标关节角度
    bool enable;               // 视觉控制使能
    uint32_t frequency;        // 控制频率（ms）
    // uint32_t move_duration;    // 移动持续时间（ms）
} VisionData;

/* 视觉控制状态 */
enum VisionState {
    VISION_DISABLED,
    VISION_STARTING,
    VISION_RUNNING
};

/* 视觉控制器类 - 只负责视觉目标角度控制 */
class VisionController {
private:
    VisionState state;
    uint32_t last_update_time;
    float previous_angles[7];   // 之前的目标角度
    float start_angles[7];      // 缓启动起始角度
    bool first;                 // 是否第一次进入视觉模式
    bool soft_start_active;     // F 键起始动作是否正在执行
    
public:
    VisionController() : state(VISION_DISABLED), last_update_time(0), soft_start_active(false) {
        memset(previous_angles, 0, sizeof(previous_angles));
        // memset(start_angles, 0, sizeof(start_angles));
        start_angles[0] = 0;
        start_angles[1] = 138.55;
        start_angles[2] = 111.28;
        start_angles[3] = -52.63;
        start_angles[4] = 0;
        start_angles[5] = 55.06;
        start_angles[6] = 0;
        first = false;
    }
    
    /**
     * @brief 更新视觉控制
     */
    void update(VisionData& data, Joint joints[], 
                TrajectoryPointArray trajectory_arrays[], 
                uint8_t joint_count);

    /**
     * @brief 移动到视觉控制起始角度
     */
    void startSoftStart(Joint joints[], TrajectoryPointArray trajectory_arrays[], uint8_t joint_count);
    
    /**
     * @brief 重置控制器状态
     */
    void reset();
    
    /**
     * @brief 获取当前状态
     */
    VisionState getState() const { return state; }
    
    /**
     * @brief 是否使能
     */
    bool isEnabled() const { return state != VISION_DISABLED; }
    
    /**
     * @brief 是否运行中
     */
    bool isRunning() const { return state == VISION_RUNNING; }
    
private:
    /**
     * @brief 检查目标是否变化
     */
    bool checkTargetsChanged(VisionData& data);

    /**
     * @brief 检查目标是否都为 0（此时相当于通信中断，单片机没有收到数据）
     */
    bool checkTargetsAllZero(VisionData& data);
};

extern "C" {
    extern VisionController vision_controller;
}

#endif
#endif
