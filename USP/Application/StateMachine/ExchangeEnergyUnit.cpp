#include "ExchangeEnergyUnit.h"
#include "internal.h"
#include "Keyboard.h"
#include "srml_std_lib.h"

/* 全局状态变量 */
/* 资源管理变量定义 */
uint8_t EnergyUnitCount = 0;         // 当前携带的能量单元数量
// uint8_t ExchangeSuccessCount = 0;    // 成功兑换次数
static ExchangeMode current_mode = ExchangeMode::ACTION_GROUP;
static ExchangeState current_state = ExchangeState::IDLE;
static bool action_in_progress = false;
static ActionGroup_enum current_action = ActionGroup_enum::NuLLAction;
static uint8_t add_once_flag = 0;

/* 兑换参数（外部配置） */
static ExchangeParams exchange_params = {
    .insert_distance = 100.0f,      // 默认插入100mm
    .translation_distance = 100.0f,// 默认平移100mm
    .rotation_angle = 90.0f,       // 默认旋转90度
    .translation_required = true,  // 默认需要平移
    .rotation_required = true,     // 默认需要旋转
    .timeout_ms = 20000,           // 20秒超时
    .retry_count = 3               // 最多重试3次
};

/* 状态管理变量 */
static uint32_t state_start_time = 0;
static uint32_t state_timeout = 0;
static uint8_t current_retry = 0;

/* 控制数据 */
static CustomData custom_control_data;
static VisionData vision_control_data;

/* 正向运动学计算函数（简化版本） */
struct EndEffectorPose {
    float x, y, z;      // 位置（mm）
    float rx, ry, rz;   // 姿态（欧拉角，度）
};

/**
 * @brief 简化的正向运动学计算
 * @param joint_angles 6个关节角度
 * @return 末端执行器位姿
 */
EndEffectorPose forwardKinematics(float joint_angles[6]) {
    EndEffectorPose pose = {0};
    
    // 简化计算：这里需要根据实际的机器人DH参数来实现
    // 这里只是一个示例，实际应用中需要根据机器人模型实现
    
    // 假设简单模型：前三个关节决定位置，后三个关节决定姿态
    pose.x = joint_angles[0] * 10.0f;  // 示例比例系数
    pose.y = joint_angles[1] * 10.0f;
    pose.z = joint_angles[2] * 10.0f;
    pose.rx = joint_angles[3];
    pose.ry = joint_angles[4];
    pose.rz = joint_angles[5];
    
    return pose;
}

/**
 * @brief 简化的逆运动学计算
 * @param target_pose 目标末端位姿
 * @param current_angles 当前关节角度
 * @param result_angles 计算得到的关节角度
 * @return 是否成功
 */
bool inverseKinematics(const EndEffectorPose& target_pose, 
                      const float current_angles[6],
                      float result_angles[6]) {
    // 简化计算：这里需要根据实际的机器人DH参数来实现
    // 这里只是一个示例，实际应用中需要根据机器人模型实现
    
    // 简单假设：关节角度直接与末端位置成比例
    for (int i = 0; i < 6; i++) {
        // 保持当前角度作为基础，根据目标位姿调整
        result_angles[i] = current_angles[i];
    }
    
    // 调整位置（简化处理）
    result_angles[0] += target_pose.x / 100.0f;
    result_angles[1] += target_pose.y / 100.0f;
    result_angles[2] += target_pose.z / 100.0f;
    result_angles[3] += target_pose.rx;
    result_angles[4] += target_pose.ry;
    result_angles[5] += target_pose.rz;
    
    return true;
}

/**
 * @brief 计算插入动作的关节角度
 */
bool calculateInsertionTrajectory(float insert_distance, float trajectory_angles[6][10]) 
{
    // 获取当前关节角度
    float current_angles[7];
    for (int i = 0; i < 7; i++) 
    {
        current_angles[i] = joint[i].getCurrentAngle();
    }
    
    // 计算当前末端位姿
    EndEffectorPose current_pose = forwardKinematics(current_angles);
    
    // 计算目标位姿：向前插入指定距离
    EndEffectorPose target_pose = current_pose;
    target_pose.x += insert_distance;  // 向前插入
    
    // 生成轨迹点（10个点）
    for (int point = 0; point < 10; point++) 
    {
        float t = (float)(point + 1) / 10.0f;  // 插值系数
        
        // 线性插值
        EndEffectorPose intermediate_pose;
        intermediate_pose.x = current_pose.x + t * (target_pose.x - current_pose.x);
        intermediate_pose.y = current_pose.y + t * (target_pose.y - current_pose.y);
        intermediate_pose.z = current_pose.z + t * (target_pose.z - current_pose.z);
        intermediate_pose.rx = current_pose.rx + t * (target_pose.rx - current_pose.rx);
        intermediate_pose.ry = current_pose.ry + t * (target_pose.ry - current_pose.ry);
        intermediate_pose.rz = current_pose.rz + t * (target_pose.rz - current_pose.rz);
        
        // 逆运动学计算
        float result[6];
        if (!inverseKinematics(intermediate_pose, current_angles, result)) 
        {
            return false;
        }
        
        for (int i = 0; i < 6; i++) 
        {
            trajectory_angles[i][point] = result[i];
        }
    }
    
    return true;
}

/**
 * @brief 计算平移动作的关节角度
 */
bool calculateTranslationTrajectory(float translation_distance, float trajectory_angles[6][10]) 
{
    // 获取当前关节角度
    float current_angles[7];
    for (int i = 0; i < 7; i++) 
    {
        current_angles[i] = joint[i].getCurrentAngle();
    }
    
    // 计算当前末端位姿
    EndEffectorPose current_pose = forwardKinematics(current_angles);
    
    // 计算目标位姿：向右平移指定距离
    EndEffectorPose target_pose = current_pose;
    target_pose.y += translation_distance;  // 向右平移
    
    // 生成轨迹点（10个点）
    for (int point = 0; point < 10; point++) 
    {
        float t = (float)(point + 1) / 10.0f;  // 插值系数
        
        // 线性插值
        EndEffectorPose intermediate_pose;
        intermediate_pose.x = current_pose.x + t * (target_pose.x - current_pose.x);
        intermediate_pose.y = current_pose.y + t * (target_pose.y - current_pose.y);
        intermediate_pose.z = current_pose.z + t * (target_pose.z - current_pose.z);
        intermediate_pose.rx = current_pose.rx + t * (target_pose.rx - current_pose.rx);
        intermediate_pose.ry = current_pose.ry + t * (target_pose.ry - current_pose.ry);
        intermediate_pose.rz = current_pose.rz + t * (target_pose.rz - current_pose.rz);
        
        // 逆运动学计算
        float result[6];
        if (!inverseKinematics(intermediate_pose, current_angles, result)) 
        {
            return false;
        }
        
        for (int i = 0; i < 7; i++) 
        {
            trajectory_angles[i][point] = result[i];
        }
    }
    
    return true;
}

/**
 * @brief 执行计算得到的轨迹
 */
bool executeCalculatedTrajectory(float trajectory_angles[6][10], uint32_t duration_ms) 
{
    uint32_t current_time = HAL_GetTick();
    uint32_t point_duration = duration_ms / 10;
    
    // 为每个关节规划轨迹
    for (int i = 0; i < 7; i++) 
    {
        if (joint[i].isCalibrated()) 
        {
            joint[i].stopTrajectoryPlanner();
            joint_trajectory_point_array[i].clear();
            
            // 添加轨迹点
            for (int point = 0; point < 10; point++) 
            {
                joint_trajectory_point_array[i].add(point_duration, trajectory_angles[i][point]);
            }
            
            joint[i].attach(&joint_trajectory_point_array[i]);
            joint[i].startTrajectoryPlanner(current_time);
        }
    }

    // for(int i = 0; i < 6; i++)
    // {
    //     joint[i].takeOver(current_time);    
    // }
    
    return true;
}

/**
 * @brief 初始化兑换系统
 */
void ExchangeEnergyUnit_enterAction() 
{
    g_top_state_id = TopStateId::ExchangeEnergyUnit;

    // 清理所有轨迹
    for (uint8_t i = 0; i < 7; i++) 
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }
    
    // 重置状态
    current_state = ExchangeState::IDLE;
    current_mode = ExchangeMode::ACTION_GROUP;
    action_in_progress = false;
    current_action = ActionGroup_enum::NuLLAction;
    add_once_flag = 0;
    state_start_time = 0;
    state_timeout = 0;
    current_retry = 0;

    // if(reBottomdata.L_Store == 1 && reBottomdata.R_Store == 1)
    // EnergyUnitCount =2;
    // else if((reBottomdata.L_Store == 0 && reBottomdata.R_Store == 1) || (reBottomdata.L_Store == 1 && reBottomdata.R_Store == 0))
    // EnergyUnitCount = 1;
    // else if(reBottomdata.L_Store == 0 && reBottomdata.R_Store == 0)
    // EnergyUnitCount = 0;
    
    // 初始化控制数据
    custom_control_data.enable = false;
    custom_control_data.frequency = 60;
    
    vision_control_data.enable = false;
    vision_control_data.frequency = 33;
    // vision_control_data.move_duration = 500;
    
    // 重置控制器
    custom_controller.reset();
    vision_controller.reset();
    
    // 指示灯
    WS_indicator[4] = WS_Blue;
    WS_indicator[5] = WS_Black;
}

/**
 * @brief 检查动作组是否完成
 */
bool checkActionGroupComplete() 
{
    for (uint8_t i = 0; i < 7; i++) 
    {
        if (!joint[i].isTrajectoryPlannerFree()) 
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 设置超时时间
 */
void setStateTimeout(uint32_t timeout_ms) 
{
    state_timeout = timeout_ms;
    state_start_time = HAL_GetTick();
}

/**
 * @brief 检查是否超时
 */
bool checkStateTimeout() 
{
    if (state_timeout == 0) return false;
    return (HAL_GetTick() - state_start_time) > state_timeout;
}

/**
 * @brief 切换到下一个状态
 */
void gotoNextState(ExchangeState next_state) 
{
    uint32_t current_time = HAL_GetTick();
    if ((current_time - state_start_time) > 300)    return;

    for (uint8_t i = 0; i < 7; i++) 
    {
        joint[i].stopTrajectoryPlanner();
        joint_trajectory_point_array[i].clear();
    }
    current_state = next_state;
    action_in_progress = false;
    current_action = ActionGroup_enum::NuLLAction;
    add_once_flag = 0;
    state_start_time = current_time;
}

/**
 * @brief 处理动作组模式
 */
void handleActionGroupMode() 
{
    bool action_complete = checkActionGroupComplete();
    
    switch (current_state) 
    {
        case ExchangeState::IDLE:
            WS_indicator[4] = WS_Blue;
            WS_indicator[5] = WS_Black;
            break;
            
        case ExchangeState::INSERTING:
            if (!action_in_progress) 
            {
                // 计算并执行插入轨迹
                float trajectory_angles[6][10];
                if (calculateInsertionTrajectory(exchange_params.insert_distance, trajectory_angles)) 
                {
                    if (executeCalculatedTrajectory(trajectory_angles, 3000)) 
                    { 
                        // 3秒插入
                        // setStateTimeout(5000);
                        action_in_progress = true;
                    } 
                    // else 
                    // {
                    //     gotoNextState(ExchangeState::VERIFYING);
                    // }
                } 
                else 
                {
                    gotoNextState(ExchangeState::VERIFYING);
                }
            }
            
            // 动作组中一定要上一个动作结束才能进入下一个装配状态
            if (action_complete) 
            {
                // 插入完成，检查是否需要平移
                if((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B)))
                {
                    if (exchange_params.translation_required) 
                    {
                        gotoNextState(ExchangeState::TRANSLATING);
                    } 
                    // else if (exchange_params.rotation_required) 
                    // {
                    //     gotoNextState(ExchangeState::ROTATING);
                    // } 
                    else 
                    {
                        gotoNextState(ExchangeState::VERIFYING);
                    }
                }
            }
            
            // if (checkStateTimeout()) 
            // {

                // if (current_retry < exchange_params.retry_count) 
                // {
                //     current_retry++;
                //     action_in_progress = false;
                // } 
                // else 
                // {
                    // gotoNextState(ExchangeState::VERIFYING);
                // }
            // }
            break;
            
        case ExchangeState::TRANSLATING:
            if (!action_in_progress) 
            {
                // 计算并执行平移轨迹
                float trajectory_angles[6][10];
                if (calculateTranslationTrajectory(exchange_params.translation_distance, trajectory_angles)) 
                {
                    if (executeCalculatedTrajectory(trajectory_angles, 2000)) 
                    { 
                        // 2秒平移
                        // setStateTimeout(3000);
                        action_in_progress = true;
                    } 
                    // else 
                    // {
                    //     gotoNextState(ExchangeState::VERIFYING);
                    // }
                } 
                else 
                {
                    gotoNextState(ExchangeState::VERIFYING);
                }
            }
            
            if (action_complete) 
            {
                // 平移完成，检查是否需要旋转
                if((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B)))
                {
                    if (exchange_params.rotation_required) 
                    {
                        gotoNextState(ExchangeState::ROTATING);
                    } 
                    else 
                    {
                        gotoNextState(ExchangeState::VERIFYING);
                    }
                }
            }
            
            // if (checkStateTimeout()) 
            // {
            //     gotoNextState(ExchangeState::VERIFYING);
            // }
            break;
            
        // 动作组很难做三级装配的旋转，此处得切换至视觉或者自定义控制器模式
        case ExchangeState::ROTATING:
            if (!action_in_progress) 
            {
                // 执行旋转动作组
                current_action = ActionGroup_enum::ExchangeRotate;
                add_once_flag = 1;
                action_in_progress = true;
                // ExchangeRotate动作组的设定是其他关节保持当前位置不变，末端旋转，需后续加上设定
                
                ActionGroup_Choose(joint_trajectory_point_array, current_action, add_once_flag);
                ActionGroup_Execute(joint, joint_trajectory_point_array, 6, add_once_flag);
                // setStateTimeout(3000);
            }
            
            if (action_complete) 
            {
                if((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B)))
                gotoNextState(ExchangeState::VERIFYING);
            }
            
            // if (checkStateTimeout()) 
            // {
            //     gotoNextState(ExchangeState::VERIFYING);
            // }
            break;
            
        case ExchangeState::VERIFYING:
            // 等待兑换验证
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::IDLE);
            }
        //     static uint32_t verify_start = 0;
        //     if (verify_start == 0) 
        //     {
        //         verify_start = HAL_GetTick();
        //         WS_indicator[4] = WS_Yellow;
        //         WS_indicator[5] = WS_Yellow;
        //     }
            
        //     // 3秒验证时间
        //     if ((HAL_GetTick() - verify_start) > 3000) 
        //     {
        //         verify_start = 0;
                
        //         // 兑换成功
        //         if (EnergyUnitCount > 0) 
        //         {
        //             EnergyUnitCount--;
        //             ExchangeSuccessCount++;
                    
        //             if((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B)))
        //             gotoNextState(ExchangeState::COMPLETED);
        //         } 
        //         else 
        //         {
        //             gotoNextState(ExchangeState::FAILED);
        //         }
        //     }
        //     break;
            
        // case ExchangeState::COMPLETED:
        //     WS_indicator[4] = WS_Green;
        //     WS_indicator[5] = WS_Green;
            
        //     // 保持在完成状态
        //     if ((DR16.IsKeyPress(DR16_KEY_R)) || (VT13.IsKeyPress(VT13_KEY_R))) 
        //     {
        //         gotoNextState(ExchangeState::IDLE);
        //         current_retry = 0;
        //     }
        //     break;
            
        // case ExchangeState::FAILED:
        //     WS_indicator[4] = WS_Red;
        //     WS_indicator[5] = WS_Red;
            
        //     // 失败处理
        //     if ((DR16.IsKeyPress(DR16_KEY_R)) || (VT13.IsKeyPress(VT13_KEY_R))) 
        //     {
        //         if (current_retry < exchange_params.retry_count) 
        //         {
        //             current_retry++;
        //             gotoNextState(ExchangeState::INSERTING);
        //         } 
        //         else 
        //         {
        //             gotoNextState(ExchangeState::IDLE);
        //             current_retry = 0;
        //         }
        //     }
        //     break;
    }
}

/**
 * @brief 处理自定义控制器模式
 */
void handleCustomControlMode() 
{
    // 接收自定义控制器数据
    if (VT13.GetStatus() == VT13_ESTABLISHED) 
    {
        for (int i = 0; i < 7; i++) 
        {
            custom_control_data.joint_angle[i] = VT13.Datapack_custom.joint_angle[i];
        }
        
        // 更新自定义控制器
        switch (current_state)
        {
        case ExchangeState::IDLE:
            WS_indicator[4] = WS_Blue;
            WS_indicator[5] = WS_Black;
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::INSERTING);
            }
            break;

        case ExchangeState::INSERTING:
        case ExchangeState::TRANSLATING:
        case ExchangeState::ROTATING:
            custom_controller.update(custom_control_data, joint, joint_trajectory_point_array, 7);
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::VERIFYING);
            }
            break;

        case ExchangeState::VERIFYING:
            // 等待兑换验证
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::IDLE);
            }

        //     static uint32_t verify_start = 0;
        //     if (verify_start == 0) 
        //     {
        //         verify_start = HAL_GetTick();
        //         WS_indicator[4] = WS_Yellow;
        //         WS_indicator[5] = WS_Yellow;
        //     }
            
        //     // 3秒验证时间
        //     if ((HAL_GetTick() - verify_start) > 3000) 
        //     {
        //         verify_start = 0;
                
        //         // 兑换成功
        //         if (EnergyUnitCount > 0) 
        //         {
        //             EnergyUnitCount--;
        //             ExchangeSuccessCount++;
                    
        //             if((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B)))
        //             gotoNextState(ExchangeState::COMPLETED);
        //         } 
        //         else 
        //         {
        //             gotoNextState(ExchangeState::FAILED);
        //         }
        //     }
        //     break;
            
        // case ExchangeState::COMPLETED:
        //     WS_indicator[4] = WS_Green;
        //     WS_indicator[5] = WS_Green;
            
        //     // 保持在完成状态
        //     if ((DR16.IsKeyPress(DR16_KEY_R)) || (VT13.IsKeyPress(VT13_KEY_R))) 
        //     {
        //         gotoNextState(ExchangeState::IDLE);
        //         current_retry = 0;
        //     }
        //     break;
            
        // case ExchangeState::FAILED:
        //     WS_indicator[4] = WS_Red;
        //     WS_indicator[5] = WS_Red;
            
        //     // 失败处理
        //     if ((DR16.IsKeyPress(DR16_KEY_R)) || (VT13.IsKeyPress(VT13_KEY_R))) 
        //     {
        //         if (current_retry < exchange_params.retry_count) 
        //         {
        //             current_retry++;
        //             gotoNextState(ExchangeState::INSERTING);
        //         } 
        //         else 
        //         {
        //             gotoNextState(ExchangeState::IDLE);
        //             current_retry = 0;
        //         }
        //     }
        //     break;
       
        }
        
    }
    
    // // 处理兑换状态
    // if (current_state == ExchangeState::IDLE) 
    // {
    //     // 等待开始兑换信号
    //     if (DR16.IsKeyPress(DR16_KEY_E)) 
    //     {
    //         gotoNextState(ExchangeState::INSERTING);
    //     }
    // }
    
    // 手动控制流程（为了方便与动作组随时切换，状态对齐，不使用动作组模式的话暂时先注释）
    // if (current_state == ExchangeState::INSERTING) 
    // {
    //     if ((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B))) 
    //     {
    //         gotoNextState(ExchangeState::TRANSLATING);
    //     }
    // }
    // else if (current_state == ExchangeState::TRANSLATING) 
    // {
    //     if ((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B))) 
    //     {
    //         gotoNextState(ExchangeState::ROTATING);
    //     }
    // }
    // else if (current_state == ExchangeState::ROTATING) 
    // {
    //     if ((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B))) 
    //     {
    //         gotoNextState(ExchangeState::VERIFYING);
    //     }
    // }
}

/**
 * @brief 处理视觉控制模式
 */
void handleVisionControlMode() 
{
    // 接收视觉数据
   if (ulTaskNotifyTake(pdTRUE, 0) == 55)
   {
        for (int i = 0; i < 7; i++)
        {
            vision_control_data.joint_angle[i] = reVisiondata.joint_target[i];
        }

        // 更新视觉控制
        switch (current_state)
        {
        case ExchangeState::IDLE:
            WS_indicator[4] = WS_Blue;
            WS_indicator[5] = WS_Black;
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::INSERTING);
            }
            break;

        case ExchangeState::INSERTING:
        case ExchangeState::TRANSLATING:
        case ExchangeState::ROTATING:
            vision_controller.update(vision_control_data, joint, joint_trajectory_point_array, 6);
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::VERIFYING);
            }
            break;

        case ExchangeState::VERIFYING:
            // 等待兑换验证
            if (DR16.IsKeyPress(DR16_KEY_B) || VT13.IsKeyPress(VT13_KEY_B))
            {
                gotoNextState(ExchangeState::IDLE);
            }
        //     static uint32_t verify_start = 0;
        //     if (verify_start == 0) 
        //     {
        //         verify_start = HAL_GetTick();
        //         WS_indicator[4] = WS_Yellow;
        //         WS_indicator[5] = WS_Yellow;
        //     }
            
        //     // 3秒验证时间
        //     if ((HAL_GetTick() - verify_start) > 3000) 
        //     {
        //         verify_start = 0;
                
        //         // 兑换成功
        //         if (EnergyUnitCount > 0) 
        //         {
        //             EnergyUnitCount--;
        //             ExchangeSuccessCount++;
                    
        //             if((DR16.IsKeyPress(DR16_KEY_B)) || (VT13.IsKeyPress(VT13_KEY_B)))
        //             gotoNextState(ExchangeState::COMPLETED);
        //         } 
        //         else 
        //         {
        //             gotoNextState(ExchangeState::FAILED);
        //         }
        //     }
        //     break;
            
        // case ExchangeState::COMPLETED:
        //     WS_indicator[4] = WS_Green;
        //     WS_indicator[5] = WS_Green;
            
        //     // 保持在完成状态
        //     if ((DR16.IsKeyPress(DR16_KEY_R)) || (VT13.IsKeyPress(VT13_KEY_R))) 
        //     {
        //         gotoNextState(ExchangeState::IDLE);
        //         current_retry = 0;
        //     }
        //     break;
            
        // case ExchangeState::FAILED:
        //     WS_indicator[4] = WS_Red;
        //     WS_indicator[5] = WS_Red;
            
        //     // 失败处理
        //     if ((DR16.IsKeyPress(DR16_KEY_R)) || (VT13.IsKeyPress(VT13_KEY_R))) 
        //     {
        //         if (current_retry < exchange_params.retry_count) 
        //         {
        //             current_retry++;
        //             gotoNextState(ExchangeState::INSERTING);
        //         } 
        //         else 
        //         {
        //             gotoNextState(ExchangeState::IDLE);
        //             current_retry = 0;
        //         }
        //     }
        //     break;
       
        }
       
    }
    
    // // 处理兑换状态
    // if (current_state == ExchangeState::IDLE) 
    // {
    //     // 视觉模式下自动开始兑换
    //     static uint32_t idle_start = 0;
    //     if (idle_start == 0) 
    //     {
    //         idle_start = HAL_GetTick();
    //     }
        
    //     // 等待2秒后自动开始
    //     if ((HAL_GetTick() - idle_start) > 2000)
    //     {
    //         idle_start = 0;
    //         gotoNextState(ExchangeState::INSERTING);
    //     }
    // }
    
    // // 视觉模式下自动状态切换
    // static uint32_t state_start = 0;
    // if (state_start == 0) 
    // {
    //     state_start = HAL_GetTick();
    // }
    
    // // 每个状态停留一定时间后自动切换到下一个
    // uint32_t state_duration = HAL_GetTick() - state_start;
    
    // switch (current_state) 
    // {
    //     case ExchangeState::IDLE:

    //         break;
    //     case ExchangeState::INSERTING:
    //         if (state_duration > 3000) 
    //         { 
    //             // 插入3秒
    //             state_start = 0;
    //             if (exchange_params.translation_required) 
    //             {
    //                 gotoNextState(ExchangeState::TRANSLATING);
    //             } else if (exchange_params.rotation_required) 
    //             {
    //                 gotoNextState(ExchangeState::ROTATING);
    //             } else 
    //             {
    //                 gotoNextState(ExchangeState::VERIFYING);
    //             }
    //         }
    //         break;
            
    //     case ExchangeState::TRANSLATING:
    //         if (state_duration > 2000) 
    //         { // 平移2秒
    //             state_start = 0;
    //             if (exchange_params.rotation_required) 
    //             {
    //                 gotoNextState(ExchangeState::ROTATING);
    //             } 
    //             else 
    //             {
    //                 gotoNextState(ExchangeState::VERIFYING);
    //             }
    //         }
    //         break;
            
    //     case ExchangeState::ROTATING:
    //         if (state_duration > 2000) 
    //         { 
    //             // 旋转2秒
    //             state_start = 0;
    //             gotoNextState(ExchangeState::VERIFYING);
    //         }
    //         break;
            
    //     case ExchangeState::VERIFYING:
    //         if (state_duration > 3000) 
    //         { 
    //             // 验证3秒
    //             state_start = 0;
    //             if (EnergyUnitCount > 0) 
    //             {
    //                 EnergyUnitCount--;
    //                 ExchangeSuccessCount++;
    //                 gotoNextState(ExchangeState::COMPLETED);
    //             } 
    //             else 
    //             {
    //                 gotoNextState(ExchangeState::FAILED);
    //             }
    //         }
    //         break;
    //     case ExchangeState::COMPLETED:

    //         break;
    //     case ExchangeState::FAILED:
            
    //         break;

    // }
}

/**
 * @brief 执行关节控制接管
 */
void executeJoint() 
{
    uint32_t current_time = HAL_GetTick();
    
    if (current_mode == ExchangeMode::CUSTOM_CONTROL && custom_controller.isEnabled()) 
    {
        // 自定义控制模式
        for (uint8_t i = 0; i < 7; i++) 
        {
            joint[i].takeOver(current_time);
        }
    } 
    else if (current_mode == ExchangeMode::VISION_CONTROL && vision_controller.isEnabled()) 
    {
        // 视觉控制模式
        for (uint8_t i = 0; i < 7; i++) 
        {
            joint[i].takeOver(current_time);
        }
    } 
    else 
    {
        // 动作组模式
        for (uint8_t i = 0; i < 7; i++) 
        {
            joint[i].takeOver(current_time);
        }
    }
}

/**
 * @brief 更新指示灯
 */
void updateIndicators() 
{
    // 控制模式指示灯
    switch (current_mode) 
    {
        case ExchangeMode::ACTION_GROUP:
            WS_indicator[0] = WS_Green;
            break;
        case ExchangeMode::CUSTOM_CONTROL:
            WS_indicator[0] = WS_Blue;
            break;
        case ExchangeMode::VISION_CONTROL:
            WS_indicator[0] = WS_Yellow;
            break;
    }
    
    // 兑换状态指示灯
    switch (current_state) 
    {
        case ExchangeState::IDLE:
            WS_indicator[1] = WS_Blue;
            break;
        case ExchangeState::VERIFYING:
            WS_indicator[1] = WS_Green;
            break;
        // case ExchangeState::FAILED:
        //     WS_indicator[1] = WS_Red;
        //     break;
        default:
            // 执行中闪烁
            if ((HAL_GetTick() / 500) % 2 == 0) 
            {
                WS_indicator[1] = WS_Yellow;
            } 
            else 
            {
                WS_indicator[1] = WS_Black;
            }
            break;
    }
}

/**
 * @brief 处理模式切换
 */
void handleModeSwitch() 
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 防抖处理
    if (current_time - last_press_time < 300) return;
    
    // G键切换模式
    if ((DR16.IsKeyPress(DR16_KEY_G)) || (VT13.IsKeyPress(VT13_KEY_G))) 
    {
        last_press_time = current_time;

         // 动作组模式下必须等动作完成再切换 
        if (current_mode == ExchangeMode::ACTION_GROUP)
        {
            if (!checkActionGroupComplete()) return;
        }
        
        // 停止所有关节轨迹规划
        for (uint8_t i = 0; i < 7; i++) 
        {
            joint[i].stopTrajectoryPlanner();
            joint_trajectory_point_array[i].clear();
        }
        // 只有在空闲状态才能切换模式
        // if (current_state == ExchangeState::IDLE) 
        // {
            if (current_mode == ExchangeMode::ACTION_GROUP) 
            {   
                current_mode = ExchangeMode::CUSTOM_CONTROL;
                custom_control_data.enable = true;
                vision_control_data.enable = false;
            } 
            else if (current_mode == ExchangeMode::CUSTOM_CONTROL) 
            {
                current_mode = ExchangeMode::VISION_CONTROL;
                custom_control_data.enable = false;
                vision_control_data.enable = true;
            } 
            else 
            {
                current_mode = ExchangeMode::ACTION_GROUP;
                custom_control_data.enable = false;
                vision_control_data.enable = false;
            }
        // }
    }
    
    // X键开始/停止兑换
    if ((DR16.IsKeyPress(DR16_KEY_X)) || (VT13.IsKeyPress(VT13_KEY_X))) 
    {
        last_press_time = current_time;
        
        if (current_state == ExchangeState::IDLE) 
        {
            ExchangeEnergyUnit_startExchange();
        } 
        else if (current_state != ExchangeState::VERIFYING) 
        {
            ExchangeEnergyUnit_stopExchange();
        }
    }
}

/**
 * @brief 兑换状态机主函数
 */
void ExchangeEnergyUnit_action(Event_t **pEvent) 
{
    // 处理模式切换
    handleModeSwitch();
    
    // 更新指示灯
    updateIndicators();

    //设置兑换方式的请求
    ExchangeEnergyUnit_setParams(exchange_params);
    
    // 根据当前模式执行不同逻辑
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)) 
    {
        switch (current_mode) 
        {
            case ExchangeMode::ACTION_GROUP:
                handleActionGroupMode();
                break;
                
            case ExchangeMode::CUSTOM_CONTROL:
                handleCustomControlMode();
                break;
                
            case ExchangeMode::VISION_CONTROL:
                handleVisionControlMode();
                break;
        }
        
        // 执行关节接管
        executeJoint();
    }
    
    // 底盘控制
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        KeyBoard_MoveCtrl_Ore(toBottomdata);
    }
    if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        VT13_KeyBoard_MoveCtrl(toBottomdata);
    }

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        Keyboard_Grippers();
    }
    if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        VT13_KeyBoard_Grip();
    }
}

void ExchangeEnergyUnit_exitAction() 
{
    // 重置状态
    current_state = ExchangeState::IDLE;
    current_mode = ExchangeMode::ACTION_GROUP;
    custom_control_data.enable = false;
    vision_control_data.enable = false;
    
    custom_controller.reset();
    vision_controller.reset();
    
    // 清理轨迹
    for (uint8_t i = 0; i < 7; i++) 
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }
    
    // 重置指示灯
    WS_indicator[4] = WS_Black;
    WS_indicator[5] = WS_Black;
}

/**
 * @brief 配置接口实现
 */
void ExchangeEnergyUnit_setParams(ExchangeParams& params) 
{
    if (current_state == ExchangeState::IDLE) 
    {
        if((DR16.IsKeyPress(DR16_KEY_SHIFT) && DR16.IsKeyPress(DR16_KEY_R)) || VT13.IsKeyPress(VT13_KEY_SHIFT) && VT13.IsKeyPress(VT13_KEY_R))
        params.rotation_required = true;
        else if(DR16.IsKeyPress(DR16_KEY_CTRL) && DR16.IsKeyPress(DR16_KEY_R) || VT13.IsKeyPress(VT13_KEY_CTRL) && VT13.IsKeyPress(VT13_KEY_R))
        params.rotation_required = false;
        else if(DR16.IsKeyPress(DR16_KEY_SHIFT) && DR16.IsKeyPress(DR16_KEY_F) || VT13.IsKeyPress(VT13_KEY_SHIFT) && VT13.IsKeyPress(VT13_KEY_F))
        params.translation_required = true;
        else if(DR16.IsKeyPress(DR16_KEY_CTRL) && DR16.IsKeyPress(DR16_KEY_F) || VT13.IsKeyPress(VT13_KEY_CTRL) && VT13.IsKeyPress(VT13_KEY_F))
        params.translation_required = false;
    }
}

void ExchangeEnergyUnit_startExchange() 
{
    if (current_state == ExchangeState::IDLE && EnergyUnitCount > 0) 
    {
        // current_mode = ExchangeMode::ACTION_GROUP;
        gotoNextState(ExchangeState::INSERTING);
        // current_retry = 0;
    }
}

void ExchangeEnergyUnit_stopExchange() 
{
    if (current_state != ExchangeState::IDLE && current_state != ExchangeState::VERIFYING /*&& current_state != ExchangeState::COMPLETED && current_state != ExchangeState::FAILED*/) 
    {
        // 位于中间装配状态时停止，返回空闲状态
        // current_mode = ExchangeMode::ACTION_GROUP;
        custom_control_data.enable = false;
        vision_control_data.enable = false;
        gotoNextState(ExchangeState::IDLE);
    }
}

bool ExchangeEnergyUnit_isExchanging() 
{
    return (current_state != ExchangeState::IDLE && current_state != ExchangeState::VERIFYING /*&& current_state != ExchangeState::COMPLETED && current_state != ExchangeState::FAILED*/);
}

ExchangeState ExchangeEnergyUnit_getCurrentState() 
{
    return current_state;
}