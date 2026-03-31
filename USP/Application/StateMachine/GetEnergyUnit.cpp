#include "Keyboard.h"
#include "GetEnergyUnit.h"
#include "Claw.h"
#include "CustomControl.h"
#include "ActionGroupController.h"
#include "VisualControl.h"

/* 控制模式枚举 */
enum class ControlMode {
    ACTION_GROUP,       // 动作组模式
    CUSTOM_CONTROL,     // 自定义控制模式
    VISION_CONTROL      // 视觉控制模式
};

// /**
//  * @brief 取能量单元状态枚举
//  */
// enum class TakeOre_enum
// {
//     Zero = 0,

//     TakeOre1_prepare,   // 准备
//     TakeOre2_prepare,   // 准备
//     TakeOre3_prepare,   // 准备
//     TakeOre4_prepare,   // 准备
//     TakeOre5_prepare,   // 准备
//     TakeOre6_prepare,   // 准备

//     TakeOre1_ing,   // 取矿进行
//     TakeOre2_ing,   // 取矿进行
//     TakeOre3_ing,   // 取矿进行
//     TakeOre4_ing,   // 取矿进行
//     TakeOre5_ing,   // 取矿进行
//     TakeOre6_ing,   // 取矿进行

//     TakeOre1_end,   // 取矿结束
//     TakeOre2_end,   // 取矿结束
//     TakeOre3_end,   // 取矿结束
//     TakeOre4_end,   // 取矿结束
//     TakeOre5_end,   // 取矿结束
//     TakeOre6_end,   // 取矿结束

//     StoreOre_L,     // 右储存
//     StoreOre_R,     // 左储存
//     ExitStoreOre_L, // 退出右储存
//     ExitStoreOre_R, // 退出左储存
//     TakeInOre_L,    // 取自身储存的 右
//     TakeInOre_R,    // 取自身储存的 左

//     TakeinOreRight_end,
//     TakeinOreLeft_end
// };

// static TakeOre_enum TakeOre_State = TakeOre_enum::Zero; // 用来进行一些取矿，储矿等的动作组切换状态表示
// static uint8_t Ore_ing[2] = {0, 0};                     // 进行储矿中，用于选择夹爪收缩的合适时间
// static uint8_t GetOre_Num[6] = {0, 0, 0, 0, 0, 0};
// static uint8_t index = 0;
static ControlMode current_mode = ControlMode::ACTION_GROUP;
static CustomData custom_control_data;            // 自定义控制数据
static VisionData vision_control_data;            // 视觉控制数据

// 模式切换状态
static bool mode_changing = false;
static uint32_t mode_change_time = 0;

/**
 * @brief 初始化所有控制器
 */
void initControllers() 
{
    // 初始化自定义控制数据
    custom_control_data.enable = false;
    custom_control_data.frequency = 60; // 60ms控制频率
    
    // 初始化视觉控制数据
    vision_control_data.enable = false;
    vision_control_data.frequency = 33; // 30Hz控制频率
    // vision_control_data.move_duration = 500; // 500ms移动时间
    
    // 重置所有控制器
    custom_controller.reset();
    vision_controller.reset();
    action_group_controller.reset();
    
    // 设置为当前位置
    for (int i = 0; i < 6; i++) 
    {
        if (joint[i].isCalibrated()) 
        {
            custom_control_data.joint_angle[i] = joint[i].getCurrentAngle();
            vision_control_data.joint_angle[i] = joint[i].getCurrentAngle();
        }
    }
}

/**
 * @brief 切换控制模式
 */
void switchControlMode(ControlMode new_mode) 
{
    if (current_mode == new_mode) return;

     // 停止所有关节轨迹规划
     for (uint8_t i = 0; i < 6; i++) 
     {
         joint[i].stopTrajectoryPlanner();
         joint_trajectory_point_array[i].clear();
     }
    
    // 标记正在切换模式
    mode_changing = true;
    mode_change_time = HAL_GetTick();
    
    // 退出当前模式
    switch (current_mode) 
    {
        case ControlMode::ACTION_GROUP:
            // 重置动作组控制器
            action_group_controller.reset();
            break;
            
        case ControlMode::CUSTOM_CONTROL:
            custom_control_data.enable = false;
            custom_controller.reset();
            break;
            
        case ControlMode::VISION_CONTROL:
            vision_control_data.enable = false;
            vision_controller.reset();
            break;
    }
    
    // 设置新模式
    current_mode = new_mode;
    
    // 进入新模式
    switch (new_mode) 
    {
        case ControlMode::ACTION_GROUP:
            // 动作组模式无需特殊初始化
            break;
            
        case ControlMode::CUSTOM_CONTROL:
            custom_control_data.enable = true;
            break;
            
        case ControlMode::VISION_CONTROL:
            vision_control_data.enable = true;
            break;
    }
}

/**
 * @brief 处理模式切换按键
 */
void handleModeSwitchInput() 
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 防抖处理
    if (current_time - last_press_time < 500) return;
    
    // G键切换模式：ACTION_GROUP -> CUSTOM_CONTROL -> VISION_CONTROL -> ACTION_GROUP
    if (DR16.IsKeyPress(DR16_KEY_G) || VT13.IsKeyPress(VT13_KEY_G)) 
    {
        last_press_time = current_time;
        
        ControlMode next_mode;
        switch (current_mode) 
        {
            case ControlMode::ACTION_GROUP:
                if (!action_group_controller.isActionGroupCompleted()) return;
                next_mode = ControlMode::CUSTOM_CONTROL;
                // next_mode = ControlMode::VISION_CONTROL;      // 视觉调试用
                break;
            case ControlMode::CUSTOM_CONTROL:
                next_mode = ControlMode::VISION_CONTROL;
                break;
            case ControlMode::VISION_CONTROL:
                next_mode = ControlMode::ACTION_GROUP;
                break;
        }
        
        switchControlMode(next_mode);
    }
}

/**
 * @brief 执行动作组模式
 */
void executeActionGroupMode() 
{
    // 使用动作组控制器处理所有动作组逻辑
    action_group_controller.update(joint, joint_trajectory_point_array, gripper, toBottomdata, reBottomdata);
}

/**
 * @brief 执行自定义控制模式
 */
void executeCustomControlMode() 
{
    // 检查图传连接状态
    if (VT13.GetStatus() == VT13_ESTABLISHED) 
    {
        // 接收自定义控制器数据
        for (int i = 0; i < 6; i++) 
        {
            custom_control_data.joint_angle[i] = VT13.Datapack_custom.joint_angle[i];
        }
        
        // 更新自定义控制器
        custom_controller.update(custom_control_data, joint, joint_trajectory_point_array, 6);
    }
    
    // 在自定义控制模式下，夹爪仍然可以独立控制
    // Keyboard_Grippers();
    // Keyboard_StoreGrippers(toBottomdata);
    VT13_KeyBoard_Grip();
    VT13_KeyboardStoreGrippers(toBottomdata);
}

/**
 * @brief 执行视觉控制模式
 */
void executeVisionControlMode() 
{
    // 检查是否收到小电脑数据
    if (ulTaskNotifyTake(pdTRUE, 0) == 55)
    {
        for (int i = 0; i < 6; i++)
        {
            vision_control_data.joint_angle[i] = reVisiondata.joint_target[i];
        }

        // 更新视觉控制
        vision_controller.update(vision_control_data, joint, joint_trajectory_point_array, 6);
    }

    // 视觉模式下也可以控制夹爪
    // Keyboard_Grippers();
    // Keyboard_StoreGrippers(toBottomdata);

    // VT13_KeyBoard_Grip();
    // VT13_KeyboardStoreGrippers(toBottomdata);
}

/**
 * @brief 执行关节控制接管
 */
void executeJointTakeover() 
{
    uint32_t current_time = HAL_GetTick();
    
    // 所有模式都需要执行关节接管
    for (uint8_t i = 0; i < 6; i++) 
    {
        joint[i].takeOver(current_time);
    }
}

void GetEnergyUnit_enterAction()
{
    for (uint8_t i = 0; i < 6; i++)
    {
        joint_trajectory_point_array[i].clear();
        joint[i].stopTrajectoryPlanner();
    }

    // 初始化所有控制器
    initControllers();
    
    // 设置为动作组模式
    current_mode = ControlMode::ACTION_GROUP;
    mode_changing = false;

}

void GetEnergyUnit_action(Event_t **pEvent)
{
    // 根据当前模式执行不同逻辑
    if ((DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED) || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)) 
    {
        switch (current_mode) 
        {
            case ControlMode::ACTION_GROUP:
                executeActionGroupMode();

                break;
                
            case ControlMode::CUSTOM_CONTROL:
                executeCustomControlMode();
                executeJointTakeover();
                break;
                
            case ControlMode::VISION_CONTROL:
                executeVisionControlMode();
                executeJointTakeover();
                break;
        }
        
    }

    switch (current_mode)
    {
        case ControlMode::ACTION_GROUP :
            if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                KeyBoard_MoveCtrl_Ore(toBottomdata);
            }
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                VT13_KeyBoard_MoveCtrl(toBottomdata);
            }
            break;
        case ControlMode::CUSTOM_CONTROL :
            if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                KeyBoard_MoveCtrl_Custom(toBottomdata);
            }
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                VT13_KeyBoard_MoveCtrl_Custom(toBottomdata);
            }
        case ControlMode::VISION_CONTROL :
            if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                KeyBoard_MoveCtrl_Ore(toBottomdata);
            }
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
            {
                VT13_KeyBoard_MoveCtrl(toBottomdata);
            }
            break;        
        default:
            break;
    }

    handleModeSwitchInput(); 

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        Keyboard_Grippers();
        Keyboard_StoreGrippers(toBottomdata);
    }
    if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        VT13_KeyBoard_Grip();
        VT13_KeyboardStoreGrippers(toBottomdata);
    }

    // static ActionGroup_enum action_enum = ActionGroup_enum::NuLLAction; // 动作组标志位
    // static uint8_t Add_onlyOnce = 0;                                    // 按键保留按下的时间会经过几个周期，因此需要额外的单独的标志位确保动作组只被添加一次
    // static uint8_t last_actionDone = 0;                                 // 用于判断上次动作组是否完成了
    // bool ActionFree = true;                                             // 动作组是否空闲，用于判断是否可以执行动作组

    // WS_indicator[0] = WS_Green;

    // if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    // {
    //     /* 模式切换控制 */
    //     if (DR16.IsKeyPress(DR16_KEY_G)) 
    //     {
    //         // 按下G键切换模式
    //         if (current_mode == GetMode::ACTION_GROUP) 
    //         {
    //             current_mode = GetMode::CUSTOM_CONTROL;
    //             custom_control_data.enable = true;
    //             TakeOre_State = TakeOre_enum::Zero;  // 清除动作组状态
                
    //         } else 
    //         {
    //             current_mode = GetMode::ACTION_GROUP;
    //             custom_control_data.enable = false;
    //             custom_controller.reset();
    
    //         }
    //     }

    //      /*判断动作组是否执行完毕*/
    //     for (uint8_t i = 0; i < 6; i++)
    //     {
    //         if (joint[i].isTrajectoryPlannerFree() == false)
    //         {
    //             ActionFree = false;
    //         }
    //     }

    //     if ((DR16.GetS1() == SW_DOWN && DR16.GetS2() == SW_DOWN)) // 清除取矿状态标志
    //     {
    //         TakeOre_State = TakeOre_enum::Zero;
    //         OreBit[0] = 0;
    //         OreBit[1] = 0;
    //     }

    //     if(custom_control_data.enable == 1)
    //     {
    //         WS_indicator[3] = WS_Blue;
    //     }
    //     else
    //     {
    //         WS_indicator[3] = WS_Black;
    //     }

    //     /* 根据当前模式执行不同逻辑 */
    //     if (current_mode == GetMode::ACTION_GROUP) 
    //     {
    //         // ==================== 动作组模式逻辑 ====================


    //         if(DR16.IsKeyPress(DR16_KEY_X))
    //         {
    //             index++;
    //             if(index > 5)
    //             index = 0;
    //         }
    //         for(int i = 0; i < 6; i++)
    //         {
    //             if(i == index) GetOre_Num[i] = 1;
    //             else GetOre_Num[i] = 0;
    //         }

    //         //储矿
    //         if(ActionFree == true && DR16.IsKeyPress(DR16_KEY_F) && gripper.isObjectGripped() == true && TakeOre_State == TakeOre_enum::Zero)
    //         {
    //             if(OreBit[0] == 0 && OreBit[1] == 0) // 左储存位空
    //             {
    //                 TakeOre_State = TakeOre_enum::StoreOre_L;
    //                 Add_onlyOnce = 1;
    //                 Ore_ing[0] = 1;
    //                 toBottomdata.L_gripper = 1;
    //             }
    //             else if(OreBit[0] == 1 && OreBit[1] == 0) // 右储存位空
    //             {
    //                 TakeOre_State = TakeOre_enum::StoreOre_R;
    //                 Add_onlyOnce = 1;
    //                 Ore_ing[1] = 1;
    //                 toBottomdata.R_gripper = 1;
    //             }
    //             else if(OreBit[0] == 1 && OreBit[1] == 1) // 都有矿
    //             {
    //                 // 不执行任何操作
    //                 action_enum = ActionGroup_enum::NuLLAction;
    //             }
    //         }
    //         //退出储矿
    //         else if(ActionFree == true && Ore_ing[0] == 2)
    //         {
    //                 Ore_ing[0] = 3;
    //                 TakeOre_State = TakeOre_enum::ExitStoreOre_L;
    //                 Add_onlyOnce = 1;
    //                 TakeOre_State = TakeOre_enum::Zero;
    //         }
    //         else if(ActionFree == true && Ore_ing[1] == 2)
    //         {
    //                 Ore_ing[1] = 3;
    //                 TakeOre_State = TakeOre_enum::ExitStoreOre_R;
    //                 Add_onlyOnce = 1;
    //                 TakeOre_State = TakeOre_enum::Zero;
    //         }
    //         //取矿1
    //         else if(GetOre_Num[0] == 1 && ActionFree == true && TakeOre_State == TakeOre_enum::Zero && gripper.isObjectGripped() == false && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre1_prepare;
    //             gripper.open(1.0f);
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre1_prepare;
    //             UI_Mode = 1;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre1_prepare && gripper.isObjectGripped() == true && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre1_ing;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre1_end;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre1_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeOre1_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //             UI_Mode = 0;
    //         }
    //         //2
    //         else if(GetOre_Num[1] == 1 && ActionFree == true && TakeOre_State == TakeOre_enum::Zero && gripper.isObjectGripped() == false && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre2_prepare;
    //             gripper.open(1.0f);
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre2_prepare;
    //             UI_Mode = 1;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre2_prepare && gripper.isObjectGripped() == true && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre2_ing;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre2_end;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre2_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeOre2_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //             UI_Mode = 0;
    //         }
    //         //3
    //         else if(GetOre_Num[2] == 1 && ActionFree == true && TakeOre_State == TakeOre_enum::Zero && gripper.isObjectGripped() == false && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre3_prepare;
    //             gripper.open(1.0f);
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre3_prepare;
    //             UI_Mode = 1;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre3_prepare && gripper.isObjectGripped() == true && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre3_ing;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre3_end;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre3_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeOre3_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //             UI_Mode = 0;
    //         }
    //         //4
    //         else if(GetOre_Num[3] == 1 && ActionFree == true && TakeOre_State == TakeOre_enum::Zero && gripper.isObjectGripped() == false && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre4_prepare;
    //             gripper.open(1.0f);
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre4_prepare;
    //             UI_Mode = 1;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre4_prepare && gripper.isObjectGripped() == true && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre4_ing;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre4_end;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre4_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeOre4_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //             UI_Mode = 0;
    //         }
    //         //5
    //         else if(GetOre_Num[4] == 1 && ActionFree == true && TakeOre_State == TakeOre_enum::Zero && gripper.isObjectGripped() == false && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre5_prepare;
    //             gripper.open(1.0f);
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre5_prepare;
    //             UI_Mode = 1;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre5_prepare && gripper.isObjectGripped() == true && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre5_ing;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre5_end;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre5_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeOre5_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //             UI_Mode = 0;
    //         }
    //         //6
    //         else if(GetOre_Num[5] == 1 && ActionFree == true && TakeOre_State == TakeOre_enum::Zero && gripper.isObjectGripped() == false && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre6_prepare;
    //             gripper.open(1.0f);
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre6_prepare;
    //             UI_Mode = 1;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre6_prepare && gripper.isObjectGripped() == true && DR16.IsKeyPress(DR16_KEY_V))
    //         {
    //             action_enum = ActionGroup_enum::TakeOre6_ing;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::TakeOre6_end;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeOre6_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeOre6_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //             UI_Mode = 0;
    //         }
    //         //取肚矿
    //         else if(ActionFree == true && DR16.IsKeyPress(DR16_KEY_C) && (OreBit[0] == 1 || OreBit[1] == 1) && gripper.isObjectGripped() == false)
    //         {
    //             if(OreBit[0] == 1 && OreBit[1] == 1)
    //             {
    //                 action_enum = ActionGroup_enum::TakeInOre_L;
    //                 Add_onlyOnce = 1;
    //                 TakeOre_State = TakeOre_enum::TakeInOre_L;
    //                 gripper.open(1.0f);
    //             }
    //             else if(OreBit[0] == 0 && OreBit[1] == 1)
    //             {
    //                 action_enum = ActionGroup_enum::TakeInOre_R;
    //                 Add_onlyOnce = 1;
    //                 TakeOre_State = TakeOre_enum::TakeInOre_R;
    //                 gripper.open(1.0f);
    //             }
    //             else if(OreBit[0] == 1 && OreBit[1] == 0)
    //             {
    //                 action_enum = ActionGroup_enum::TakeInOre_L;
    //                 Add_onlyOnce = 1;
    //                 TakeOre_State = TakeOre_enum::TakeInOre_L;
    //                 gripper.open(1.0f);
    //             }
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeinOreLeft_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeinOreLeft_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //         }
    //         else if(ActionFree == true && TakeOre_State == TakeOre_enum::TakeinOreRight_end)
    //         {
    //             action_enum = ActionGroup_enum::TakeinOreRight_end;
    //             Add_onlyOnce = 1;
    //             TakeOre_State = TakeOre_enum::Zero;
    //         }
    //         else
    //         {
    //             action_enum = ActionGroup_enum::NuLLAction;
    //         }
            
    //         if (ActionFree == true)
    //         {
    //             last_actionDone = 1;
    //         }
    //         else
    //         {
    //             last_actionDone = 0;
    //         }

    //         /*选择动作组*/
    //         ActionGroup_Choose(joint_trajectory_point_array, action_enum, Add_onlyOnce);

    //         /* 动作组执行 */
    //         ActionGroup_Execute(joint, joint_trajectory_point_array, 6, Add_onlyOnce);

    //         //存取矿过程
    //         if(Ore_ing[0] == 1) // 左储存矿石中
    //         {
    //             toBottomdata.L_gripper = 2;
    //             if(reBottomdata.L_Ore == 1) // 左储存位检测到矿石
    //             {
    //                 gripper.open(1.0f); // 夹爪打开
    //                 Ore_ing[0] = 2;
    //                 OreBit[0] = 1; // 更新左储存位状态为有矿
    //             }
    //             else
    //             {
    //                 gripper.close(1.0f); // 夹爪夹紧
    //             }
                
    //         }
    //         else if(Ore_ing[1] == 1) // 右储存矿石中
    //         {
    //             toBottomdata.R_gripper = 2;
    //             if(reBottomdata.R_Ore == 1) // 右储存位检测到矿石
    //             {
    //                 gripper.open(1.0f); // 夹爪打开
    //                 Ore_ing[1] = 2;
    //                 OreBit[1] = 1; // 更新右储存位状态为有矿
    //             }
    //             else
    //             {
    //                 gripper.close(1.0f); // 夹爪夹紧
    //             }
    //         }
    //         //取矿过程  1
    //         if(TakeOre_State == TakeOre_enum::TakeOre1_prepare)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             TakeOre_State = TakeOre_enum::TakeOre1_ing;
    //         }
    //         //2
    //         else if(TakeOre_State == TakeOre_enum::TakeOre2_prepare)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             TakeOre_State = TakeOre_enum::TakeOre2_ing;
    //         }
    //         //3
    //         else if(TakeOre_State == TakeOre_enum::TakeOre3_prepare)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             TakeOre_State = TakeOre_enum::TakeOre3_ing;
    //         }
    //         //4
    //         else if(TakeOre_State == TakeOre_enum::TakeOre4_prepare)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             TakeOre_State = TakeOre_enum::TakeOre4_ing;
    //         }
    //         //5
    //         else if(TakeOre_State == TakeOre_enum::TakeOre5_prepare)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             TakeOre_State = TakeOre_enum::TakeOre5_ing;
    //         }
    //         //6
    //         else if(TakeOre_State == TakeOre_enum::TakeOre6_prepare)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             TakeOre_State = TakeOre_enum::TakeOre6_ing;
    //         }
    //         //取肚矿
    //         if(TakeOre_State == TakeOre_enum::TakeInOre_L)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             if(gripper.isObjectGripped() == true)
    //             {
    //                 OreBit[0] = 0;
    //                 toBottomdata.L_gripper = 1;
    //                 TakeOre_State = TakeOre_enum::TakeinOreLeft_end;
    //             }
    //         }
    //         else if(TakeOre_State == TakeOre_enum::TakeInOre_R)
    //         {
    //             if(DR16.IsKeyPress(DR16_KEY_E))
    //             {
    //                 gripper.close(1.0f);
    //             }
    //             if(gripper.isObjectGripped() == true)
    //             {
    //                 OreBit[1] = 1;
    //                 toBottomdata.R_gripper = 1;
    //                 TakeOre_State = TakeOre_enum::TakeinOreRight_end;
    //             }
    //         }
    //     }
    //     else if (current_mode == GetMode::CUSTOM_CONTROL) 
    //     {
    //         // ==================== 自定义控制模式逻辑 ====================
    //         if(VT13.GetStatus() == VT13_ESTABLISHED)
    //         {
    //             for(int i = 0; i < 6; i++)
    //             {
    //                 custom_control_data.joint_angle[i] = VT13.Datapack_custom.joint_angle[i];
    //             }

    //             custom_controller.update(custom_control_data, joint, joint_trajectory_point_array, 6);
    //             Keyboard_Grippers();
    //             Keyboard_StoreGrippers(toBottomdata);

    //         }
            
    //     }
    // }
    // else
    // {

    // }

    // KeyBoard_MoveCtrl_Ore(toBottomdata);
    // DR16_Grip();
    // DR16_StoreGrippers(toBottomdata);

}

void GetEnergyUnit_exitAction()
{
    // TakeOre_State = TakeOre_enum::Zero;
    // UI_Mode = 0;
    // Ore_ing[0] = 0;
    // Ore_ing[1] = 0;
    // for(int i = 0; i < 6; i++)
    // {
    //     GetOre_Num[i] = 0;
    // }
    // index = 0;

     // 重置所有控制器
     custom_control_data.enable = false;
     vision_control_data.enable = false;
     
     custom_controller.reset();
     vision_controller.reset();
     action_group_controller.reset();
     
     // 重置模式状态
     current_mode = ControlMode::ACTION_GROUP;
     mode_changing = false;
}
