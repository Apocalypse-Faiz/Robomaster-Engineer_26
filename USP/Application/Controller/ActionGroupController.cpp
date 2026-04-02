#include "ActionGroupController.h"
#include "ActionGroup.h"
#include "Keyboard.h"

/* 全局动作组控制器实例 */
ActionGroupController action_group_controller;

/**
 * @brief 构造函数
 */
ActionGroupController::ActionGroupController() 
{
    init();
}

/**
 * @brief 初始化控制器
 */
void ActionGroupController::init() 
{
    take_ore_state = TakeOre_enum::Zero;
    memset(ore_ing, 0, sizeof(ore_ing));
    memset(get_ore_num, 0, sizeof(get_ore_num));
    ore_index = 0;
    action_enum = ActionGroup_enum::NuLLAction;
    add_once_flag = 0;
    last_action_done = 0;
}

/**
 * @brief 处理选矿逻辑
 */
void ActionGroupController::handleOreIndex()
{
    static uint32_t last_press_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 防抖处理
    if (current_time - last_press_time < 500) return;

    // 选择矿石编号
    if (DR16.IsKeyPress(DR16_KEY_X) || VT13.IsKeyPress(VT13_KEY_X)) 
    {
        last_press_time = current_time;        
        ore_index = (ore_index + 1) % 6;
    }
    
    for (int i = 0; i < 6; i++) 
    {
        get_ore_num[i] = (i == ore_index) ? 1 : 0;
    }
}

/**
 * @brief 处理储矿逻辑
 */
void ActionGroupController::handleStoreOre(Joint joints[], Gripper& gripper, ToBottomDataStructdef& toBottomdata, ReBottomDataStructdef& reBottomdata) 
{
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        Keyboard_Grippers();
    }
    if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        VT13_KeyBoard_Grip();
    }
    // 只要按下按键，那么ore_ing的状态就会改变，不会再进入这个状态里了
    if (gripper.getState() == GripperState::OPENING)
    {
        // 检查储矿完成条件
        if (ore_ing[0] == 1) 
        { 
            // 左储存矿石中
            // toBottomdata.L_Store = 2;
            // if (reBottomdata.L_Ore == 1) 
            // { 
                // 左储存位检测到矿石
                ore_ing[0] = 2;
                // OreBit[0] = 1;
            // } else 
            // {
            //     gripper.close();
            // }
        } 
        else if (ore_ing[1] == 1) 
        { 
            // 右储存矿石中
            // toBottomdata.R_Store = 2;
            // if (reBottomdata.R_Ore == 1) 
            // { 
            //     // 右储存位检测到矿石
            //     gripper.open();
                ore_ing[1] = 2;
                // OreBit[1] = 1;
            // } 
            // else 
            // {
            //     gripper.close();
            // }
        }
    }
}

/**
 * @brief 处理取矿逻辑
 */
void ActionGroupController::handleTakeOre(Joint joints[], Gripper& gripper, ToBottomDataStructdef& toBottomdata) 
{
    // 处理不同取矿状态的夹爪控制
    if (take_ore_state >= TakeOre_enum::TakeOre1_prepare && take_ore_state <= TakeOre_enum::TakeOre6_prepare) 
    {
        // if ((DR16.IsKeyPress(DR16_KEY_E)) || VT13.IsKeyPress(VT13_KEY_E)) 
        // {
        //     gripper.close();
        // }
        if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
        {
            Keyboard_Grippers();
        }
        if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
        {
            VT13_KeyBoard_Grip();
        }
       
        if (gripper.getState() == GripperState::CLOSING)
        {
            switch (ore_index)
            {
            case 0:
                take_ore_state = TakeOre_enum::TakeOre1_ing;
                break;
            case 1:
                take_ore_state = TakeOre_enum::TakeOre2_ing;
                break;
            case 2:
                take_ore_state = TakeOre_enum::TakeOre3_ing;
                break;
            case 3:
                take_ore_state = TakeOre_enum::TakeOre4_ing;
                break;
            case 4:
                take_ore_state = TakeOre_enum::TakeOre5_ing;
                break;
            case 5:
                take_ore_state = TakeOre_enum::TakeOre6_ing;
                break;
            default:
                break;
            }
        }
        else
        {
            switch (ore_index)
            {
            case 0:
                take_ore_state = TakeOre_enum::TakeOre1_prepare;
                break;
            case 1:
                take_ore_state = TakeOre_enum::TakeOre2_prepare;
                break;
            case 2:
                take_ore_state = TakeOre_enum::TakeOre3_prepare;
                break;
            case 3:
                take_ore_state = TakeOre_enum::TakeOre4_prepare;
                break;
            case 4:
                take_ore_state = TakeOre_enum::TakeOre5_prepare;
                break;
            case 5:
                take_ore_state = TakeOre_enum::TakeOre6_prepare;
                break;
            default:
                break;
            }
        }
    }
}

/**
 * @brief 处理取肚矿逻辑
 */
void ActionGroupController::handleTakeBellyOre(Joint joints[], Gripper& gripper, ToBottomDataStructdef& toBottomdata) 
{
    if (take_ore_state == TakeOre_enum::TakeInOre_L) 
    {
        if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
        {
            Keyboard_Grippers();
        }
        if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
        {
            VT13_KeyBoard_Grip();
        }
        if (/*gripper.isObjectGripped() == true*/ gripper.getState() == GripperState::CLOSING) 
        {
            // OreBit[0] = 0;
            take_ore_state = TakeOre_enum::TakeinOreLeft_end;
        }
    } 
    else if (take_ore_state == TakeOre_enum::TakeInOre_R) 
    {
        if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
        {
            Keyboard_Grippers();
        }
        if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
        {
            VT13_KeyBoard_Grip();
        }
        if (/*gripper.isObjectGripped() == true*/ gripper.getState() == GripperState::CLOSING) 
        {
            // OreBit[1] = 0;
            take_ore_state = TakeOre_enum::TakeinOreRight_end;
        }
    }
}

/**
 * @brief 执行动作组
 */
void ActionGroupController::executeActionGroup(Joint joints[], TrajectoryPointArray trajectory_arrays[]) 
{
    // 选择动作组
    ActionGroup_Choose(trajectory_arrays, action_enum, add_once_flag);
    
    // 执行动作组
    ActionGroup_Execute(joints, trajectory_arrays, 6, add_once_flag);
    
}

/**
 * @brief 更新动作组控制
 */
void ActionGroupController::update(Joint joints[], TrajectoryPointArray trajectory_arrays[], Gripper& gripper, ToBottomDataStructdef& toBottomdata, ReBottomDataStructdef& reBottomdata) {
    // 检查动作组是否空闲
    bool action_free = true;
    for (uint8_t i = 0; i < 6; i++) 
    {
        if (joints[i].isTrajectoryPlannerFree() == false) 
        {
            action_free = false;
            break;
        }
    }
    
    // 清除取矿状态标志
    if ((DR16.GetS1() == SW_DOWN && DR16.GetS2() == SW_DOWN) || (VT13.GetFN1() && VT13.GetFN2())) 
    {
        take_ore_state = TakeOre_enum::Zero;
        OreBit[0] = 0;
        OreBit[1] = 0;
    }
    
    // 储矿逻辑
    if (action_free == true && (DR16.IsKeyPress(DR16_KEY_F) || VT13.IsKeyPress(VT13_KEY_F)) && /*gripper.isObjectGripped() == true*/ gripper.getState() == GripperState::CLOSING && take_ore_state == TakeOre_enum::Zero) 
    {
        if (/*OreBit[0] == 0 && OreBit[1] == 0*/DR16.IsKeyPress(DR16_KEY_CTRL) || VT13.IsKeyPress(VT13_KEY_CTRL)) 
        { // 左储存位空
            action_enum = ActionGroup_enum::StoreOre_L;
            add_once_flag = 1;
            ore_ing[0] = 1;
            toBottomdata.L_Store = 1;
            take_ore_state = TakeOre_enum::StoreOre_L;
        } 
        else if (/*OreBit[0] == 1 && OreBit[1] == 0*/DR16.IsKeyPress(DR16_KEY_SHIFT) || VT13.IsKeyPress(VT13_KEY_SHIFT)) 
        { // 右储存位空
            action_enum = ActionGroup_enum::StoreOre_R;
            add_once_flag = 1;
            ore_ing[1] = 1;
            toBottomdata.R_Store = 1;
            take_ore_state = TakeOre_enum::StoreOre_R;
        }
    }
    
    // 退出储矿逻辑
    else if (action_free == true && ore_ing[0] == 2) 
    {
        ore_ing[0] = 3;
        action_enum = ActionGroup_enum::ExitStoreOre_L;
        add_once_flag = 1;
        toBottomdata.L_Store = 2;
        take_ore_state = TakeOre_enum::Zero;
    } else if (action_free == true && ore_ing[1] == 2) 
    {
        ore_ing[1] = 3;
        action_enum = ActionGroup_enum::ExitStoreOre_R;
        add_once_flag = 1;
        toBottomdata.R_Store = 2;
        take_ore_state = TakeOre_enum::Zero;
    }
    
            //取矿1
            else if(get_ore_num[0] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre1_prepare;
                if (gripper.getState() != GripperState::OPENING)
                {
                    gripper.open();
                }
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre1_prepare;
                UI_Mode = 1;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre1_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::OPENING &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre1_ing;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre1_end;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre1_end)
            {
                action_enum = ActionGroup_enum::TakeOre1_end;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::Zero;
                UI_Mode = 0;
            }
            //2
            else if(get_ore_num[1] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre2_prepare;
                if (gripper.getState() != GripperState::OPENING)
                {
                    gripper.open();
                }
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre2_prepare;
                UI_Mode = 1;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre2_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre2_ing;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre2_end;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre2_end)
            {
                action_enum = ActionGroup_enum::TakeOre2_end;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::Zero;
                UI_Mode = 0;
            }
            //3
            else if(get_ore_num[2] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre3_prepare;
                if (gripper.getState() != GripperState::OPENING)
                {
                    gripper.open();
                }
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre3_prepare;
                UI_Mode = 1;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre3_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre3_ing;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre3_end;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre3_end)
            {
                action_enum = ActionGroup_enum::TakeOre3_end;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::Zero;
                UI_Mode = 0;
            }
            //4
            else if(get_ore_num[3] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre4_prepare;
                if (gripper.getState() != GripperState::OPENING)
                {
                    gripper.open();
                }
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre4_prepare;
                UI_Mode = 1;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre4_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre4_ing;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre4_end;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre4_end)
            {
                action_enum = ActionGroup_enum::TakeOre4_end;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::Zero;
                UI_Mode = 0;
            }
            //5
            else if(get_ore_num[4] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre5_prepare;
                if (gripper.getState() != GripperState::OPENING)
                {
                    gripper.open();
                }
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre5_prepare;
                UI_Mode = 1;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre5_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre5_ing;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre5_end;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre5_end)
            {
                action_enum = ActionGroup_enum::TakeOre5_end;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::Zero;
                UI_Mode = 0;
            }
            //6
            else if(get_ore_num[5] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre6_prepare;
                if (gripper.getState() != GripperState::OPENING)
                {
                    gripper.open();
                }
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre6_prepare;
                UI_Mode = 1;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre6_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ (DR16.IsKeyPress(DR16_KEY_V) || VT13.IsKeyPress(VT13_KEY_V)))
            {
                action_enum = ActionGroup_enum::TakeOre6_ing;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::TakeOre6_end;
            }
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre6_end)
            {
                action_enum = ActionGroup_enum::TakeOre6_end;
                add_once_flag = 1;
                take_ore_state = TakeOre_enum::Zero;
                UI_Mode = 0;
            }
    
    // 取肚矿逻辑
    else if (action_free == true && (DR16.IsKeyPress(DR16_KEY_C) || VT13.IsKeyPress(VT13_KEY_C)) /*&& (OreBit[0] == 1 || OreBit[1] == 1) && gripper.isObjectGripped() == false*/) 
    {
        if (/*OreBit[0] == 1 && OreBit[1] == 1*/DR16.IsKeyPress(DR16_KEY_CTRL) || VT13.IsKeyPress(VT13_KEY_CTRL)) 
        {
            if (gripper.getState() != GripperState::OPENING)
            {
                gripper.open();
            }
            action_enum = ActionGroup_enum::TakeInOre_L;
            add_once_flag = 1;
            toBottomdata.L_Store = 1;
            take_ore_state = TakeOre_enum::TakeInOre_L;
        } 
        else if (/*OreBit[0] == 0 && OreBit[1] == 1*/DR16.IsKeyPress(DR16_KEY_SHIFT) || VT13.IsKeyPress(VT13_KEY_SHIFT)) 
        {
            if (gripper.getState() != GripperState::OPENING)
            {
                gripper.open();
            }
            action_enum = ActionGroup_enum::TakeInOre_R;
            add_once_flag = 1;
            toBottomdata.R_Store = 1;
            take_ore_state = TakeOre_enum::TakeInOre_R;
        } 
        // else if (OreBit[0] == 1 && OreBit[1] == 0) 
        // {
        //     if (gripper.getState() != GripperState::OPENING)
        //     {
        //         gripper.open();
        //     }
        //     action_enum = ActionGroup_enum::TakeInOre_L;
        //     add_once_flag = 1;
        //     toBottomdata.L_Store = 1;
        //     take_ore_state = TakeOre_enum::TakeInOre_L;
        // }
    }
    else if(action_free == true && take_ore_state == TakeOre_enum::TakeinOreLeft_end)
    {
        action_enum = ActionGroup_enum::TakeinOreLeft_end;
        add_once_flag= 1;
        take_ore_state = TakeOre_enum::Zero;
    }
    else if(action_free == true && take_ore_state == TakeOre_enum::TakeinOreRight_end)
    {
        action_enum = ActionGroup_enum::TakeinOreRight_end;
        add_once_flag = 1;
        take_ore_state = TakeOre_enum::Zero;
    }
    else
    {
        action_enum = ActionGroup_enum::NuLLAction;
    }
    
    // 执行动作组
    executeActionGroup(joints, trajectory_arrays);
    
    // 处理储矿过程
    handleStoreOre(joints, gripper, toBottomdata, reBottomdata);
    
    // 处理取矿过程
    handleTakeOre(joints, gripper, toBottomdata);

    // 处理矿石编号
    handleOreIndex();
    
    // 处理取肚矿过程
    handleTakeBellyOre(joints, gripper, toBottomdata);
}

/**
 * @brief 重置控制器状态
 */
void ActionGroupController::reset() 
{
    init();
    UI_Mode = 0;
}

int ActionGroupController::isActionGroupCompleted()
{
    return !add_once_flag;
}
