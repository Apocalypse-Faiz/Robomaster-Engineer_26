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
 * @brief 仅清标志位 (不触发入场动作组)
 */
void ActionGroupController::initFlags()
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
 * @brief 初始化控制器 + 标记入场动作组待触发
 */
void ActionGroupController::init()
{
    initFlags();
    entry_action_pending = true; // 下次 update() 自动触发一次起始动作组
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
    // 检查储矿完成条件 — 4 槽 (0=LF,1=RF,2=LB,3=RB)
    if (gripper.getState() == GripperState::OPENING)
    {
        for (int s = 0; s < 4; s++)
        {
            if (ore_ing[s] == 1)
            {
                ore_ing[s] = 2;
                // OreBit[s] = 1;  // GPIO 读取自动更新, 此处注释
                break;
            }
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
    // 取肚矿逻辑 — 4 槽 (LF=左前, RF=右前, LB=左后, RB=右后)
    static const TakeOre_enum takein_states[4] = {
        TakeOre_enum::TakeInOre_LF, TakeOre_enum::TakeInOre_RF,
        TakeOre_enum::TakeInOre_LB, TakeOre_enum::TakeInOre_RB
    };
    static const TakeOre_enum end_states[4] = {
        TakeOre_enum::TakeinOreLF_end, TakeOre_enum::TakeinOreRF_end,
        TakeOre_enum::TakeinOreLB_end, TakeOre_enum::TakeinOreRB_end  // 注意: RB 枚举名无下划线
    };

    for (int s = 0; s < 4; s++)
    {
        if (take_ore_state == takein_states[s])
        {
            if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
                Keyboard_Grippers();
            if (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
                VT13_KeyBoard_Grip();

            if (gripper.getState() == GripperState::CLOSING)
            {
                // OreBit[s] = 0;
                take_ore_state = end_states[s];
            }
            return;
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
    ActionGroup_Execute(joints, trajectory_arrays, 7, add_once_flag);
    
}

/**
 * @brief 更新动作组控制
 */
void ActionGroupController::update(Joint joints[], TrajectoryPointArray trajectory_arrays[], Gripper& gripper, ToBottomDataStructdef& toBottomdata, ReBottomDataStructdef& reBottomdata) {

    // 检查动作组是否空闲
    bool action_free = true;
    for (uint8_t i = 0; i < 7; i++) 
    {
        if (joints[i].isTrajectoryPlannerFree() == false) 
        {
            action_free = false;
            break;
        }
    }

    // init() 后首次 update 自动触发一次入场起始动作组
    // 必须作为 if-else 链的第一个分支: 否则下方 else 会把 action_enum 覆盖成 NuLLAction,
    // 而 add_once_flag 仍为 1, 导致对空轨迹数组 startTrajectoryPlanner → CQC 越界 → 死锁
    if (entry_action_pending && action_free == true)
    {
        entry_action_pending = false;
        action_enum = ActionGroup_enum::ToActionGroup;
        add_once_flag = 1;
    }
    // 储矿逻辑 — 根据 OreBit 自动选空槽 (优先级: LF>RF>LB>RB)
    //   F 键触发储存, 自动寻找第一个 OreBit[s]==0 的槽位
    else if (action_free == true && (DR16.IsKeyPress(DR16_KEY_F) || VT13.IsKeyPress(VT13_KEY_F)) && gripper.getState() == GripperState::CLOSING && take_ore_state == TakeOre_enum::Zero)
    {
        static const ActionGroup_enum store_actions[4] = {
            ActionGroup_enum::StoreOre_LF, ActionGroup_enum::StoreOre_RF,
            ActionGroup_enum::StoreOre_LB, ActionGroup_enum::StoreOre_RB
        };
        static const TakeOre_enum store_states[4] = {
            TakeOre_enum::StoreOre_LF, TakeOre_enum::StoreOre_RF,
            TakeOre_enum::StoreOre_LB, TakeOre_enum::StoreOre_RB
        };
        // 按优先级 LF→RF→LB→RB 找第一个空槽
        for (int s = 0; s < 4; s++)
        {
            if (OreBit[s] == 0)
            {
                action_enum = store_actions[s];
                add_once_flag = 1;
                ore_ing[s] = 1;
                take_ore_state = store_states[s];
                break;
            }
        }
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
                // UI_Mode = 1;
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
                // UI_Mode = 0;
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
                // UI_Mode = 1;
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
                // UI_Mode = 0;
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
                // UI_Mode = 1;
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
                // UI_Mode = 0;
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
                // UI_Mode = 1;
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
                // UI_Mode = 0;
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
                // UI_Mode = 1;
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
                // UI_Mode = 0;
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
                // UI_Mode = 1;
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
                // UI_Mode = 0;
            }
    
    
    // 取肚矿逻辑 — 根据 OreBit 自动选有矿槽 (优先级: LF>RF>LB>RB)
    //   C 键触发取出, 自动寻找第一个 OreBit[s]==1 的槽位
    else if (action_free == true && (DR16.IsKeyPress(DR16_KEY_C) || VT13.IsKeyPress(VT13_KEY_C) || VT13.Datapack_custom.TakeOut) && take_ore_state == TakeOre_enum::Zero)
    {
        static const ActionGroup_enum takein_actions[4] = {
            ActionGroup_enum::TakeInOre_LF, ActionGroup_enum::TakeInOre_RF,
            ActionGroup_enum::TakeInOre_LB, ActionGroup_enum::TakeInOre_RB
        };
        static const TakeOre_enum takein_states[4] = {
            TakeOre_enum::TakeInOre_LF, TakeOre_enum::TakeInOre_RF,
            TakeOre_enum::TakeInOre_LB, TakeOre_enum::TakeInOre_RB
        };
        // 按优先级 LF→RF→LB→RB 找第一个有矿槽
        for (int s = 0; s < 4; s++)
        {
            if (OreBit[s] == 1)
            {
                if (gripper.getState() != GripperState::OPENING)
                    gripper.open();
                action_enum = takein_actions[s];
                add_once_flag = 1;
                take_ore_state = takein_states[s];
                break;
            }
        }
    }
    else
    {
        action_enum = ActionGroup_enum::NuLLAction;
    }
    
    // 退出储矿逻辑 — 4 槽
    for (int s = 0; s < 4; s++)
    {
        if (action_free == true && ore_ing[s] == 2)
        {
            static const ActionGroup_enum exit_actions[4] = {
                ActionGroup_enum::ExitStoreOre_LF, ActionGroup_enum::ExitStoreOre_RF,
                ActionGroup_enum::ExitStoreOre_LB, ActionGroup_enum::ExitStoreOre_RB
            };
            ore_ing[s] = 3;
            action_enum = exit_actions[s];
            add_once_flag = 1;
            take_ore_state = TakeOre_enum::Zero;
            break;
        }
    }
    
    // 取肚矿结束 — 4 槽
    for (int s = 0; s < 4; s++)
    {
        static const TakeOre_enum end_states[4] = {
            TakeOre_enum::TakeinOreLF_end, TakeOre_enum::TakeinOreRF_end,
            TakeOre_enum::TakeinOreLB_end, TakeOre_enum::TakeinOreRB_end
        };
        static const ActionGroup_enum end_actions[4] = {
            ActionGroup_enum::TakeinOreLF_end, ActionGroup_enum::TakeinOreRF_end,
            ActionGroup_enum::TakeinOreLB_end, ActionGroup_enum::TakeinOreRB_end
        };
        if (action_free == true && take_ore_state == end_states[s])
        {
            action_enum = end_actions[s];
            add_once_flag = 1;
            take_ore_state = TakeOre_enum::Zero;
            break;
        }
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
 * @brief 重置控制器状态 (仅清标志, 不触发入场动作组)
 */
void ActionGroupController::reset()
{
    init();
    // UI_Mode = 0;
}

int ActionGroupController::isActionGroupCompleted()
{
    return !add_once_flag;
}
