#include "ActionGroupController.h"
#include "ActionGroup.h"
#include "Keyboard.h"

namespace {

constexpr uint8_t AUTO_FOUR_ORE_COUNT = 4U;
constexpr uint32_t AUTO_GRIPPER_OPEN_TIME_MS = 0U;
constexpr uint32_t AUTO_GRIPPER_CLOSE_TIME_MS = 300U;
constexpr uint32_t AUTO_ORE_DROP_SETTLE_TIME_MS = 0U;
constexpr uint32_t AUTO_ACTION_TIMEOUT_MS = 5000U;

// 数组使用零基索引，因此 {1,2,5,4} 对应实际的 2、3、6、5 号矿。
const uint8_t auto_ore_sequence[AUTO_FOUR_ORE_COUNT] = {1U, 2U, 5U, 4U};

// 储矿顺序：左前、左后、右前、右后。
const uint8_t auto_slot_sequence[AUTO_FOUR_ORE_COUNT] = {0U, 2U, 1U, 3U};

const ActionGroup_enum auto_take_prepare_actions[6] = {
    ActionGroup_enum::TakeOre1_prepare, ActionGroup_enum::TakeOre2_prepare,
    ActionGroup_enum::TakeOre3_prepare, ActionGroup_enum::TakeOre4_prepare,
    ActionGroup_enum::TakeOre5_prepare, ActionGroup_enum::TakeOre6_prepare
};

const ActionGroup_enum auto_take_store_actions[AUTO_FOUR_ORE_COUNT] = {
    ActionGroup_enum::AutoTakeStoreOre2_LF,
    ActionGroup_enum::AutoTakeStoreOre3_LB,
    ActionGroup_enum::AutoTakeStoreOre6_RF,
    ActionGroup_enum::AutoTakeStoreOre5_RB
};

const ActionGroup_enum auto_exit_store_actions[4] = {
    ActionGroup_enum::ExitStoreOre_LF, ActionGroup_enum::ExitStoreOre_RF,
    ActionGroup_enum::ExitStoreOre_LB, ActionGroup_enum::ExitStoreOre_RB
};

const ActionGroup_enum auto_exit_to_next_prepare_actions[AUTO_FOUR_ORE_COUNT - 1U] = {
    ActionGroup_enum::AutoExitLFToOre3Prepare,
    ActionGroup_enum::AutoExitLBToOre6Prepare,
    ActionGroup_enum::AutoExitRFToOre5Prepare
};

} // namespace

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
    takein_phase = 0U;
    manual_gripper_action_tick = 0U;
    manual_ore_auto_active = false;
    last_manual_store_pressed = false;
    last_manual_takeout_pressed = false;
    manual_slot_mode = false;
    manual_slot_index = 0;
    last_mouse_left_pressed = false;
    last_mouse_right_pressed = false;
    action_enum = ActionGroup_enum::NuLLAction;
    add_once_flag = 0;
    last_action_done = 0;

    auto_four_ore_state = AutoFourOreState::Idle;
    auto_four_ore_error = AutoFourOreError::None;
    auto_four_ore_count = 0U;
    auto_four_current_ore = auto_ore_sequence[0];
    auto_four_current_slot = 0U;
    auto_state_enter_tick = 0U;
    auto_four_ore_active = false;
    last_auto_start_pressed = false;
    last_auto_abort_pressed = false;
}

void ActionGroupController::startActionGroup(ActionGroup_enum action)
{
    action_enum = action;
    add_once_flag = 1U;
    last_action_done = 0U;
    auto_state_enter_tick = HAL_GetTick();
}

void ActionGroupController::stopAutoFourOre(AutoFourOreError error)
{
    auto_four_ore_active = false;
    auto_four_ore_error = error;
    auto_four_ore_state = (error == AutoFourOreError::UserAborted)
        ? AutoFourOreState::Aborted
        : AutoFourOreState::Fault;
    take_ore_state = TakeOre_enum::Zero;
    action_enum = ActionGroup_enum::NuLLAction;
    add_once_flag = 0U;
    last_action_done = 0U;
}

/**
 * @brief 一键四矿状态机。
 *
 * Shift+V 上升沿启动；Shift+C 立即停止当前轨迹并执行 ToActionGroup，
 * 回归完成后退出。PWM 夹爪没有反馈，因此开合完成使用固定等待时间判断。
 */
bool ActionGroupController::handleAutoFourOre(
    Joint joints[], TrajectoryPointArray trajectory_arrays[],
    Gripper& gripper, bool action_free)
{
    const bool dr16_online =
        DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED;
    const bool vt13_online =
        VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED &&
        VT13.GetModeSwitch() != MS_Status_Typedef::MS_C;

    const bool start_pressed =
        (dr16_online && DR16.IsKeyPress(DR16_KEY_SHIFT) && DR16.IsKeyPress(DR16_KEY_V)) ||
        (vt13_online && VT13.IsKeyPress(VT13_KEY_SHIFT) && VT13.IsKeyPress(VT13_KEY_V));
    const bool abort_pressed =
        (dr16_online && DR16.IsKeyPress(DR16_KEY_SHIFT) && DR16.IsKeyPress(DR16_KEY_C)) ||
        (vt13_online && VT13.IsKeyPress(VT13_KEY_SHIFT) && VT13.IsKeyPress(VT13_KEY_C));

    const bool start_rising = start_pressed && !last_auto_start_pressed;
    const bool abort_rising = abort_pressed && !last_auto_abort_pressed;
    last_auto_start_pressed = start_pressed;
    last_auto_abort_pressed = abort_pressed;

    if (auto_four_ore_active && abort_rising &&
        auto_four_ore_state != AutoFourOreState::WaitAbortReturn)
    {
        // 硬中止当前一键四矿轨迹，并清除尚未执行的旧目标点。
        for (uint8_t i = 0U; i < 7U; ++i)
        {
            joints[i].stopTrajectoryPlanner();
            trajectory_arrays[i].clear();
        }

        auto_four_ore_error = AutoFourOreError::UserAborted;
        take_ore_state = TakeOre_enum::Zero;
        startActionGroup(ActionGroup_enum::ToActionGroup);
        auto_four_ore_state = AutoFourOreState::WaitAbortReturn;
        return true;
    }

    if (!auto_four_ore_active && start_rising)
    {
        auto_four_ore_count = 0U;
        auto_four_current_slot = 0U;
        auto_four_current_ore = auto_ore_sequence[0];
        ore_index = auto_four_current_ore;
        auto_four_ore_error = AutoFourOreError::None;

        if (!action_free || take_ore_state != TakeOre_enum::Zero ||
            add_once_flag != 0U || manual_ore_auto_active)
        {
            stopAutoFourOre(AutoFourOreError::Busy);
            return true;
        }

        for (uint8_t i = 0U; i < 7U; ++i)
        {
            if (!joints[i].isCalibrated())
            {
                stopAutoFourOre(AutoFourOreError::ArmNotCalibrated);
                return true;
            }
        }

        for (uint8_t slot = 0U; slot < 4U; ++slot)
        {
            if (OreBit[slot] != 0U)
            {
                stopAutoFourOre(AutoFourOreError::StorageNotEmpty);
                return true;
            }
        }

        if (gripper.getState() == GripperState::ERROR)
        {
            stopAutoFourOre(AutoFourOreError::GripperError);
            return true;
        }

        auto_four_ore_active = true;
        auto_four_ore_state = AutoFourOreState::OpenGripper;
        auto_state_enter_tick = HAL_GetTick();
        take_ore_state = TakeOre_enum::Zero;
        last_action_done = 0U;
    }

    // Shift 组合键按住期间必须吞掉普通 V/C，避免同时触发原有手动逻辑。
    if (!auto_four_ore_active)
    {
        return start_pressed || abort_pressed;
    }

    const uint32_t now = HAL_GetTick();

    if (gripper.getState() == GripperState::ERROR)
    {
        stopAutoFourOre(AutoFourOreError::GripperError);
        return true;
    }

    switch (auto_four_ore_state)
    {
        case AutoFourOreState::OpenGripper:
            gripper.open();
            auto_state_enter_tick = now;
            auto_four_ore_state = AutoFourOreState::WaitOpenGripper;
            break;

        case AutoFourOreState::WaitOpenGripper:
            if (gripper.getState() != GripperState::OPENING)
            {
                stopAutoFourOre(AutoFourOreError::GripperCommandOverridden);
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_GRIPPER_OPEN_TIME_MS)
            {
                startActionGroup(auto_take_prepare_actions[auto_four_current_ore]);
                auto_four_ore_state = AutoFourOreState::WaitTakePrepare;
            }
            break;

        case AutoFourOreState::WaitTakePrepare:
            if (action_free)
            {
                gripper.close();
                auto_state_enter_tick = now;
                auto_four_ore_state = AutoFourOreState::WaitCloseGripper;
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_ACTION_TIMEOUT_MS)
            {
                stopAutoFourOre(AutoFourOreError::ActionTimeout);
            }
            break;

        case AutoFourOreState::WaitCloseGripper:
            if (gripper.getState() != GripperState::CLOSING)
            {
                stopAutoFourOre(AutoFourOreError::GripperCommandOverridden);
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_GRIPPER_CLOSE_TIME_MS)
            {
                auto_four_current_slot = auto_slot_sequence[auto_four_ore_count];
                if (OreBit[auto_four_current_slot] != 0U)
                {
                    stopAutoFourOre(AutoFourOreError::StorageNotEmpty);
                    break;
                }

                startActionGroup(auto_take_store_actions[auto_four_ore_count]);
                auto_four_ore_state = AutoFourOreState::WaitStore;
            }
            break;

        case AutoFourOreState::WaitStore:
            if (action_free)
            {
                gripper.open();
                auto_state_enter_tick = now;
                auto_four_ore_state = AutoFourOreState::WaitRelease;
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_ACTION_TIMEOUT_MS)
            {
                stopAutoFourOre(AutoFourOreError::ActionTimeout);
            }
            break;

        case AutoFourOreState::WaitRelease:
            if (gripper.getState() != GripperState::OPENING)
            {
                stopAutoFourOre(AutoFourOreError::GripperCommandOverridden);
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_ORE_DROP_SETTLE_TIME_MS)
            {
                const bool has_next_ore =
                    (auto_four_ore_count + 1U) < AUTO_FOUR_ORE_COUNT;
                if (has_next_ore)
                {
                    startActionGroup(
                        auto_exit_to_next_prepare_actions[auto_four_ore_count]);
                    auto_four_ore_state = AutoFourOreState::WaitExitToNextPrepare;
                }
                else
                {
                    startActionGroup(auto_exit_store_actions[auto_four_current_slot]);
                    auto_four_ore_state = AutoFourOreState::WaitExitStore;
                }
            }
            break;

        case AutoFourOreState::WaitExitToNextPrepare:
            if (action_free)
            {
                ++auto_four_ore_count;
                auto_four_current_ore = auto_ore_sequence[auto_four_ore_count];
                ore_index = auto_four_current_ore;
                gripper.close();
                auto_state_enter_tick = now;
                auto_four_ore_state = AutoFourOreState::WaitCloseGripper;
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_ACTION_TIMEOUT_MS)
            {
                stopAutoFourOre(AutoFourOreError::ActionTimeout);
            }
            break;

        case AutoFourOreState::WaitExitStore:
            if (action_free)
            {
                ++auto_four_ore_count;
                if (auto_four_ore_count >= AUTO_FOUR_ORE_COUNT)
                {
                    auto_four_ore_active = false;
                    auto_four_ore_state = AutoFourOreState::Completed;
                    auto_four_ore_error = AutoFourOreError::None;
                    take_ore_state = TakeOre_enum::Zero;
                    action_enum = ActionGroup_enum::NuLLAction;
                }
                else
                {
                    auto_four_current_ore = auto_ore_sequence[auto_four_ore_count];
                    ore_index = auto_four_current_ore;
                    auto_four_ore_state = AutoFourOreState::OpenGripper;
                    auto_state_enter_tick = now;
                }
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_ACTION_TIMEOUT_MS)
            {
                stopAutoFourOre(AutoFourOreError::ActionTimeout);
            }
            break;

        case AutoFourOreState::WaitAbortReturn:
            if (action_free && add_once_flag == 0U)
            {
                stopAutoFourOre(AutoFourOreError::UserAborted);
            }
            else if ((uint32_t)(now - auto_state_enter_tick) >= AUTO_ACTION_TIMEOUT_MS)
            {
                stopAutoFourOre(AutoFourOreError::ActionTimeout);
            }
            break;

        case AutoFourOreState::Idle:
        case AutoFourOreState::Completed:
        case AutoFourOreState::Aborted:
        case AutoFourOreState::Fault:
        default:
            break;
    }

    return true;
}

/**
 * @brief Handle VT13 mouse-controlled belly-slot selection.
 *
 * Left-click toggles between OreBit automatic selection and manual selection.
 * While manual selection is active, each right-click advances LF->RF->LB->RB.
 */
void ActionGroupController::handleBellySlotSelection()
{
    const bool vt13_online =
        VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED;
    const bool mouse_left_pressed =
        vt13_online && (VT13.GetPress_L() != 0U);
    const bool mouse_right_pressed =
        vt13_online && (VT13.GetPress_R() != 0U);

    const bool mouse_left_rising =
        mouse_left_pressed && !last_mouse_left_pressed;
    const bool mouse_right_rising =
        mouse_right_pressed && !last_mouse_right_pressed;

    last_mouse_left_pressed = mouse_left_pressed;
    last_mouse_right_pressed = mouse_right_pressed;

    if (mouse_left_rising)
    {
        manual_slot_mode = !manual_slot_mode;
    }

    if (manual_slot_mode && mouse_right_rising)
    {
        manual_slot_index = (manual_slot_index + 1U) % 4U;
    }
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
    static const TakeOre_enum store_states[4] = {
        TakeOre_enum::StoreOre_LF, TakeOre_enum::StoreOre_RF,
        TakeOre_enum::StoreOre_LB, TakeOre_enum::StoreOre_RB
    };

    bool action_free = true;
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        if (!joints[i].isTrajectoryPlannerFree())
        {
            action_free = false;
            break;
        }
    }

    // StoreOre_X 到位后自动松爪；等待矿石释放后再允许 ExitStoreOre_X。
    for (int s = 0; s < 4; ++s)
    {
        if (take_ore_state != store_states[s])
        {
            continue;
        }

        if (ore_ing[s] == 1U && action_free)
        {
            gripper.open();
            manual_gripper_action_tick = HAL_GetTick();
            ore_ing[s] = 2U;
        }
        else if (ore_ing[s] == 2U)
        {
            if (gripper.getState() != GripperState::OPENING)
            {
                // 自动流程拥有夹爪控制权；若命令被覆盖则重新发出并重新计时。
                gripper.open();
                manual_gripper_action_tick = HAL_GetTick();
            }
            else if ((uint32_t)(HAL_GetTick() - manual_gripper_action_tick) >=
                     AUTO_ORE_DROP_SETTLE_TIME_MS)
            {
                ore_ing[s] = 3U;
            }
        }
        return;
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
            bool action_free = true;
            for (uint8_t i = 0U; i < 7U; ++i)
            {
                if (!joints[i].isTrajectoryPlannerFree())
                {
                    action_free = false;
                    break;
                }
            }

            // TakeInOre_X 到位后自动夹紧，稳定后进入对应的取肚矿收尾动作。
            if (takein_phase == 1U && action_free)
            {
                gripper.close();
                manual_gripper_action_tick = HAL_GetTick();
                takein_phase = 2U;
            }
            else if (takein_phase == 2U)
            {
                if (gripper.getState() != GripperState::CLOSING)
                {
                    // 自动流程拥有夹爪控制权；若命令被覆盖则重新发出并重新计时。
                    gripper.close();
                    manual_gripper_action_tick = HAL_GetTick();
                }
                else if ((uint32_t)(HAL_GetTick() - manual_gripper_action_tick) >=
                         AUTO_GRIPPER_CLOSE_TIME_MS)
                {
                    takein_phase = 0U;
                    take_ore_state = end_states[s];
                }
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

    // 自动流程运行期间不响应手动槽位选择。
    if (!auto_four_ore_active)
    {
        handleBellySlotSelection();
    }

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

    // 手动 V/C 只在没有 Shift/Ctrl 修饰键时有效，避免组合键分两帧到达时
    // 误触发单次取矿或取肚矿。WASD 等其他按键仍可与手动动作同时使用。
    const bool manual_take_key_pressed =
        (DR16.IsKeyPress(DR16_KEY_V) &&
         !DR16.IsKeyPress(DR16_KEY_SHIFT) &&
         !DR16.IsKeyPress(DR16_KEY_CTRL)) ||
        (VT13.IsKeyPress(VT13_KEY_V) &&
         !VT13.IsKeyPress(VT13_KEY_SHIFT) &&
         !VT13.IsKeyPress(VT13_KEY_CTRL));
    const bool manual_store_key_pressed =
        (DR16.IsKeyPress(DR16_KEY_F) &&
         !DR16.IsKeyPress(DR16_KEY_SHIFT) &&
         !DR16.IsKeyPress(DR16_KEY_CTRL)) ||
        (VT13.IsKeyPress(VT13_KEY_F) &&
         !VT13.IsKeyPress(VT13_KEY_SHIFT) &&
         !VT13.IsKeyPress(VT13_KEY_CTRL));
    const bool manual_takeout_key_pressed =
        (DR16.IsKeyPress(DR16_KEY_C) &&
         !DR16.IsKeyPress(DR16_KEY_SHIFT) &&
         !DR16.IsKeyPress(DR16_KEY_CTRL)) ||
        (VT13.IsKeyPress(VT13_KEY_C) &&
         !VT13.IsKeyPress(VT13_KEY_SHIFT) &&
         !VT13.IsKeyPress(VT13_KEY_CTRL));
    const bool manual_takeout_command_pressed =
        manual_takeout_key_pressed || VT13.Datapack_custom.TakeOut;
    const bool manual_store_start =
        manual_store_key_pressed && !last_manual_store_pressed;
    const bool manual_takeout_start =
        manual_takeout_command_pressed && !last_manual_takeout_pressed;
    last_manual_store_pressed = manual_store_key_pressed;
    last_manual_takeout_pressed = manual_takeout_command_pressed;

    bool auto_owns_cycle = false;

    // 入场动作完成前不接受其他动作，避免空轨迹或动作组互相覆盖。
    if (entry_action_pending)
    {
        auto_owns_cycle = true;
        if (action_free)
        {
            entry_action_pending = false;
            startActionGroup(ActionGroup_enum::ToActionGroup);
        }
    }
    else
    {
        auto_owns_cycle =
            handleAutoFourOre(joints, trajectory_arrays, gripper, action_free);
    }

    if (!auto_owns_cycle)
    {
    // 储矿逻辑 — 根据 OreBit 自动选空槽 (优先级: LF>RF>LB>RB)
    //   F 键触发储存, 自动寻找第一个 OreBit[s]==0 的槽位
    if (action_free == true && manual_store_start &&
        !manual_ore_auto_active &&
        gripper.getState() == GripperState::CLOSING &&
        take_ore_state == TakeOre_enum::Zero)
    {
        static const ActionGroup_enum store_actions[4] = {
            ActionGroup_enum::StoreOre_LF, ActionGroup_enum::StoreOre_RF,
            ActionGroup_enum::StoreOre_LB, ActionGroup_enum::StoreOre_RB
        };
        static const TakeOre_enum store_states[4] = {
            TakeOre_enum::StoreOre_LF, TakeOre_enum::StoreOre_RF,
            TakeOre_enum::StoreOre_LB, TakeOre_enum::StoreOre_RB
        };
        int selected_slot = -1;
        if (manual_slot_mode)
        {
            // Manual mode intentionally bypasses OreBit; the operator owns slot safety.
            selected_slot = manual_slot_index;
        }
        else
        {
            // Automatic mode keeps the original LF->RF->LB->RB empty-slot search.
            for (int s = 0; s < 4; s++)
            {
                if (OreBit[s] == 0)
                {
                    selected_slot = s;
                    break;
                }
            }
        }

        if (selected_slot >= 0)
        {
            manual_ore_auto_active = true;
            action_enum = store_actions[selected_slot];
            add_once_flag = 1;
            ore_ing[selected_slot] = 1;
            take_ore_state = store_states[selected_slot];
        }
    }
    
            //取矿1
            else if(get_ore_num[0] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ manual_take_key_pressed)
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
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre1_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::OPENING &&*/ manual_take_key_pressed)
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
            else if(get_ore_num[1] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ manual_take_key_pressed)
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
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre2_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ manual_take_key_pressed)
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
            else if(get_ore_num[2] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ manual_take_key_pressed)
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
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre3_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ manual_take_key_pressed)
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
            else if(get_ore_num[3] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ manual_take_key_pressed)
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
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre4_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ manual_take_key_pressed)
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
            else if(get_ore_num[4] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ manual_take_key_pressed)
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
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre5_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ manual_take_key_pressed)
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
            else if(get_ore_num[5] == 1 && action_free == true && take_ore_state == TakeOre_enum::Zero && /*gripper.isObjectGripped() == false &&*/ manual_take_key_pressed)
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
            else if(action_free == true && take_ore_state == TakeOre_enum::TakeOre6_ing && /*gripper.isObjectGripped() == true gripper.getState() == GripperState::CLOSING &&*/ manual_take_key_pressed)
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
    else if (action_free == true && manual_takeout_start &&
             !manual_ore_auto_active &&
             take_ore_state == TakeOre_enum::Zero)
    {
        static const ActionGroup_enum takein_actions[4] = {
            ActionGroup_enum::TakeInOre_LF, ActionGroup_enum::TakeInOre_RF,
            ActionGroup_enum::TakeInOre_LB, ActionGroup_enum::TakeInOre_RB
        };
        static const TakeOre_enum takein_states[4] = {
            TakeOre_enum::TakeInOre_LF, TakeOre_enum::TakeInOre_RF,
            TakeOre_enum::TakeInOre_LB, TakeOre_enum::TakeInOre_RB
        };
        int selected_slot = -1;
        if (manual_slot_mode)
        {
            // Manual mode intentionally bypasses OreBit; the operator owns slot safety.
            selected_slot = manual_slot_index;
        }
        else
        {
            // Automatic mode keeps the original LF->RF->LB->RB occupied-slot search.
            for (int s = 0; s < 4; s++)
            {
                if (OreBit[s] == 1)
                {
                    selected_slot = s;
                    break;
                }
            }
        }

        if (selected_slot >= 0)
        {
            manual_ore_auto_active = true;
            if (gripper.getState() != GripperState::OPENING)
                gripper.open();
            action_enum = takein_actions[selected_slot];
            add_once_flag = 1;
            takein_phase = 1U;
            take_ore_state = takein_states[selected_slot];
        }
    }
    else
    {
        action_enum = ActionGroup_enum::NuLLAction;
    }
    
    // 退出储矿逻辑 — 4 槽
    for (int s = 0; s < 4; s++)
    {
        if (action_free == true && ore_ing[s] == 3U)
        {
            static const ActionGroup_enum exit_actions[4] = {
                ActionGroup_enum::ExitStoreOre_LF, ActionGroup_enum::ExitStoreOre_RF,
                ActionGroup_enum::ExitStoreOre_LB, ActionGroup_enum::ExitStoreOre_RB
            };
            ore_ing[s] = 4U;
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
    }

    // 执行动作组
    executeActionGroup(joints, trajectory_arrays);

    // 自动流程接管期间禁止手动夹爪和手动取储矿状态机改写控制状态。
    if (!auto_owns_cycle && !auto_four_ore_active)
    {
        handleStoreOre(joints, gripper, toBottomdata, reBottomdata);
        handleTakeOre(joints, gripper, toBottomdata);
        handleOreIndex();
        handleTakeBellyOre(joints, gripper, toBottomdata);
    }

    bool execution_free = true;
    for (uint8_t i = 0U; i < 7U; ++i)
    {
        if (!joints[i].isTrajectoryPlannerFree())
        {
            execution_free = false;
            break;
        }
    }

    // 自动储/取的最终退出动作完成后释放夹爪控制权。
    // 手动输入的边沿状态由 Keyboard.cpp 在锁定期间持续同步，恢复时不会补触发。
    if (manual_ore_auto_active && execution_free &&
        add_once_flag == 0U && take_ore_state == TakeOre_enum::Zero)
    {
        manual_ore_auto_active = false;
    }

    last_action_done =
        (!entry_action_pending && !auto_four_ore_active && execution_free &&
         add_once_flag == 0U && take_ore_state == TakeOre_enum::Zero) ? 1U : 0U;
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
    return last_action_done;
}
