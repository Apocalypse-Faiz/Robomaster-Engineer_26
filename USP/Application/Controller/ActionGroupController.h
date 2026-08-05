// ActionGroupController.h
#ifndef __ACTIONGROUPCONTROLLER_H__
#define __ACTIONGROUPCONTROLLER_H__

#ifdef __cplusplus
#include "internal.h"
#include "SRML.h"
#include "Joint.h"
#include "Trajectory.h"
#include "ActionGroup.h"
#include "Claw.h"

/* 取能量单元状态枚举 */
enum class TakeOre_enum {
    Zero = 0,
    TakeOre1_prepare, TakeOre2_prepare, TakeOre3_prepare,
    TakeOre4_prepare, TakeOre5_prepare, TakeOre6_prepare,
    TakeOre1_ing, TakeOre2_ing, TakeOre3_ing,
    TakeOre4_ing, TakeOre5_ing, TakeOre6_ing,
    TakeOre1_end, TakeOre2_end, TakeOre3_end,
    TakeOre4_end, TakeOre5_end, TakeOre6_end,
    // 储矿槽 — 4 槽 (LF=左前, RF=右前, LB=左后, RB=右后)
    StoreOre_LF, StoreOre_RF, StoreOre_LB, StoreOre_RB,
    ExitStoreOre_LF, ExitStoreOre_RF, ExitStoreOre_LB, ExitStoreOre_RB,
    TakeInOre_LF, TakeInOre_RF, TakeInOre_LB, TakeInOre_RB,
    TakeinOreLF_end, TakeinOreRF_end, TakeinOreLB_end, TakeinOreRB_end
};

/* 一键四矿流程状态。取矿顺序固定为 2、3、6、5，储矿顺序为 LF、LB、RF、RB。 */
enum class AutoFourOreState : uint8_t {
    Idle = 0,
    OpenGripper,
    WaitOpenGripper,
    WaitTakePrepare,
    WaitCloseGripper,
    WaitStore,
    WaitRelease,
    WaitExitToNextPrepare,
    WaitExitStore,
    WaitAbortReturn,
    Completed,
    Aborted,
    Fault
};

enum class AutoFourOreError : uint8_t {
    None = 0,
    Busy,
    ArmNotCalibrated,
    StorageNotEmpty,
    GripperError,
    GripperCommandOverridden,
    ActionTimeout,
    UserAborted
};

/* 动作组控制器类 - 专门处理动作组逻辑 */
class ActionGroupController {
private:
    // 状态变量
    TakeOre_enum take_ore_state;
    uint8_t ore_ing[4];
    uint8_t get_ore_num[6];
    uint8_t ore_index;

    // Manual one-key store/take-out sequencing state.
    uint8_t takein_phase;
    uint32_t manual_gripper_action_tick;
    bool last_manual_store_pressed;
    bool last_manual_takeout_pressed;

    // Manual belly-slot selection: false = OreBit auto selection, true = VT13 mouse-selected slot.
    bool manual_slot_mode;
    uint8_t manual_slot_index;
    bool last_mouse_left_pressed;
    bool last_mouse_right_pressed;

    // 入场动作组标志: init() 后首次 update() 自动触发一次起始动作组
    bool entry_action_pending = false;

    // 临时变量
    ActionGroup_enum action_enum;
    uint8_t add_once_flag;
    uint8_t last_action_done;

    // 一键四矿状态变量
    AutoFourOreState auto_four_ore_state;
    AutoFourOreError auto_four_ore_error;
    uint8_t auto_four_ore_count;
    uint8_t auto_four_current_ore;
    uint8_t auto_four_current_slot;
    uint32_t auto_state_enter_tick;
    bool auto_four_ore_active;
    bool last_auto_start_pressed;
    bool last_auto_abort_pressed;

    void initFlags(); // 仅清标志位, 不触发入场动作组 (供 reset 调用)

public:
    ActionGroupController();

    /**
     * @brief 初始化控制器 (含入场动作组触发)
     */
    void init();

    /**
     * @brief 更新动作组控制
     * @param joints 关节数组
     * @param trajectory_arrays 轨迹数组
     * @param gripper 夹爪对象
     * @param toBottomdata 底盘数据
     * @param reBottomdata 接收的底盘数据
     */
    void update(Joint joints[], TrajectoryPointArray trajectory_arrays[], Gripper& gripper, ToBottomDataStructdef& toBottomdata, ReBottomDataStructdef& reBottomdata);

    /**
     * @brief 重置控制器状态 (不清入场动作组触发)
     */
    void reset();
    
    /**
     * @brief 获取当前状态
     */
    TakeOre_enum getState() const { return take_ore_state; }
    
    /**
     * @brief 获取当前选择的矿石编号
     */
    uint8_t getSelectedOre() const { return ore_index; }

    /** @brief Whether F/C currently use the mouse-selected belly slot. */
    bool isManualSlotMode() const { return manual_slot_mode; }

    /** @brief Selected belly slot, 0=LF, 1=RF, 2=LB, 3=RB. */
    uint8_t getSelectedBellySlot() const { return manual_slot_index; }

    /** @brief 一键四矿流程是否正在接管机械臂和夹爪。 */
    bool isAutoFourOreRunning() const { return auto_four_ore_active; }

    /** @brief 已成功储存的矿数，范围 0~4。 */
    uint8_t getAutoFourOreCount() const { return auto_four_ore_count; }

    /** @brief 获取一键四矿流程状态，便于调试或 UI 显示。 */
    AutoFourOreState getAutoFourOreState() const { return auto_four_ore_state; }

    /** @brief 获取最近一次一键四矿错误。 */
    AutoFourOreError getAutoFourOreError() const { return auto_four_ore_error; }
    
    /**
     * @brief 设置UI模式
     */
    void setUIMode(uint8_t mode) { UI_Mode = mode; }

     /**
     * @brief 动作组是否执行完毕
     */
    int isActionGroupCompleted();
    
private:
    /**
     * @brief 处理储矿逻辑
     */
    void handleStoreOre(Joint joints[], Gripper& gripper, ToBottomDataStructdef& toBottomdata, ReBottomDataStructdef& reBottomdata);
    
    /**
     * @brief 处理取矿逻辑
     */
    void handleTakeOre(Joint joints[], Gripper& gripper, ToBottomDataStructdef& toBottomdata);
    
    /**
     * @brief 处理取肚矿逻辑
     */
    void handleTakeBellyOre(Joint joints[], Gripper& gripper, ToBottomDataStructdef& toBottomdata);

    /**
     * @brief 处理选矿逻辑
     */
    void handleOreIndex();

    /** @brief Toggle automatic/manual belly-slot selection and advance the manual index. */
    void handleBellySlotSelection();

    /** @brief 处理 Shift+V 启动、Shift+C 急停回归及自动取储矿流程。 */
    bool handleAutoFourOre(Joint joints[], TrajectoryPointArray trajectory_arrays[],
                           Gripper& gripper, bool action_free);

    /** @brief 将一个动作组提交给统一执行器。 */
    void startActionGroup(ActionGroup_enum action);

    /** @brief 进入自动流程故障/中止状态，不强制打断当前机械臂轨迹。 */
    void stopAutoFourOre(AutoFourOreError error);
    
    /**
     * @brief 执行动作组
     */
    void executeActionGroup(Joint joints[], TrajectoryPointArray trajectory_arrays[]);
};

extern "C" {
    extern ActionGroupController action_group_controller;
}

#endif
#endif
