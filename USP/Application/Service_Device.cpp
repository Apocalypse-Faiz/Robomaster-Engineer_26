/**
 ******************************************************************************
 * @file   task.cpp
 * @brief  freertos task running file.
 ******************************************************************************
 * @note
 *  - Before running your devices, just do what you want ~ !
 *  - More devices or using other classification is decided by yourself ~ !
 ===============================================================================
								   Task List
 ===============================================================================
 * <table>
 * <tr><th>Task Name     <th>Priority          <th>Frequency/Hz    <th>Stack/Byte
 * <tr><td>              <td>                  <td>                <td>
 * </table>
 *
*/

/* Includes ------------------------------------------------------------------*/
#include "internal.h"
#include "TopState.h"
#include "Keyboard.h"
#include "usbd_cdc_if.h"
// #include "drv_VirtualCom.h"
#include "ActionGroupController.h"

/* Private define ------------------------------------------------------------*/
#define ENABLE_J1_INERTIA_VALIDATION 0

// 底盘姿态反馈使用弧度制；对应 10°收回、7°释放，并设置回差避免反复伸缩。
static const float GIMBAL_LIFT_RETRACT_TILT_RAD = 0.17453293f;
static const float GIMBAL_LIFT_RELEASE_TILT_RAD = 0.12217305f;
static const uint32_t GIMBAL_LIFT_FEEDBACK_TIMEOUT_MS = 100U;
static bool gimbal_lift_tilt_retract_active = false;

/* Private variables ---------------------------------------------------------*/
TaskHandle_t Motor_Handle;
TaskHandle_t IMU_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t VT13_Handle; // VT13遥控器任务，如果不用了可以删除掉
TaskHandle_t FS_I6X_Handle;
TaskHandle_t Rx_Referee_Handle;
TaskHandle_t Sx_Referee_Handle;
TaskHandle_t Log_Handle;
TaskHandle_t tskBackpackdrive_Handle; // 杂项机构任务
TaskHandle_t StateMachine_Handle;	  // 状态机任务
TaskHandle_t BoardCom_Handle;  //通信任务
#if ENABLE_J1_INERTIA_VALIDATION
volatile float j1_validation_cmd_debug = 0.0f;            // J1验证模式输出力矩命令（Nm）
volatile float j1_validation_inertia_total_debug = 0.0f;  // 当前姿态估计的J1总等效惯量 I_total
volatile float j1_validation_k_dynamic_debug = 0.0f;      // 动态刚度 K_dynamic = k * I_total
volatile float j1_validation_theta_center_debug = 0.0f;   // 虚拟弹簧中心角 theta1_center（deg）
volatile float j1_validation_spring_k_debug = 30.0f;       // 虚拟弹簧比例系数 k
volatile float j1_validation_torque_limit_debug = 15.0f;   // 力矩限幅（Nm）
volatile float j1_validation_speed_limit_debug = 20.0f;   // 超速保护阈值（同 motor[0].getSpeed() 单位）
volatile float j1_validation_recenter_req_debug = 0.0f;   // 置1后下一周期重置中心角，随后自动清零
volatile float j1_validation_period_debug = 0.0f;         // 自动测得的振荡周期（ms），各姿态对比此值
#endif
/* Private function declarations ---------------------------------------------*/
void tskMotor(void *arg);
void tskFS_I6X(void *arg);
void tskIMU(void *arg);
void tskDR16(void *arg);
void tskVT13(void *arg); // VT13遥控器任务，如果不用了可以删除掉
void Rx_Referee(void *arg);
void Sx_Referee(void *arg);
void tskLog(void *arg);
void tskBackpackdrive(void *arg);
void tskStateMachine(void *arg);
void tskBoardCom(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialization of device management service
 * @param  None.
 * @return None.
 */
void Service_Devices_Init(void)
{
	xTaskCreate(tskMotor, "App.Motor", Huge_Stack_Size + Huge_Stack_Size, NULL, PrioritySuperHigh, &Motor_Handle);
	xTaskCreate(tskStateMachine, "App.StateMachine", Huge_Stack_Size + Normal_Stack_Size, NULL, PriorityNormal, &StateMachine_Handle);
	xTaskCreate(tskBackpackdrive, "App.Backpackdrive", Normal_Stack_Size, NULL, PriorityNormal, &tskBackpackdrive_Handle);
	xTaskCreate(tskBoardCom, "App.BoardCom", Normal_Stack_Size, NULL, PriorityNormal, &BoardCom_Handle);

#if USE_SRML_MPU6050
	xTaskCreate(tskIMU, "App.IMU", Normal_Stack_Size, NULL, PrioritySuperHigh, &IMU_Handle);
#endif

#if USE_SRML_DR16
	xTaskCreate(tskDR16, "App.DR16", Small_Stack_Size, NULL, PrioritySuperHigh, &DR16_Handle);
#endif

#if USE_SRML_VT13
	xTaskCreate(tskVT13, "App.VT13", Normal_Stack_Size, NULL, PrioritySuperHigh, &VT13_Handle);
#endif

#if USE_SRML_LOG
	xTaskCreate(tskLog, "APP.Log", Normal_Stack_Size, NULL, PriorityNormal, &Log_Handle);
#endif

#if USE_SRML_FS_I6X
	xTaskCreate(tskFS_I6X, "App.FS_I6X", Small_Stack_Size, NULL, PrioritySuperHigh, &FS_I6X_Handle);
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
	Referee.CC_Mutex_init(Referee_VT_mutex);
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
	xTaskCreate(Sx_Referee, "Sx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Sx_Referee_Handle);
#endif
}

void tskStateMachine(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	VisionFrameBuffer vision_rx_frame;
	ReVisionDataStruct vision_control_snapshot;
	xLastWakeTime_t = xTaskGetTickCount();
	Total_init();
	vTaskDelayUntil(&xLastWakeTime_t, 1000);
	for (;;)
	{
		while (Visiondata_TakeLatestFrame(&vision_rx_frame) != 0U)
		{
			if (Visiondata_DecodeControlFrame(&vision_rx_frame, &vision_control_snapshot) != 0U)
			{
				/* reVisiondata is published and consumed only by this task. */
				memcpy(&reVisiondata, &vision_control_snapshot, sizeof(reVisiondata));
				xTaskNotify(StateMachine_Handle, VISION_NOTIFY_BIT, eSetBits);
			}
		}

		/* wait for next circle */
		TopStateMachine.action();
		// if (ulTaskNotifyTake(pdTRUE, 0) == 55)
		// {
		// 	for (int i = 0; i < 6; i++)
		// 	{
		// 		toVisiondata.joint_current[i] = reVisiondata.joint_target[i];
		// 	}
		// 	toVisiondata.seq_num = reVisiondata.seq_num;
		// 	toVisiondata.timestamp = HAL_GetTick();
		// 	Visiondata_Transmist();	
		// }

		vTaskDelayUntil(&xLastWakeTime_t, 1);

		// 状态机运行代码放在这里

	}
}

void tskBackpackdrive(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	int increase_buffer = 0;
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 10);

		// 杂项机构运行代码放在这里
		/* 矿石检测 (GPIO 低电平有效) */
		OreBit[0] = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) ? 1 : 0;
		OreBit[1] = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) ? 1 : 0;
		OreBit[2] = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_RESET) ? 1 : 0;
		OreBit[3] = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET) ? 1 : 0;

		/* 指示灯控制 */
		// 末端夹爪状态观察（闭合指令态点亮）
		// if (gripper.getState() == GripperState::CLOSING)
		// {
		// 	WS_indicator[0] = WS_Green;
		// }
		// else
		// {
		// 	WS_indicator[0] = WS_Yellow;
		// }

		// if (reBottomdata.ChassisState == 0)
		// {
		// 	WS_indicator[1] = WS_Red;
		// }
		// else if (reBottomdata.ChassisState == 1)
		// {
		// 	WS_indicator[1] = WS_Green;
		// }
		// else if (reBottomdata.ChassisState == 2)
		// {
		// 	WS_indicator[1] = WS_Blue;
		// }
		// else if (reBottomdata.ChassisState == 3 || reBottomdata.ChassisState == 4)
		// {
		// 	WS_indicator[1] = WS_Cyan;
		// }
		// else
		// {
		// 	WS_indicator[1] = WS_Yellow;
		// }
		
		
		// 储矿夹爪状态观察
		// if (reBottomdata.L_Store == 1)
		// {
		// 	WS_indicator[5] = WS_White;
		// }
		// else
		// {
		// 	WS_indicator[5] = WS_Black;
		// }

		// if (reBottomdata.R_Store == 1)
		// {
		// 	WS_indicator[4] = WS_White;
		// }
		// else
		// {
		// 	WS_indicator[4] = WS_Black;
		// }

		// 电机状态观察
		// if (DM8009.GetMotorState() != 0 || DM4310.GetMotorState() != 0 || DM10010.GetMotorState() != 0 || DM4340.GetMotorState() != 0)
		// {
		// 	WS_indicator[4] = WS_Red;
		// 	WS_indicator[5] = WS_Red;
		// }
		// else
		// {
		// 	WS_indicator[4] = WS_Green;
		// 	WS_indicator[5] = WS_Green;
		// }

		// 指示灯状态更新
		for (int i = 0; i < 6; i++)
		{
			switch (WS_indicator[i])
			{
			case WS_Black:
				Indicator.Change_Singal_RGB(0x01 << i, 0, 0, 0, 0);
				break;
			case WS_Red:
				Indicator.Change_Singal_RGB(0x01 << i, 255, 0, 0, 255);
				break;
			case WS_Green:
				Indicator.Change_Singal_RGB(0x01 << i, 0, 255, 0, 255);
				break;
			case WS_Blue:
				Indicator.Change_Singal_RGB(0x01 << i, 0, 0, 255, 255);
				break;
			case WS_Yellow:
				Indicator.Change_Singal_RGB(0x01 << i, 255, 255, 0, 255);
				break;
			case WS_Cyan:
				Indicator.Change_Singal_RGB(0x01 << i, 0, 255, 255, 255);
				break;
			case WS_Purple:
				Indicator.Change_Singal_RGB(0x01 << i, 255, 0, 255, 255);
				break;
			case WS_White:
				Indicator.Change_Singal_RGB(0x01 << i, 255, 255, 255, 255);
				break;
			default:
				break;
			}
		}

		Indicator.Update();

	}
}

namespace
{
// Power up with the arm disabled.  Even if the remote link is already
// established, control must pass through the stable-link release path so that
// every joint target is synchronized to its measured position before torque
// output is enabled.
volatile bool arm_safety_latched = true;

enum class DmRecoveryPhase : uint8_t
{
	Idle,
	SendClearError,
	SendStart,
	WaitReady
};

bool ArmControlLinkEstablished()
{
	bool established = false;
#if USE_SRML_DR16
	established = (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED);
#endif
#if USE_SRML_VT13
	established = established ||
		((VT13.GetModeSwitch() != MS_Status_Typedef::MS_C) &&
		 (VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED));
#endif
	return established;
}

void ClearArmFeedforward()
{
	for (uint8_t i = 0; i < 7U; i++)
	{
		Gravity_feedforward[i] = 0.0f;
		BackEMF_feedforward[i] = 0.0f;
		Coulomb_feedforward[i] = 0.0f;
		Viscous_feedforward[i] = 0.0f;
		Inertia_feedforward[i] = 0.0f;

		motor[i].setGravity_feedforward(0.0f);
		motor[i].setBackEMF_feedforward(0.0f);
		motor[i].setCoulomb_feedforward(0.0f);
		motor[i].setViscous_feedforward(0.0f);
		motor[i].setInertia_feedforward(0.0f);
		motor[i].setSpeed_feedforward(0.0f);
		motor[i].setAcceleration_feedforward(0.0f);
	}

}

void ClearMotorFeedforward(uint8_t motor_index)
{
	Gravity_feedforward[motor_index] = 0.0f;
	BackEMF_feedforward[motor_index] = 0.0f;
	Coulomb_feedforward[motor_index] = 0.0f;
	Viscous_feedforward[motor_index] = 0.0f;
	Inertia_feedforward[motor_index] = 0.0f;

	motor[motor_index].setGravity_feedforward(0.0f);
	motor[motor_index].setBackEMF_feedforward(0.0f);
	motor[motor_index].setCoulomb_feedforward(0.0f);
	motor[motor_index].setViscous_feedforward(0.0f);
	motor[motor_index].setInertia_feedforward(0.0f);
	motor[motor_index].setSpeed_feedforward(0.0f);
	motor[motor_index].setAcceleration_feedforward(0.0f);
}

void ReleaseArmSafetyStop()
{
	ClearArmFeedforward();

	for (uint8_t i = 0; i < 7U; i++)
	{
		joint_trajectory_point_array[i].clear();
		if (motor[i].getStatus() == MotorStatus::ready)
		{
			joint[i].setMode(JointMode::angle);
			// setMode(angle) 先把目标置为当前角度，再将规划器同步到该目标。
			joint[i].stopTrajectoryPlanner();
		}
		else
		{
			joint[i].setMode(JointMode::disabled);
			motor[i].forceSetMode(MotorMode::disabled);
		}
	}

	arm_safety_latched = false;
}
} // namespace

void ArmSafety_Trip()
{
	const bool first_trip = !arm_safety_latched;
	arm_safety_latched = true;
	all_motor_connected = false;

	if (first_trip)
	{
		for (uint8_t i = 0; i < 7U; i++)
		{
			joint_trajectory_point_array[i].clear();
			joint[i].stopTrajectoryPlanner();
		}
	}

	ClearArmFeedforward();
	for (uint8_t i = 0; i < 7U; i++)
	{
		joint[i].setMode(JointMode::disabled);
		// setMode() 会在电机 error 时拒绝写入，因此再强制设置电机层模式。
		motor[i].forceSetMode(MotorMode::disabled);
	}
	gripper.emergencyStop();
}

bool ArmSafety_IsLatched()
{
	return arm_safety_latched;
}

void CheckMotorStatus()
{
	static constexpr uint8_t DM_MOTOR_COUNT = 7U;
	static constexpr uint8_t READY_STABLE_TICKS = 20U;
	const TickType_t DM_RECOVERY_FRAME_GAP = pdMS_TO_TICKS(2U);
	const TickType_t DM_RECOVERY_TIMEOUT = pdMS_TO_TICKS(1000U);

	static DmRecoveryPhase recovery_phase = DmRecoveryPhase::Idle;
	static uint8_t recovery_scan_idx = 0U;
	static uint8_t recovery_active_idx = 0U;
	static uint8_t motor_fault_latched[DM_MOTOR_COUNT] = {0U};
	static uint8_t motor_ready_stable_cnt[DM_MOTOR_COUNT] = {0U};
	static uint8_t control_link_stable_cnt = 0U;
	static TickType_t recovery_last_action_tick = 0U;
	static TickType_t recovery_start_tick = 0U;

	const bool control_link_established = ArmControlLinkEstablished();
	const bool top_state_blocks_control =
		(g_top_state_id == TopStateId::Lost) || (g_top_state_id == TopStateId::None);
	bool joint_reference_ready = true;
	for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++)
	{
		if ((motor[i].getStatus() != MotorStatus::ready) || !joint[i].isCalibrated())
		{
			joint_reference_ready = false;
			break;
		}
	}
	const bool full_arm_stop_required =
		!control_link_established || top_state_blocks_control || !joint_reference_ready;

	// 遥控链路丢失或处于 Lost/None 时仍锁存全臂安全停机。
	if (full_arm_stop_required)
	{
		control_link_stable_cnt = 0U;
		ArmSafety_Trip();
	}
	else if (ArmSafety_IsLatched())
	{
		if (control_link_stable_cnt < READY_STABLE_TICKS)
		{
			control_link_stable_cnt++;
		}

		if (control_link_stable_cnt >= READY_STABLE_TICKS)
		{
			// 链路连续有效后解除整臂锁存；仍故障的电机在下方继续单独隔离。
			ReleaseArmSafetyStop();
			control_link_stable_cnt = 0U;
		}
		else
		{
			// 等待链路稳定期间持续重申整臂失能。
			ArmSafety_Trip();
		}
	}
	else
	{
		control_link_stable_cnt = 0U;
	}

	bool all_ready = true;
	for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++)
	{
		if (motor[i].getStatus() != MotorStatus::ready)
		{
			all_ready = false;
			motor_ready_stable_cnt[i] = 0U;

			if (motor_fault_latched[i] == 0U)
			{
				// 只终止故障关节的轨迹，其他关节继续按原逻辑运行。
				joint_trajectory_point_array[i].clear();
				joint[i].stopTrajectoryPlanner();
				joint[i].setMode(JointMode::disabled);
				motor_fault_latched[i] = 1U;
			}

			ClearMotorFeedforward(i);
			// 状态异常时 setMode() 会拒绝写入，因此每周期强制保持零力矩模式。
			motor[i].forceSetMode(MotorMode::disabled);
		}
		else if (motor_fault_latched[i] != 0U)
		{
			// 刚恢复通信的电机先保持停机，连续 ready 20 ms 后再单独恢复。
			ClearMotorFeedforward(i);
			motor[i].forceSetMode(MotorMode::disabled);

			if (motor_ready_stable_cnt[i] < READY_STABLE_TICKS)
			{
				motor_ready_stable_cnt[i]++;
			}

			if ((motor_ready_stable_cnt[i] >= READY_STABLE_TICKS) &&
				!ArmSafety_IsLatched())
			{
				joint_trajectory_point_array[i].clear();
				joint[i].setMode(JointMode::angle);
				// 从当前位置恢复，避免沿用故障前的旧目标造成突跳。
				joint[i].stopTrajectoryPlanner();
				motor_fault_latched[i] = 0U;
				motor_ready_stable_cnt[i] = 0U;
			}
		}
		else
		{
			motor_ready_stable_cnt[i] = 0U;
		}
	}

	// 该变量仅表示连接状态，不再作为整臂控制的使能条件。
	all_motor_connected = all_ready;

	const TickType_t now_tick = xTaskGetTickCount();
	if (all_ready)
	{
		recovery_phase = DmRecoveryPhase::Idle;
		return;
	}

	if (recovery_phase == DmRecoveryPhase::Idle)
	{
		// 故障电机按顺序自动清错、启动，不阻塞其他电机的控制。
		recovery_scan_idx = 0U;
		recovery_start_tick = now_tick;
		recovery_last_action_tick = now_tick - DM_RECOVERY_FRAME_GAP;
		recovery_phase = DmRecoveryPhase::SendClearError;
	}

	if ((now_tick - recovery_start_tick) >= DM_RECOVERY_TIMEOUT)
	{
		// 本轮未恢复则回到 Idle；下一周期会自动重新清错、启动。
		recovery_phase = DmRecoveryPhase::Idle;
		return;
	}

	switch (recovery_phase)
	{
	case DmRecoveryPhase::SendClearError:
		if ((now_tick - recovery_last_action_tick) < DM_RECOVERY_FRAME_GAP)
		{
			return;
		}

		while ((recovery_scan_idx < DM_MOTOR_COUNT) &&
			   (motor[recovery_scan_idx].getStatus() == MotorStatus::ready))
		{
			recovery_scan_idx++;
		}

		if (recovery_scan_idx >= DM_MOTOR_COUNT)
		{
			recovery_phase = DmRecoveryPhase::WaitReady;
			return;
		}

		recovery_active_idx = recovery_scan_idx;
		drive_dm_motor[recovery_active_idx].clearError();
		recovery_last_action_tick = now_tick;
		recovery_phase = DmRecoveryPhase::SendStart;
		return;

	case DmRecoveryPhase::SendStart:
		if ((now_tick - recovery_last_action_tick) < DM_RECOVERY_FRAME_GAP)
		{
			return;
		}

		drive_dm_motor[recovery_active_idx].startMotor();
		recovery_scan_idx = recovery_active_idx + 1U;
		recovery_last_action_tick = now_tick;
		recovery_phase = DmRecoveryPhase::SendClearError;
		return;

	case DmRecoveryPhase::WaitReady:
		// 等待故障电机反馈 ready；超时后由上面的逻辑重新发起恢复。
		return;

	case DmRecoveryPhase::Idle:
	default:
		return;
	}
}

/**
 * @brief 云台升降电机 (Gimbal_Lift, DJI2006) 力矩复位标定
 * @note  参考旧 ResetState 力矩复位逻辑. 需在 tskMotor 主循环周期调用,
 *        靠 static 状态机分阶段推进:
 *          1) 从当前角度持续自减目标角, 让电机往限位方向转
 *          2) 监测回传力矩, 超阈值判定堵转到限位
 *          3) 堵转后: 先 disabled, 再 calibrate 标定当前位置为 0°, 再设回 angle
 *          4) 标定完成: 置 gimbal_lift_init_done, 并记录 gimbal_lift_max_angle
 */
void Gimbal_Lift_Init()
{
    // ── 复位参数 (TODO: 实机标定) ──
    static const float DECREASE_STEP   = 20.0f;    // 每周期目标角自减量 (度), 决定下压速度
    static const float TORQUE_THRESHOLD = 1.0f;   // 堵转力矩阈值 (与 getTorque 同单位), 超过判定到限位
    static const float GIMBAL_LIFT_RANGE = 23000.0f; // 标定后可用行程 (度): 0° 为限位, +RANGE 为另一端 (TODO 实测)

    static MeanFilter<20> torque_filter;          // 力矩低通, 抑制噪声误判
    enum class InitState { DECREASING, CALIBRATED };
    static InitState state = InitState::DECREASING;

    if (gimbal_lift_init_done)
    {
        return;
    }

    // 电机未就绪时不动作, 等待上线
    if (Gimbal_Lift.getStatus() != MotorStatus::ready)
    {
        return;
    }

    switch (state)
    {
    case InitState::DECREASING:
    {
        // 确保处于角度模式 (首次进入)
        if (Gimbal_Lift.getMode() != MotorMode::angle)
        {
            Gimbal_Lift.setMode(MotorMode::angle);
        }

        float torque = torque_filter.f(fabsf(Gimbal_Lift.getTorque()));

        if (torque < TORQUE_THRESHOLD)
        {
            // 未堵转: 目标角持续自减, 往限位方向压
            Gimbal_Lift.setTarget(Gimbal_Lift.target_angle - DECREASE_STEP);
        }
        else
        {
            // 堵转到限位: 先失能, 再标定当前位置为 0°, 再设回角度模式
            Gimbal_Lift.setMode(MotorMode::disabled);
            Gimbal_Lift.calibrate();          // 标定当前角为 0°
            Gimbal_Lift.setMode(MotorMode::angle);
            Gimbal_Lift.setTarget(0.0f);      // 标定后目标设为 0°, 顶住限位

            gimbal_lift_max_angle = GIMBAL_LIFT_RANGE; // 记录可用最大角度
			gimbal_lift_min_angle = 200.0f;
            gimbal_lift_init_done = true;
            state = InitState::CALIBRATED;
        }
        break;
    }
    case InitState::CALIBRATED:
    default:
        // 已完成, 不再处理 (gimbal_lift_init_done 已为 true, 上面已 return)
        break;
    }
}

static void Gimbal_Lift_UpdateTiltProtection()
{
    static uint32_t last_rx_ok_packets = 0U;
    static TickType_t last_feedback_tick = 0U;

    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t rx_ok_packets = g_bottom_com_diag_stats.rx_ok_packets;
    if (rx_ok_packets != last_rx_ok_packets)
    {
        last_rx_ok_packets = rx_ok_packets;
        last_feedback_tick = now_tick;
    }

    // 未收到过有效反馈，或反馈中断时按危险姿态处理，避免误伸出。
    if (rx_ok_packets == 0U ||
        (now_tick - last_feedback_tick) > pdMS_TO_TICKS(GIMBAL_LIFT_FEEDBACK_TIMEOUT_MS))
    {
        gimbal_lift_tilt_retract_active = true;
        return;
    }

    const float pitch_abs = fabsf(reBottomdata.chassis_angle[0]);
    const float roll_abs = fabsf(reBottomdata.chassis_angle[1]);

    if (!gimbal_lift_tilt_retract_active &&
        (pitch_abs >= GIMBAL_LIFT_RETRACT_TILT_RAD ||
         roll_abs >= GIMBAL_LIFT_RETRACT_TILT_RAD))
    {
        gimbal_lift_tilt_retract_active = true;
    }
    else if (gimbal_lift_tilt_retract_active &&
             pitch_abs <= GIMBAL_LIFT_RELEASE_TILT_RAD &&
             roll_abs <= GIMBAL_LIFT_RELEASE_TILT_RAD)
    {
        gimbal_lift_tilt_retract_active = false;
    }
}

/**
 * @brief 云台升降电机步进式目标角设定
 * @param goal_deg  最终目标角度 (度), 0°=下限位, gimbal_lift_max_angle=上限位
 * @param step_deg  每周期最大步长 (度), 正=往上, 负=往下
 * @note  需在 tskMotor 主循环中周期调用 (每 1ms).
 *        内部自动 cl(map) 到 [0, gimbal_lift_max_angle] 范围.
 *        若未完成力矩复位标定则直接设 goal_deg (fallback).
 */
void Gimbal_Lift_StepToGoal(float goal_deg, float step_deg)
{
    if (Gimbal_Lift.getStatus() != MotorStatus::ready)
        return;
    if (Gimbal_Lift.getMode() != MotorMode::angle)
        Gimbal_Lift.setMode(MotorMode::angle);

    // 限位 clamp (力矩复位完成后才有意义)
    if (gimbal_lift_init_done)
    {
        // 倾角保护优先于各状态请求的云台升降目标。
        if (gimbal_lift_tilt_retract_active)
        {
            goal_deg = gimbal_lift_min_angle;
        }

        goal_deg = std_lib::constrain(goal_deg, gimbal_lift_min_angle, gimbal_lift_max_angle);
    }

    float current_target = Gimbal_Lift.target_angle;
    float diff = goal_deg - current_target;
    float step  = fabsf(diff) > fabsf(step_deg) ? step_deg : fabsf(diff);

    if (diff > 0.0f)
        Gimbal_Lift.setTarget(current_target + step);
    else if (diff < -0.0f)
        Gimbal_Lift.setTarget(current_target - step);
}

/**
 * @brief 根据关节正常范围选择 DM 上电等价圈对应的映射偏置
 */
static float SelectDmStartupOffset(float motor_angle,
								 float mapping_k,
								 float nominal_offset,
								 float joint_min,
								 float joint_max)
{
	// DM 位置反馈范围为 25 rad。上电时反馈可能选择相差 2*PI rad 的
	// 等价分支；按当前 DM 驱动的 4*360 deg/25 rad 换算，会表现为
	// 360*PI/3.125 ~= 361.91 deg 的关节角跳变。
	static const float dm_equivalent_turn_deg = 360.0f * PI / 3.125f;
	const float joint_center = 0.5f * (joint_min + joint_max);
	const float nominal_angle = motor_angle * mapping_k + nominal_offset;
	const float branch_f = (joint_center - nominal_angle) / dm_equivalent_turn_deg;
	const int branch = (branch_f >= 0.0f)
		? static_cast<int>(branch_f + 0.5f)
		: static_cast<int>(branch_f - 0.5f);

	return nominal_offset + branch * dm_equivalent_turn_deg;
}

/**
 * @brief 关节角度限位设置
 * @note 需要在获取电机初始角度后，做一些判断后再设置
 */
void Joint_limitSet()
{

	/*关节角映射，和仿真对齐*/
	static uint8_t Set_ok = 0;
	if (Set_ok == 1)
	{
		// J1 位于中心区域时，J2 > 127 deg 与 J3 < 50 deg 不允许同时发生。
		// 每周期先恢复基础软限位，再根据当前姿态收紧一侧；离开该区域后自动恢复。
		// static const float J1_COMBINED_LIMIT = 27.0f;
		// static const float J2_COMBINED_LIMIT = 127.0f;
		// static const float J3_COMBINED_LIMIT = 50.0f;
		// static const float J2_SECOND_COMBINED_LIMIT = 150.0f;
		// static const float J3_SECOND_COMBINED_LIMIT = 112.0f;
		// static const float J2_MIN_ANGLE = 46.85f;
		// static const float J2_MAX_ANGLE = 161.85f;
		// static const float J3_MIN_ANGLE = 39.8f;
		// static const float J3_MAX_ANGLE = 157.8f;

		// joint[1].setJointParam(J2_MIN_ANGLE, J2_MAX_ANGLE, -10, 10);
		// joint[2].setJointParam(J3_MIN_ANGLE, J3_MAX_ANGLE, -10, 10);

		// if (motor[0].getStatus() == MotorStatus::ready &&
		// 	motor[1].getStatus() == MotorStatus::ready &&
		// 	motor[2].getStatus() == MotorStatus::ready)
		// {
		// 	joint[0].updateJointData();
		// 	joint[1].updateJointData();
		// 	joint[2].updateJointData();

		// 	const float j1_angle = joint[0].getCurrentAngle();
		// 	const float j2_angle = joint[1].getCurrentAngle();
		// 	const float j3_angle = joint[2].getCurrentAngle();

		// 	if (fabsf(j1_angle) <= J1_COMBINED_LIMIT)
		// 	{
		// 		// 第二级联动限位：J2 > 150 deg 与 J3 < 112 deg 不允许同时发生。
		// 		if (j2_angle > J2_COMBINED_LIMIT)
		// 		{
		// 			joint[2].setJointParam(J3_COMBINED_LIMIT, J3_MAX_ANGLE, -10, 10);
		// 		}
		// 		if (j2_angle > J2_SECOND_COMBINED_LIMIT)
		// 		{
		// 			joint[2].setJointParam(J3_SECOND_COMBINED_LIMIT, J3_MAX_ANGLE, -10, 10);
		// 		}

		// 		// 先应用第二级上限，再应用更严格的第一级上限，避免后者被覆盖。
		// 		if (j3_angle < J3_SECOND_COMBINED_LIMIT)
		// 		{
		// 			joint[1].setJointParam(J2_MIN_ANGLE, J2_SECOND_COMBINED_LIMIT, -10, 10);
		// 		}
		// 		if (j3_angle < J3_COMBINED_LIMIT)
		// 		{
		// 			joint[1].setJointParam(J2_MIN_ANGLE, J2_COMBINED_LIMIT, -10, 10);
		// 		}
		// 	}
		// }

		// J3的动态限位仅依赖J2角度，且只在J2对应电机ready时更新
		// if (motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready)
		// {
		// 	joint[1].updateJointData();
		// 	float j2_angle = joint[1].getCurrentAngle();
		// 	if (j2_angle <= 90)
		// 	{
		// 		joint[2].setJointParam(-50, 50, -10, 10);
		// 	}
		// 	else if (j2_angle <= 180 && j2_angle > 90)
		// 	{
		// 		joint[2].setJointParam(-(180 - j2_angle + 50 - 90), 50, -10, 10);
		// 	}
		// }

		return;
	}

	// float Reduction_ratio = 0.0278 * 0.5; // 1/36 DJI2006的减速比再加上传动
	// float k1 = 6.28 / 12.5;

	if (motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready && motor[6].getStatus() == MotorStatus::ready)
	{

		// 以下设置映射中的偏置b是通过DM电机返回的多圈角度进行各种判断设置对应偏置（单圈内不同地方上电，同一地方反馈角度不同），实际正确方法应该是通过单圈角度来设置偏置，希望新赛季队员能够进行优化。

		// 只有初次上电的时候才会设置，复位之前暂时使用这些初始值，复位状态机中重新设置
		// 对于DM电机初次上电时在这里就已经设置好了，DJI电机需要在复位状态机中设置
		/*标定和极性设置*/
		// 相对最初代码坐标的累计校正：J2 +6.85°、J3 +7.8°、J4 +1.1°、J6 -0.4°。
		// 将零点、选圈范围和软限位一起迁移，保持原有机械行程不变。
		const float j1_offset = SelectDmStartupOffset(motor[0].getAngle(), 1.0f, 0.0f, -170.0f, 170.0f);
		const float j2_offset = SelectDmStartupOffset(motor[1].getAngle(), 1.0f, 164.85f, 60.0f, 161.85f);
		const float j3_offset = SelectDmStartupOffset(motor[2].getAngle(), 1.0f, 41.8f, 39.8f, 157.8f);
		const float j5_offset = SelectDmStartupOffset(motor[4].getAngle(), -1.0f, 0.0f, -165.0f, 165.0f);
		const float j7_offset = SelectDmStartupOffset(motor[6].getAngle(), -1.0f, 0.0f, -150.0f, 167.0f);
		const float j1_angle = motor[0].getAngle() + j1_offset;
		const float j2_angle = motor[1].getAngle() + j2_offset;
		const float j3_angle = motor[2].getAngle() + j3_offset;
		const float j5_angle = motor[4].getAngle() + j5_offset;
		const float j7_angle = -motor[6].getAngle() + j7_offset;

		// 等待第一帧有效角度；异常反馈不能据此锁定上电偏置。
		if (j1_angle < -180.0f || j1_angle > 180.0f ||
			j2_angle < 0.0f || j2_angle > 360.0f ||
			j3_angle < 0.0f || j3_angle > 360.0f ||
			j5_angle < -180.0f || j5_angle > 180.0f ||
			j7_angle < -180.0f || j7_angle > 180.0f)
		{
			return;
		}

		joint[0].setJointMapping(1, j1_offset);
		joint[1].setJointMapping(1, j2_offset);
		joint[2].setJointMapping(1, j3_offset);
		joint[3].setJointMapping(-1, -77.9f);
		joint[4].setJointMapping(-1, j5_offset);
		joint[5].setJointMapping(-1, -0.1f);
		joint[6].setJointMapping(-1, j7_offset);

		special_actuator3[0].calibrate();
		special_actuator3[1].calibrate();
		special_actuator3[2].calibrate();
		special_actuator3[3].calibrate();
		special_actuator3[4].calibrate();
		special_actuator3[5].calibrate();
		special_actuator3[6].calibrate();

		/*关节软限位*/
		joint[0].setJointParam(-170, 170, 10, -10);
		joint[1].setJointParam(60.0f, 161.85f, -10, 10);	
		joint[2].setJointParam(39.8f, 157.8f, -10, 10);
		joint[3].setJointParam(-75.9f, 18.1f, -10, 10);
		joint[4].setJointParam(-165, 165, -10, 10); // J5关节在力矩复位后还要重新再限位一下
		joint[5].setJointParam(-100.4f, 101.6f, -10, 10);
		joint[6].setJointParam(-150, 167, -10, 10);

		Set_ok = 1;
	}
}

static void UpdateMotor0PidByPose()
{
	if (motor[0].getStatus() != MotorStatus::ready || motor[1].getStatus() != MotorStatus::ready || motor[2].getStatus() != MotorStatus::ready)
	{
		return;
	}

	struct Motor0PidPosePoint
	{
		float j2;
		float j3;
		float a_kp;
		float a_ki;
		float v_kp;
	};

	static const Motor0PidPosePoint pose_points[] = {
		{70.92f, 9.65f, 0.064f, 0.0f, 5.7f},
		{65.85f, 34.76f, 0.064f, 0.017f, 5.6f},
		{66.67f, -18.08f, 0.06f, 0.0f, 6.4f},
		{45.49f, -35.90f, 0.05f, 0.0f, 6.7f},
		{48.17f, -30.63f, 0.067f, 0.0f, 5.7f},
	};

	float j2_angle = joint[1].getCurrentAngle();
	float j3_angle = joint[2].getCurrentAngle();
	float a_kp = 0.0f;
	float a_ki = 0.0f;
	float v_kp = 0.0f;

	// 坐标校正后 96.85° 对应原来的 90° 机械姿态。
	if (j2_angle >= 96.85f)
	{
		a_kp = 0.07f;
		a_ki = 0.017f;
		v_kp = 5.6f;
	}
	else
	{
		// J2<=90度区域按(J2,J3)二维标定点做加权插值，实现连续自动调参。
		const float eps = 1e-3f;
		const float exact_match_d2 = 1e-4f;
		float w_sum = 0.0f;
		float a_kp_sum = 0.0f;
		float a_ki_sum = 0.0f;
		float v_kp_sum = 0.0f;

		for (uint32_t i = 0; i < (sizeof(pose_points) / sizeof(pose_points[0])); i++)
		{
			float dj2 = j2_angle - pose_points[i].j2;
			float dj3 = j3_angle - pose_points[i].j3;
			float d2 = dj2 * dj2 + dj3 * dj3;

			if (d2 < exact_match_d2)
			{
				a_kp = pose_points[i].a_kp;
				a_ki = pose_points[i].a_ki;
				v_kp = pose_points[i].v_kp;
				w_sum = -1.0f;
				break;
			}

			float w = 1.0f / (d2 + eps);
			w_sum += w;
			a_kp_sum += w * pose_points[i].a_kp;
			a_ki_sum += w * pose_points[i].a_ki;
			v_kp_sum += w * pose_points[i].v_kp;
		}

		if (w_sum > 0.0f)
		{
			a_kp = a_kp_sum / w_sum;
			a_ki = a_ki_sum / w_sum;
			v_kp = v_kp_sum / w_sum;
		}
	}

	motor[0].setPidParameters({.v_Kp = v_kp, .v_Min = -20, .v_Max = 20, .a_Kp = a_kp, .a_Ki = a_ki, .a_Kd = 0, .a_Min = -10, .a_Max = 10, .a_IntSep = 0, .a_IntMax = 2});
}

// ============================================================================
// MDH 关节角 → 补偿函数角度 转换
// ============================================================================
//
// joint[i].getCurrentAngle() 返回的是 MDH 坐标系下的关节角:
//
// 补偿函数的角度约定 (详见 Compension.h):
//   世界系: Z₀↑, X₀→前, Y₀⊗
//   绝对俯仰角: 0°=水平前伸(∥X₀Y₀), >0°=上仰(绕+Y₀右手定则)
//   roll 角:     0°=腕部竖直, >0°=从后方看顺时针
//
// 以下函数完成 MDH角度 → 补偿函数所需角度 的转换.
// 零位偏置常量需根据 MDH 坐标系的精确定义校准.

// ── MDH 零位偏置 (度) ─────────────────────────────
// 定义: 当补偿函数期望的角度 = 0° 时, 对应的 MDH 关节角的值.
// 即: compensation_angle = MDH_angle - MDH_ZERO_OFFSET
// (TODO: 需根据 MDH 坐标系精确定义校准, 当前基于 JointMapping 参数反推)

// 绝对俯仰角零位: MDH 多少度 = 连杆水平前伸
static const float MDH_J2_ZERO_ABS = 164.0f;   // 假设 motor_raw=0° 时水平前伸, MDH=0+164=164°
                                                // (TODO: 确认 motor_raw_J2=0 时是否确实水平前伸)

// 注: MDH→相对俯仰角的零位偏置由各 MDH2xxx() 函数内部处理,
//     如需精细校准, 请在对应转换函数中增减常数.

// ── 重力补偿用角度 ─────────────────────────────────
// GravityCompension_Calculate 期望 (与惯量补偿完全一致):
//   _J2: 绝对俯仰角 (度), 0°=水平前伸, >0°=上仰
//   _J3: 相对俯仰角 (度), 0°=Link3与Link2共线, >0°=Link3相对Link2上仰
//   _J4: 相对俯仰角 (度), 0°=Link4与Link3共线, >0°=Link4相对Link3上仰
//   _J5: roll 角 (度), 0°=腕部竖直
//   _J6: 相对俯仰角 (度), 0°=Wrist与Link4共线, >0°=Wrist相对Link4上仰

static inline float MDH2Gravity_J2(float mdh_j2) {
    // MDH_J2 = motor_raw + 164° (k=1,b=164)
    // 绝对俯仰 = motor_raw (假设 motor_raw=0 时水平前伸)
    return mdh_j2;
}
static inline float MDH2Gravity_J3(float mdh_j3) {
    // 相对俯仰 = Link3绝对 - Link2绝对
    // MDH_J3 = motor_raw (k=1,b=0)
    // (TODO: 若 MDH 角即为两连杆间相对角, 则直通; 否则需换算)
    return -mdh_j3;
}
static inline float MDH2Gravity_J4(float mdh_j4) {
    // 相对俯仰 = Link4绝对 - Link3绝对
    // MDH_J4 = -motor_raw (k=-1, 方向反转)
    return -mdh_j4 - 90.0f;
}
static inline float MDH2Gravity_J5(float mdh_j5) {
    // roll 角, 0°=腕部竖直. MDH_J5 = -motor_raw (k=-1, 方向反转)
    return -mdh_j5;
}
static inline float MDH2Gravity_J6(float mdh_j6) {
    // 相对俯仰 = Wrist绝对 - Link4绝对. MDH_J6 = -motor_raw (k=-1, 方向反转)
    return -mdh_j6;
}

// ── 惯量补偿用角度 ─────────────────────────────────
// InertiaCompension_Calculate 期望 (与重力补偿完全一致):
//   _J2: 绝对俯仰角 (度), 0°=水平前伸, >0°=上仰
//   _J3: 相对俯仰角 (度), 0°=Link3与Link2共线, >0°=Link3相对Link2上仰
//   _J4: 相对俯仰角 (度), 0°=Link4与Link3共线, >0°=Link4相对Link3上仰
//   _J5: roll 角 (度), 0°=腕部竖直
//   _J6: 相对俯仰角 (度), 0°=Wrist与Link4共线, >0°=Wrist相对Link4上仰

static inline float MDH2Inertia_J2(float mdh_j2) {
    // 同重力: 绝对俯仰 = MDH - 164° (假设 motor_raw=0 时水平前伸)
    return mdh_j2;
}
static inline float MDH2Inertia_J3(float mdh_j3) {
    // 相对俯仰 = Link3绝对 - Link2绝对. MDH_J3 = motor_raw (k=1,b=0)
    // (TODO: 若 MDH 角即为两连杆间相对角, 则直通; 否则需换算)
    return -mdh_j3;
}
static inline float MDH2Inertia_J4(float mdh_j4) {
    // 相对俯仰 = Link4绝对 - Link3绝对. MDH_J4 = -motor_raw (k=-1, 方向反转)
    return -mdh_j4 - 90.0f;
}
static inline float MDH2Inertia_J5(float mdh_j5) {
    // roll 角, 同重力
    return -mdh_j5;
}
static inline float MDH2Inertia_J6(float mdh_j6) {
    // 相对俯仰 = Wrist绝对 - Link4绝对. MDH_J6 = -motor_raw (k=-1, 方向反转)
    return -mdh_j6;
}

/**
 * @brief <freertos> 电机控制任务
 */
void tskMotor(void *arg)
{
	/*	pre load for task	*/
	Motor_CAN_COB Tx_Buff_DJI_Can1 = {};

	drive_dm_motor[0].bindTxQueueHandle(CAN1_TxPort);
	drive_dm_motor[1].bindTxQueueHandle(CAN1_TxPort);
	drive_dm_motor[2].bindTxQueueHandle(CAN1_TxPort);

	drive_dm_motor[3].bindTxQueueHandle(CAN2_TxPort);
	drive_dm_motor[4].bindTxQueueHandle(CAN2_TxPort);
	drive_dm_motor[5].bindTxQueueHandle(CAN2_TxPort);
	drive_dm_motor[6].bindTxQueueHandle(CAN2_TxPort);
	
	motor[0].setPidParameters({.v_Kp = 4.3, .v_Min = -25, .v_Max = 25, .a_Kp = 0.47, .a_Ki = 0.03, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[1].setPidParameters({.v_Kp = 6.3, .v_Min = -30, .v_Max = 30, .a_Kp = 0.26, .a_Ki = 0.01, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[2].setPidParameters({.v_Kp = 7, .v_Min = -25, .v_Max = 25, .a_Kp = 0.9, .a_Ki = 0.04, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 5});
	motor[3].setPidParameters({.v_Kp = 2, .v_Min = -20, .v_Max = 20, .a_Kp = 1.4, .a_Ki = 0.03, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[4].setPidParameters({.v_Kp = 1.2, .v_Min = -20, .v_Max = 20, .a_Kp = 0.75, .a_Ki = 0.01, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[5].setPidParameters({.v_Kp = 1.5, .v_Min = -20, .v_Max = 20, .a_Kp = 0.8, .a_Ki = 0.02, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[6].setPidParameters({.v_Kp = 0.5, .v_Min = -20, .v_Max = 20, .a_Kp = 0.95, .a_Ki = 0, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});

	vTaskDelay(1000);

	Gimbal_Lift.setPidParameters({.v_Kp = 15, .v_Min = -10000, .v_Max = 10000, .a_Kp = 8, .a_Ki = 0.3, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 2});

	// CheckMotorStatus() 会自动清错、启动故障 DM，并只隔离对应电机；
	// 遥控链路建立且退出 Lost 后，才解除由链路/Lost 触发的全臂失能。

	DM4340P.setEncoderOffset(0x7348);
	DM8009.setEncoderOffset(0x7370);
	DM4340_1.setEncoderOffset(0x4F6A);
	DM4340_2.setEncoderOffset(0x9ECF);
	DM4310_1.setEncoderOffset(0x720F);
	DM4310_2.setEncoderOffset(0x723C);
	DM4310_3.setEncoderOffset(0x7516);
	// DM8009.setEncoderOffset(0x7872);
	// DM10010.setEncoderOffset(0x60E7);
	// DM4340.setEncoderOffset(0x64FF);
	// DM4310.setEncoderOffset(0x9326);

	angle_kp = 0.0f;
    angle_ki = 0.0f;
    angle_kd = 0.0f;
    angle_Out_Max = 20.0f;
    angle_I_Max = 0.0f;
    speed_kp = 0.0f;
    speed_Out_Max = 20.0f;

    // angle_kp1 = 10.0f;
    // angle_ki1 = 0.0f;
    // angle_kd1 = 0.0f;
    // angle_Out_Max1 = 6000.0f;
    // angle_I_Max1 = 100.0f;
    // speed_kp1 = 10.0f;
    // speed_Out_Max1 = 10000.0f;

 
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	uint8_t last_cmd_chassis_state = 0U;
	bool last_cmd_chassis_state_init = false;
	float chassis_height_hold = chassis_height_min;
	bool chassis_height_hold_valid = false;
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 1);

		current_chassis_height = constrain(reBottomdata.chassis_height,
												chassis_height_min,
												chassis_height_max);

		if (!last_cmd_chassis_state_init)
		{
			last_cmd_chassis_state = toBottomdata.ChassisState;
			last_cmd_chassis_state_init = true;
		}

		// Remember height when leaving ChassisState=2 and restore on return.
		if (last_cmd_chassis_state == 2 && toBottomdata.ChassisState == 0)
		{
			chassis_height_hold = current_chassis_height;
			chassis_height_hold_valid = true;
		}
		else if (last_cmd_chassis_state == 0 && toBottomdata.ChassisState == 2 && chassis_height_hold_valid)
		{
			target_chassis_height = chassis_height_hold;
			target_height_initialized = true;
			align_to_target_active = true;
		}
		last_cmd_chassis_state = toBottomdata.ChassisState;

		if (!target_height_initialized)
		{
			target_chassis_height = current_chassis_height;
			target_height_initialized = true;
		}

		// Use commanded state here to avoid clearing target height before feedback state updates.
		if (toBottomdata.ChassisState != 2)
		{
			target_chassis_height = current_chassis_height;
			align_to_target_active = false;
		}

		UpdateTargetChassisHeight();

		toBottomdata.chassis_control_struct.chassis_height = constrain(target_chassis_height - 0.000f,
																			  chassis_height_min,
																			  chassis_height_max);

		for (uint8_t i = 0; i < 7; i++)
		{
			motor[i].update(); // 更新电机数据
		}

		Gimbal_Lift.update(); // 更新云台升降电机数据
		Gimbal_Lift_UpdateTiltProtection();

		if (ArmControlLinkEstablished() &&
			(g_top_state_id != TopStateId::Lost) &&
			(g_top_state_id != TopStateId::None))
		{
			Gimbal_Lift_Init(); // 云台升降电机力矩复位标定

			// 电机任务在输出前再次执行倾角保护，覆盖不持续下发升降目标的状态。
			if (gimbal_lift_init_done && gimbal_lift_tilt_retract_active)
			{
				Gimbal_Lift_StepToGoal(gimbal_lift_min_angle, 20.0f);
			}
		}

		Joint_limitSet();
		CheckMotorStatus();

		gripper.update(); // 更新夹爪数据
		gripper.updateControl(); // 更新夹爪控制逻辑
		
#if ENABLE_J1_INERTIA_VALIDATION
		if (!ArmSafety_IsLatched() && DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
		{
			if (motor[0].getStatus() == MotorStatus::ready)
			{
			// J1惯量验证模式：虚拟弹簧法纯力矩控制，不走J1角度/速度PID。
			float j1_validation_cmd = 0.0f;
			float j1_validation_inertia_total = j1_validation_inertia_total_debug;
			float j1_validation_k_dynamic = j1_validation_k_dynamic_debug;
			float j1_validation_theta_center = j1_validation_theta_center_debug;
			float j1_validation_period = j1_validation_period_debug;
			uint8_t recenter_req = (j1_validation_recenter_req_debug != 0.0f) ? 1U : 0U;
			InertiaCompension_ValidationStep(joint[0].getCurrentAngle(), joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), joint[3].getCurrentAngle(), joint[4].getCurrentAngle(), joint[5].getCurrentAngle(), motor[0].getSpeed(), j1_validation_spring_k_debug, j1_validation_torque_limit_debug, j1_validation_speed_limit_debug, recenter_req, &j1_validation_cmd, &j1_validation_inertia_total, &j1_validation_k_dynamic, &j1_validation_theta_center, &j1_validation_period);
			j1_validation_cmd_debug = j1_validation_cmd;
			j1_validation_inertia_total_debug = j1_validation_inertia_total;
			j1_validation_k_dynamic_debug = j1_validation_k_dynamic;
			j1_validation_theta_center_debug = j1_validation_theta_center;
			j1_validation_period_debug = j1_validation_period;
			j1_validation_recenter_req_debug = 0.0f;
			joint[0].setMode(JointMode::torque);
			motor[0].forceSetMode(MotorMode::torque);
			motor[0].setTarget(j1_validation_cmd);
			// 验证时关闭J1惯量前馈，避免混入原补偿路径。
			motor[0].setInertia_feedforward(0.0f);
			}
		}
#endif

			// J1 角加速度估计: 每周期调用一次 (static 状态, 不能重入)
			const float alpha_j1 = EstimateJ1Acceleration(f_omega1.f(motor[0].getSpeed()));

		if (ArmSafety_IsLatched())
		{
			// 安全锁存期间所有 DM 电机只允许下发零力矩。
			for (int i = 0; i < 7; i++)
			{
				motor[i].execute();
			}
		}
		else
		{
			// 加上全补偿
			// 故障电机保留最后一次有效关节数据，其他电机继续正常更新和控制。
			for(int i = 0; i < 7; i++)
			{
				if (motor[i].getStatus() == MotorStatus::ready)
				{
					joint[i].updateJointData();
				}
			}

			// DM电机无需复位，Robot_Prepared默认为1，补偿始终启用
			// DR16_Debug_Ctrl();
			// UpdateMotor0PidByPose();

				// ============================================================
				// 7-DOF 重力补偿: J2(肩), J3(肘), J4(肘后), J6(腕)
				// J5 角度用于调制 J6 的有效俯仰分量 (cos φ₅)
				// MDH 关节角 → 补偿角度: 见 MDH2Gravity_xxx() 函数
				// ============================================================
				GravityCompension_Calculate(
				    MDH2Gravity_J2(joint[1].getCurrentAngle()),
				    MDH2Gravity_J3(joint[2].getCurrentAngle()),
				    MDH2Gravity_J4(joint[3].getCurrentAngle()),
				    MDH2Gravity_J5(joint[4].getCurrentAngle()),
				    MDH2Gravity_J6(joint[5].getCurrentAngle()),
				    &Gravity_feedforward[1], &Gravity_feedforward[2],
				    &Gravity_feedforward[3], &Gravity_feedforward[4], &Gravity_feedforward[5]);
				motor[1].setGravity_feedforward(Gravity_feedforward[1] * gravity_J2);
				motor[2].setGravity_feedforward(Gravity_feedforward[2] * gravity_J3);
				motor[3].setGravity_feedforward(Gravity_feedforward[3] * gravity_J4);
				motor[4].setGravity_feedforward(Gravity_feedforward[4] * gravity_J5);
				motor[5].setGravity_feedforward(Gravity_feedforward[5] * gravity_J6);

				// ============================================================
				// 7-DOF J1 惯量补偿: 姿态 (J2~J6) 决定等效惯量 Ieq
				// 加速度优先取轨迹规划目标值 (开环,无正反馈), 无轨迹时回退速度微分
				// ============================================================
				{
					// 优先: 轨迹规划目标的 J1 角加速度 (度/s² → rad/s², k=1)
					const float a_traj = joint[0].getTrajectoryAcceleration(HAL_GetTick() + 1);
					const float alpha_use = (fabsf(a_traj) > 1e-6f)
						? a_traj * (3.14159265f / 180.0f)  // 轨迹有效, 用开环目标
						: alpha_j1;                         // 无轨迹, 回退速度微分

					InertiaCompension_Calculate(
					    MDH2Inertia_J2(joint[1].getCurrentAngle()),
					    MDH2Inertia_J3(joint[2].getCurrentAngle()),
					    MDH2Inertia_J4(joint[3].getCurrentAngle()),
					    MDH2Inertia_J5(joint[4].getCurrentAngle()),
					    MDH2Inertia_J6(joint[5].getCurrentAngle()),
					    alpha_use, f_torque1.f(motor[0].v_pid.Out),
					    &Inertia_feedforward[0]);
				}
				// 惯量前馈力矩 = Ieq · α_J1, 乘以调参增益后注入 motor[0]
				motor[0].setInertia_feedforward(Inertia_feedforward[0] * inertia_J1);

			BackEMFCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &BackEMF_feedforward[0], &BackEMF_feedforward[1], &BackEMF_feedforward[2], &BackEMF_feedforward[3], &BackEMF_feedforward[4], &BackEMF_feedforward[5]);

			CoulombCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &Coulomb_feedforward[0], &Coulomb_feedforward[1], &Coulomb_feedforward[2], &Coulomb_feedforward[3], &Coulomb_feedforward[4], &Coulomb_feedforward[5]);

			ViscousCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &Viscous_feedforward[0], &Viscous_feedforward[1], &Viscous_feedforward[2], &Viscous_feedforward[3], &Viscous_feedforward[4], &Viscous_feedforward[5]);
			
			for(int i = 0; i < 7; i++)
			{
				motor[i].setBackEMF_feedforward(BackEMF_feedforward[i]);
				motor[i].setCoulomb_feedforward(Coulomb_feedforward[i]);
				motor[i].setViscous_feedforward(Viscous_feedforward[i]);
			}

			motor[0].execute();
			motor[1].execute();
			motor[2].execute();
			motor[3].execute();
			motor[4].execute();
			motor[5].execute();
			motor[6].execute();
		}

		Gimbal_Lift.execute();
		gripper.executeOutput(); // 夹爪执行输出

		motor_dji::MotorMsgPack(Tx_Buff_DJI_Can1, DJI2006);
		xQueueSend(CAN1_TxPort, &Tx_Buff_DJI_Can1.Id200, 0);

	}
}

/**
 * @brief 板间通信任务
 */
void tskBoardCom(void *arg)
{

	TickType_t xLastWakeTime_t;
    
	uint8_t debug_computerCOm = 100;
	uint16_t seq_num = 0;
	ToVisionDataStruct vision_latest_frame = {};
	uint8_t vision_pending_valid = 0;
	uint8_t last_cdc_open = 1U;
	TickType_t cdc_last_restart_tick = 0U;
	const TickType_t cdc_restart_interval = pdMS_TO_TICKS(200);
	Vec3 bottom_latest_speed = {0.0f, 0.0f, 0.0f};
	Vec3 bottom_pending_speed = {0.0f, 0.0f, 0.0f};
	uint8_t bottom_pending_valid = 0U;
	uint8_t bottom_period_tick = 0U;
	TickType_t vision_busy_hold_ticks = 0U;
	const TickType_t vision_busy_backoff_ticks = pdMS_TO_TICKS(2);
	xLastWakeTime_t = xTaskGetTickCount();
	vTaskDelay(1000);
	/* UI mode trigger: when VT13 Q pressed, set UI_Mode=1 for next 5 cycles */
#if USE_SRML_VT13
	static uint8_t ui_mode_count = 0U;
	static uint8_t last_vt13_q_state = 0U;
#endif
	for (;;)
	{
		/* CRC, length validation and frame reassembly run in task context. */
		(void)Visiondata_ProcessRx();

		/*发送给底层*/
		// 底盘发送信息处理
		// 与底盘通信测试
		// DR16_MoveCtrl(toBottomdata);
		// DR16_Store(toBottomdata);
		bottom_latest_speed = {
			.x = toBottomdata.chassis_control_struct.x,
			.y = toBottomdata.chassis_control_struct.y,
			.z = toBottomdata.chassis_control_struct.z};

		/* VT13 Q key: 当检测到按下边沿时，触发接下来的5个周期 UI 模式为 1 */
// #if USE_SRML_VT13
// 		uint8_t cur_q = VT13.IsKeyPress(VT13_KEY_Q) ? 1U : 0U;

// 		/* 检测按下边沿 */
// 		if (cur_q && !last_vt13_q_state)
// 		{
// 			ui_mode_count = 5U; // 包含本周期在内的5个周期，每周期1ms，共5ms
// 		}

// 		/* 更新UI_Mode状态 */
// 		if (ui_mode_count > 0U)
// 		{
// 			UI_Mode = 1U;
// 			ui_mode_count--;
// 		}
// 		else
// 		{
// 			UI_Mode = 0U;
// 		}

// 		last_vt13_q_state = cur_q;
// #endif

		bottom_period_tick++;
		if ((bottom_period_tick >= 3U) || (bottom_pending_valid != 0U))
		{
			if (bottom_period_tick >= 3U)
			{
				bottom_period_tick = 0U;
			}

			bottom_pending_speed = bottom_latest_speed;
			bottom_pending_valid = 1U;

			if (Bottomdata_Transmit(bottom_pending_speed.x, bottom_pending_speed.y, bottom_pending_speed.z) != 0U)
			{
				bottom_pending_valid = 0U;
			}
		}

		/*接受底层信息*/

		/*发送给小电脑*/
		if (Robot_Prepared)
		{
			DTR_state = CDC_IsPortOpen();
			if (CDC_IsPortOpen() == 0U)
			{
				// if (last_cdc_open != 0U)
				// {
				// 	cdc_last_restart_tick = xTaskGetTickCount();
				// 	last_cdc_open = 0U;
				// }
				// else if ((xTaskGetTickCount() - cdc_last_restart_tick) >= cdc_restart_interval)
				// {
				// 	VirtualComRestart();
				// 	cdc_last_restart_tick = xTaskGetTickCount();
				// }

				vision_pending_valid = 0U;
				vision_busy_hold_ticks = 0U;
			}
			else
				{
				// last_cdc_open = 1U;
				if (vision_busy_hold_ticks > 0U)
				{
					vision_busy_hold_ticks--;
				}

				if (vision_busy_hold_ticks == 0U)
				{
					for (int i = 0; i < 7; i++)
					{
						vision_latest_frame.joint_current[i] = joint[i].getCurrentAngle();
						vision_latest_frame.torque_current[i] = joint[i].getCurrentTorque();
					}
					vision_latest_frame.chassis_height_current = current_chassis_height + 0.000f;
					vision_latest_frame.seq_num = seq_num++;
					vision_latest_frame.comm_flag = 0U;

					if (vision_pending_valid == 0U)
					{
						memcpy(&toVisiondata, &vision_latest_frame, sizeof(toVisiondata));
						vision_pending_valid = 1U;
					}

					if (vision_pending_valid != 0U)
					{
						if (Visiondata_IsTxBusy() != 0U)
						{
							vision_busy_hold_ticks = vision_busy_backoff_ticks;
						}
						else
						{
							tx_result = Visiondata_Transmist();
							if (tx_result == USBD_OK)
							{
								if (toVisiondata.seq_num != vision_latest_frame.seq_num)
								{
									memcpy(&toVisiondata, &vision_latest_frame, sizeof(toVisiondata));
									vision_pending_valid = 1U;
								}
								else
								{
									vision_pending_valid = 0U;
								}
							}
							else if (tx_result == USBD_BUSY)
							{
								vision_busy_hold_ticks = vision_busy_backoff_ticks;
							}
							else
							{
								memcpy(&toVisiondata, &vision_latest_frame, sizeof(toVisiondata));
								vision_pending_valid = 1U;
							}
						}
					}
				}
			}
					
		}
		else
		{
			vision_pending_valid = 0U;
			vision_busy_hold_ticks = 0U;
		}
		
		// toVisiondata.timestamp = HAL_GetTick();
		// Visiondata_Transmist();	
	
		/*接受小电脑信息*/
		vTaskDelayUntil(&xLastWakeTime_t, 1);
	}
}

#if USE_SRML_MPU6050
/**
 * @brief MPU6050读取数据
 */
void tskIMU(void *arg)
{
	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 2);
		/*	读取MPU6050数据	*/
		vTaskSuspendAll();		  // 挂起其他任务，防止被打断
		taskDISABLE_INTERRUPTS(); // 关闭中断，若使用中断关闭，请确保SRML定时器的中断不受FreeRTOS管辖
		dmp_read_data(&mpu_receive);
		taskENABLE_INTERRUPTS();
		xTaskResumeAll();
	}
}
#endif

#if USE_SRML_LOG
/**
 *	@brief	Log data transmit task
 */
void tskLog(void *arg)
{
	/*Pre-Load for task*/
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, (pdMS_TO_TICKS(100)));
		SRML_LOGI("MCU", "test log from [%s]", __func__); // 测试Log功能，你也可以在其他任务中使用Log
	}
}
#endif

#if USE_SRML_DR16
/**
 *	@brief	Dr16 data receive task
 */
void tskDR16(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	DR16.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		xSemaphoreTake(DR16_mutex, portMAX_DELAY);
		
		/*	等待数据	*/
		if (xQueueReceive(DR16_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Read Message
			DR16.DataCapture((DR16_DataPack_Typedef *)Rx_Package.address);
		}
		/*	检测遥控器连接 */
		DR16.Check_Link(xTaskGetTickCount());

		if (DR16.GetStatus() != DR16_ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */
			xSemaphoreGive(DR16_mutex);
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
		xSemaphoreGive(DR16_mutex);
	}
}
#endif

#if USE_SRML_VT13
/**
 * @brief  VT13接收任务
 * @param  None.
 * @return None.
 */
void tskVT13(void *arg)
{
	/* Cache for Task */
	USART_COB Rx_Package;
	/* Pre-Load for task */
	VT13.Check_Link(xTaskGetTickCount());
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		// 若要设置进入临界区，请确认好各个遥控器（倘若同时使用两个遥控器）的优先级，防止进入死锁
		xSemaphoreTake(VT13_mutex, portMAX_DELAY);
		/*	等待数据	*/
		if (xQueueReceive(VT13_QueueHandle, &Rx_Package, 100) == pdPASS)
		{
			// Receive Message
			VT13.Receive(Rx_Package.address, Rx_Package.len);
		}

		/*	检测遥控器连接 */
		VT13.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (VT13.GetStatus() != VT13_ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */
			xSemaphoreGive(VT13_mutex);
			continue;
		}
		/*	更新遥控器控制	*/
		VT13_ProcessButtons();
		/* Leave critical */
		xSemaphoreGive(VT13_mutex);
	}
}
#endif

#if USE_SRML_REFEREE
/**
 * @brief  接受裁判系统数据
 * @param  None.
 * @return None.
 */
void Rx_Referee(void *arg)
{
	/* Preoad for task */
	USART_COB *referee_pack;
	TickType_t xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{
		if (xTaskNotifyWait(0x00000000, 0xFFFFFFFF, (uint32_t *)&referee_pack, portMAX_DELAY) == pdTRUE)
		{
			Referee.unPackDataFromRF((uint8_t *)referee_pack->address, referee_pack->len);
		}
	}
}

// 需要1字节对齐
#pragma pack(1)
typedef struct
{
	/*机械臂*/
	float J1_angle; // J1关节角度，单位：度
	float J2_angle; // J2关节角度，单位：？
	float J3_angle; // J3关节角度，单位：？
	float J4_angle; // J4关节角度，单位：？
	float J5_angle; // J5关节角度，单位：？
	float J6_angle; // J6关节角度，单位：？
	float J7_angle; // J7关节角度，单位：？

	/*四轮腿*/
	float LF_pos_1; // 左前大腿位置 单位：m
	float LF_pos_2; // 左前小腿位置 单位：？
	float RF_pos_1; // 右前大腿位置 单位：？
	float RF_pos_2; // 右前小腿位置 单位：？
	float LB_pos_1; // 左后大腿位置 单位：？
	float LB_pos_2; // 左后小腿位置 单位：？
	float RB_pos_1; // 右后大腿位置 单位：？
	float RB_pos_2; // 右后小腿位置 单位：？

	float chassis_pitch;
	float chassis_roll;
	float chassis_height; // 底盘高度，单位：m

	float RFID_length; // RFID伸缩长度，单位：m
	
	uint8_t RobotState;   // 0：失能  1：机动  2：复位  3：兑矿  4：急停复位
	uint8_t ArmCtrlState; // 0：动作组  1：自定义控制器  2：视觉
	uint8_t ChassisState; // 0: 失能  1：正常  2：兑矿  3：前腿  4：后腿

	uint8_t EnergyUnit_index; // 能量单元索引

	uint8_t MotorHighTempNum; // 高温电机数量
	uint8_t OverTempFlag;     // 是否触发电机过温失能


} __attribute__((packed)) self_engineer_data;
#pragma pack()

self_engineer_data engineer_data_2_cc;

/**
 * @brief  接受裁判系统数据
 * @param  None.
 * @return None.
 */
void Sx_Referee(void *arg)
{
	/* Preoad for task */
	TickType_t xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	for (;;)
	{

		engineer_data_2_cc.J1_angle = joint[0].getCurrentAngle();
		engineer_data_2_cc.J2_angle = joint[1].getCurrentAngle();
		engineer_data_2_cc.J3_angle = joint[2].getCurrentAngle();
		engineer_data_2_cc.J4_angle = joint[3].getCurrentAngle();
		engineer_data_2_cc.J5_angle = joint[4].getCurrentAngle();
		engineer_data_2_cc.J6_angle = joint[5].getCurrentAngle();
		engineer_data_2_cc.J7_angle = joint[6].getCurrentAngle();

		engineer_data_2_cc.LF_pos_1 = reBottomdata.EndPosition[0];
		engineer_data_2_cc.LF_pos_2 = reBottomdata.EndPosition[4];
		engineer_data_2_cc.RF_pos_1 = reBottomdata.EndPosition[1];
		engineer_data_2_cc.RF_pos_2 = reBottomdata.EndPosition[5];
		engineer_data_2_cc.LB_pos_1 = reBottomdata.EndPosition[2];
		engineer_data_2_cc.LB_pos_2 = reBottomdata.EndPosition[6];
		engineer_data_2_cc.RB_pos_1 = reBottomdata.EndPosition[3];
		engineer_data_2_cc.RB_pos_2 = reBottomdata.EndPosition[7];

		engineer_data_2_cc.chassis_pitch = reBottomdata.chassis_angle[0];
		engineer_data_2_cc.chassis_roll = reBottomdata.chassis_angle[1];
		engineer_data_2_cc.chassis_height = current_chassis_height;
		engineer_data_2_cc.RFID_length = reBottomdata.rfid_height;
		engineer_data_2_cc.ArmCtrlState = ArmMode;
		engineer_data_2_cc.ChassisState = reBottomdata.ChassisState;

		switch (g_top_state_id)
		{
		case TopStateId::Mobile:
			engineer_data_2_cc.RobotState = 1;
			break;
		case TopStateId::Lost:
			engineer_data_2_cc.RobotState = 0;
			break;
		case TopStateId::GetEnergyUnit:
			engineer_data_2_cc.RobotState = 2;
			break;
		default:
			break;
		}

		engineer_data_2_cc.EnergyUnit_index = action_group_controller.getSelectedOre();

		engineer_data_2_cc.MotorHighTempNum = reBottomdata.HighTempMotorNum;
		engineer_data_2_cc.OverTempFlag = reBottomdata.OverTempFlag;

		// 具备自定义数据功能，如工程
		Referee.CC_data_input(CC_ID_ENGINEER_V2, (uint8_t *)&engineer_data_2_cc, sizeof(engineer_data_2_cc), CC_MESSAGE_SELF, 0);

		// 自定义控制器数据 (0x0309, 10Hz): 7 个关节角, 50Hz 每 5 次发一次
		static uint8_t cc_ctrl_cnt = 0;
		if (++cc_ctrl_cnt >= 5)
		{
			cc_ctrl_cnt = 0;
			float joint_angle[7];
			uint8_t cc_ctrl_data[30] = {0};
			for (int i = 0; i < 7; i++)
				joint_angle[i] = joint[i].getCurrentAngle();
			memcpy(cc_ctrl_data, joint_angle, sizeof(joint_angle));
			Referee.CC_controller_data_send(cc_ctrl_data, sizeof(cc_ctrl_data));
		}

		vTaskDelayUntil(&xLastWakeTime_t, pdMS_TO_TICKS(20)); // 50Hz 发送
	}
}

#endif

#if USE_SRML_FS_I6X
/**
 *	@brief	FS_I6X data receive task
 */
void tskFS_I6X(void *arg)
{
	/* Cache for Task */
	/* Pre-Load for task */
	/* Infinite loop */
	for (;;)
	{
		/* Enter critical */
		/*	等待数据	*/
		if (ulTaskNotifyTake(pdTRUE, 100) != 0)
		{
			// Read Message
			remote.DataProcess();
		}
		/*	检测遥控器连接 */
		remote.Check_Link(xTaskGetTickCount());
		/*	判断是否连接 	 */
		if (remote.GetStatus() != ESTABLISHED)
		{
			/**
			 * lost the remote control
			 */

			/* Leave critical */;
			continue;
		}
		/*	更新遥控器控制	*/

		/* Leave critical */
	}
}
#endif // !USE_SRML_FS_I6X

#if USE_SRML_BMI088
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case BMI088_ACC_INT_Pin:
		/** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
		// BMI088.AccReadOut();
		// BMI088_AHRS.feed_acc(BMI088.AccData);
		// BMI088_AHRS.doAcc();
		BMI088_AccHandler();
		break;
	case BMI088_GYRO_INT_Pin:
		/** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
		// BMI088.GyroReadout();
		// BMI088_AHRS.feed_gyro(BMI088.GyroData);
		// BMI088_AHRS.doGYro();
		BMI088_GyroHandler();
		break;
	default:
		break;
	}
}
#endif
