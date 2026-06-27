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
	xLastWakeTime_t = xTaskGetTickCount();
	Total_init();
	vTaskDelayUntil(&xLastWakeTime_t, 1000);
	for (;;)
	{
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

void CheckMotorStatus()
{
	static TickType_t last_dm_reconnect_tick = 0U;
	static uint8_t dm_reconnect_rr_idx = 0U;
	static uint8_t all_ready_stable_cnt = 0U;
	static uint8_t dm_offline_latched[7] = {0U, 0U, 0U, 0U, 0U, 0U, 0U};
	static uint8_t ready_init_done[7] = {0U};
	static uint8_t mode_init_done = 0U;
	static uint8_t dm_recover_pending = 0U;
	static const uint8_t DM_MOTOR_COUNT = 7U;
	static const uint8_t ALL_READY_STABLE_TICKS = 20U;
	const TickType_t DM_RECONNECT_PERIOD_TICK = pdMS_TO_TICKS(100);
#if ENABLE_J1_INERTIA_VALIDATION
	const uint8_t mode_set_start = 1U;
#else
	const uint8_t mode_set_start = 0U;
#endif

	bool all_ready = true;
	bool dm_has_disconnect = false;

	for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++)
	{
		if (motor[i].getStatus() != MotorStatus::ready)
		{
			all_ready = false;
			ready_init_done[i] = 0U;
			dm_has_disconnect = true;
			if (dm_offline_latched[i] == 0U)
			{
				motor[i].forceSetMode(MotorMode::disabled);
				joint[i].setMode(JointMode::disabled);
				dm_offline_latched[i] = 1U;
			}
		}
		else
		{
			dm_offline_latched[i] = 0U;
		}
	}

	if (all_ready)
	{
		if (all_ready_stable_cnt < ALL_READY_STABLE_TICKS)
		{
			all_ready_stable_cnt++;
		}

		if (all_ready_stable_cnt < ALL_READY_STABLE_TICKS)
		{
			all_motor_connected = false;
			return;
		}

		if (all_motor_connected == false)
		{
			if (mode_init_done == 0U)
			{
				for (uint8_t i = mode_set_start; i < DM_MOTOR_COUNT; i++)
				{
					if (ready_init_done[i] == 0U)
					{
						motor[i].setMode(MotorMode::angle);
						joint[i].setMode(JointMode::angle);
						ready_init_done[i] = 1U;
					}
				}
				mode_init_done = 1U;
			}
			else if (dm_recover_pending != 0U)
			{
				for (uint8_t i = mode_set_start; i < DM_MOTOR_COUNT; i++)
				{
					motor[i].setMode(MotorMode::angle);
					joint[i].setMode(JointMode::angle);
				}
				dm_recover_pending = 0U;
			}
		}
		all_motor_connected = true;
		return;
	}

	all_ready_stable_cnt = 0U;
	all_motor_connected = false;
	if (dm_has_disconnect == false)
	{
		return;
	}

	dm_recover_pending = 1U;

	TickType_t now_tick = xTaskGetTickCount();
	if ((now_tick - last_dm_reconnect_tick) < DM_RECONNECT_PERIOD_TICK)
	{
		return;
	}

	for (uint8_t offset = 0; offset < DM_MOTOR_COUNT; offset++)
	{
		uint8_t idx = (dm_reconnect_rr_idx + offset) % DM_MOTOR_COUNT;
		if (motor[idx].getStatus() != MotorStatus::ready)
		{
			drive_dm_motor[idx].enableMotor();
			dm_reconnect_rr_idx = (idx + 1U) % DM_MOTOR_COUNT;
			break;
		}
	}

	last_dm_reconnect_tick = now_tick;
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
    static const float GIMBAL_LIFT_RANGE = 24000.0f; // 标定后可用行程 (度): 0° 为限位, +RANGE 为另一端 (TODO 实测)

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
			gimbal_lift_min_angle = 100.0f;
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
        goal_deg = std_lib::constrain(goal_deg, 0.0f, gimbal_lift_max_angle);
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
 * @brief 关节角度限位设置
 * @note 需要在获取电机初始角度后，做一些判断后再设置
 */
void Joint_limitSet()
{

	/*关节角映射，和仿真对齐*/
	static uint8_t Set_ok = 0;
	if (Set_ok == 1)
	{
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
		joint[0].setJointMapping(1, 0);
		joint[1].setJointMapping(1, 158);
		joint[2].setJointMapping(1, 34);
		joint[3].setJointMapping(-1, -79);
		joint[4].setJointMapping(-1, 0);
		joint[5].setJointMapping(-1, 0);
		joint[6].setJointMapping(-1, 0);

		special_actuator3[0].calibrate();
		special_actuator3[1].calibrate();
		special_actuator3[2].calibrate();
		special_actuator3[3].calibrate();
		special_actuator3[4].calibrate();
		special_actuator3[5].calibrate();
		special_actuator3[6].calibrate();

		/*关节软限位*/
		joint[0].setJointParam(-170, 170, 10, -10);
		joint[1].setJointParam(40, 155, -10, 10);	
		joint[2].setJointParam(34, 150, -10, 10);
		joint[3].setJointParam(-77, 17, -10, 10);
		joint[4].setJointParam(-165, 165, -10, 10); // J5关节在力矩复位后还要重新再限位一下
		joint[5].setJointParam(-105, 105, -10, 10);
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
		{64.07f, 1.85f, 0.064f, 0.0f, 5.7f},
		{59.0f, 26.96f, 0.064f, 0.017f, 5.6f},
		{59.82f, -25.88f, 0.06f, 0.0f, 6.4f},
		{38.64f, -43.70f, 0.05f, 0.0f, 6.7f},
		{41.32f, -38.43f, 0.067f, 0.0f, 5.7f},
	};

	float j2_angle = joint[1].getCurrentAngle();
	float j3_angle = joint[2].getCurrentAngle();
	float a_kp = 0.0f;
	float a_ki = 0.0f;
	float v_kp = 0.0f;

	// J2大于90度时使用固定参数。
	if (j2_angle >= 90.0f)
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
	
	motor[0].setPidParameters({.v_Kp = 4.4, .v_Min = -25, .v_Max = 25, .a_Kp = 0.5, .a_Ki = 0.03, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[1].setPidParameters({.v_Kp = 6.3, .v_Min = -30, .v_Max = 30, .a_Kp = 0.26, .a_Ki = 0.01, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[2].setPidParameters({.v_Kp = 7, .v_Min = -25, .v_Max = 25, .a_Kp = 0.9, .a_Ki = 0.04, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 5});
	motor[3].setPidParameters({.v_Kp = 2, .v_Min = -20, .v_Max = 20, .a_Kp = 1.4, .a_Ki = 0.03, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[4].setPidParameters({.v_Kp = 1.2, .v_Min = -20, .v_Max = 20, .a_Kp = 0.75, .a_Ki = 0.01, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[5].setPidParameters({.v_Kp = 1.5, .v_Min = -20, .v_Max = 20, .a_Kp = 0.8, .a_Ki = 0, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[6].setPidParameters({.v_Kp = 0.5, .v_Min = -20, .v_Max = 20, .a_Kp = 0.95, .a_Ki = 0, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});

	vTaskDelay(1000);

	Gimbal_Lift.setPidParameters({.v_Kp = 15, .v_Min = -10000, .v_Max = 10000, .a_Kp = 10, .a_Ki = 0.3, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 2});

	for(int i = 0; i < 7; i++)
	{
		drive_dm_motor[i].enableMotor();
	}

	DM4340P.setEncoderOffset(0x8238);
	DM8009.setEncoderOffset(0x7370);
	DM4340_1.setEncoderOffset(0x4F6A);
	DM4340_2.setEncoderOffset(0x9ECF);
	DM4310_1.setEncoderOffset(0x720F);
	DM4310_2.setEncoderOffset(0x9D5B);
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

		toBottomdata.chassis_control_struct.chassis_height = constrain(target_chassis_height,
																		  chassis_height_min,
																		  chassis_height_max);

		for (uint8_t i = 0; i < 7; i++)
		{
			motor[i].update(); // 更新电机数据
		}

		Gimbal_Lift.update(); // 更新云台升降电机数据

		if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
		{
			Gimbal_Lift_Init(); // 云台升降电机力矩复位标定
		}

		CheckMotorStatus();

		gripper.update(); // 更新夹爪数据
		gripper.updateControl(); // 更新夹爪控制逻辑
		
		Joint_limitSet();

#if ENABLE_J1_INERTIA_VALIDATION
		if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
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

		if(all_motor_connected)
		{
			// 加上全补偿
			// 注意：当代码在某处进程中断停止运行，motor[i].getStatus()会变为error，导致在使用motor[i].getAngle()等函数时卡在死循环中！！！
			for(int i = 0; i < 7; i++)
			{
				joint[i].updateJointData();
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
		else
		{
			// DM电机无需复位，Robot_Prepared默认为1，部分补偿始终启用
			for (int i = 0; i < 7; i++)
			{
				if (motor[i].getStatus() == MotorStatus::ready)
				{
					joint[i].updateJointData();
				}
			}

			if (motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready)
			{
				// UpdateMotor0PidByPose();

				// 7-DOF 重力补偿 (部分电机就绪时仍启用, MDH→补偿角度转换)
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
			}
			else
			{
				motor[1].setGravity_feedforward(0.0f);
				motor[2].setGravity_feedforward(0.0f);
				motor[3].setGravity_feedforward(0.0f);
				motor[4].setGravity_feedforward(0.0f);
				motor[5].setGravity_feedforward(0.0f);
			}

			if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready)
			{
				// 7-DOF J1 惯量补偿 (部分电机就绪, 优先轨迹加速度/回退微分)
				{
					const float a_traj = joint[0].getTrajectoryAcceleration(HAL_GetTick() + 1);
					const float alpha_use = (fabsf(a_traj) > 1e-6f)
						? a_traj * (3.14159265f / 180.0f) : alpha_j1;

					InertiaCompension_Calculate(
					    MDH2Inertia_J2(joint[1].getCurrentAngle()),
					    MDH2Inertia_J3(joint[2].getCurrentAngle()),
					    MDH2Inertia_J4(joint[3].getCurrentAngle()),
					    MDH2Inertia_J5(joint[4].getCurrentAngle()),
					    MDH2Inertia_J6(joint[5].getCurrentAngle()),
					    alpha_use, f_torque1.f(motor[0].v_pid.Out),
					    &Inertia_feedforward[0]);
					motor[0].setInertia_feedforward(Inertia_feedforward[0] * inertia_J1);
				}
			}
			else
			{
				motor[0].setInertia_feedforward(0.0f);
			}

			for (int i = 0; i < 7; i++)
			{
				if (motor[i].getStatus() == MotorStatus::ready)
				{
					motor[i].execute();
				}
				else
				{
					drive_dm_motor[i].setTorque(0.0f);
				}
			}
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
	ToVisionDataStruct vision_latest_frame;
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
					vision_latest_frame.chassis_height_current = current_chassis_height;
					vision_latest_frame.seq_num = seq_num++;
					if (vision_comm_flag_override_valid)
					{
						vision_latest_frame.comm_flag = vision_comm_flag_override;
					}
					else
					{
						vision_latest_frame.comm_flag = 0;
					}

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

		// 不具备自定义数据功能，如步兵
		// Referee.InfantryInfo[0].gimbal_info.shooter_flag = 0;
		// Referee.CC_data_input(CC_ID_INFANTRY_3_V1, NULL, 0, CC_MESSAGE_SELF, 0);
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