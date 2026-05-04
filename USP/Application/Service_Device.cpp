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

/* Private define ------------------------------------------------------------*/
#define ENABLE_J1_INERTIA_VALIDATION 0

/* Private variables ---------------------------------------------------------*/
TaskHandle_t Motor_Handle;
TaskHandle_t IMU_Handle;
TaskHandle_t DR16_Handle;
TaskHandle_t VT13_Handle; // VT13遥控器任务，如果不用了可以删除掉
TaskHandle_t FS_I6X_Handle;
TaskHandle_t Rx_Referee_Handle;
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
#endif
/* Private function declarations ---------------------------------------------*/
void tskMotor(void *arg);
void tskFS_I6X(void *arg);
void tskIMU(void *arg);
void tskDR16(void *arg);
void tskVT13(void *arg); // VT13遥控器任务，如果不用了可以删除掉
void Rx_Referee(void *arg);
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
	xTaskCreate(tskMotor, "App.Motor", Huge_Stack_Size + Small_Stack_Size, NULL, PrioritySuperHigh, &Motor_Handle);
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
	xTaskCreate(Rx_Referee, "Rx_Referee", Normal_Stack_Size, NULL, PriorityNormal, &Rx_Referee_Handle);
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
	static uint8_t dm_offline_latched[4] = {0U, 0U, 0U, 0U};
	static uint8_t ready_init_done[6] = {0U};
	static uint8_t mode_init_done = 0U;
	static uint8_t dm_recover_pending = 0U;
	static const uint8_t DM_MOTOR_COUNT = 4U;
	static const uint8_t ALL_READY_STABLE_TICKS = 20U;
	const TickType_t DM_RECONNECT_PERIOD_TICK = pdMS_TO_TICKS(100);
#if ENABLE_J1_INERTIA_VALIDATION
	const uint8_t mode_set_start = 1U;
#else
	const uint8_t mode_set_start = 0U;
#endif

	bool all_ready = true;
	bool dm_has_disconnect = false;

	for (uint8_t i = 0; i < 6; i++)
	{
		if (motor[i].getStatus() != MotorStatus::ready)
		{
			all_ready = false;
			ready_init_done[i] = 0U;
			if (i < DM_MOTOR_COUNT)
			{
				dm_has_disconnect = true;
				if (dm_offline_latched[i] == 0U)
				{
					motor[i].forceSetMode(MotorMode::disabled);
					joint[i].setMode(JointMode::disabled);
					dm_offline_latched[i] = 1U;
				}
			}
		}
		else if (i < DM_MOTOR_COUNT)
		{
			dm_offline_latched[i] = 0U;
		}
		else if (ready_init_done[i] == 0U && i >= mode_set_start)
		{
			motor[i].setMode(MotorMode::angle);
			joint[i].setMode(JointMode::angle);
			ready_init_done[i] = 1U;
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
				for (uint8_t i = mode_set_start; i < 6; i++)
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
		if (motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready)
		{
			joint[1].updateJointData();
			float j2_angle = joint[1].getCurrentAngle();
			if (j2_angle <= 90)
			{
				joint[2].setJointParam(-50, 50, -10, 10);
			}
			else if (j2_angle <= 180 && j2_angle > 90)
			{
				joint[2].setJointParam(-(180 - j2_angle + 50 - 90), 50, -10, 10);
			}
		}

		return;
	}

	float Reduction_ratio = 0.0278 * 0.5; // 1/36 DJI2006的减速比再加上传动
	float k1 = 6.28 / 12.5;

	if (motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready)
	{

		// 以下设置映射中的偏置b是通过DM电机返回的多圈角度进行各种判断设置对应偏置（单圈内不同地方上电，同一地方反馈角度不同），实际正确方法应该是通过单圈角度来设置偏置，希望新赛季队员能够进行优化。

		// 只有初次上电的时候才会设置，复位之前暂时使用这些初始值，复位状态机中重新设置
		// 对于DM电机初次上电时在这里就已经设置好了，DJI电机需要在复位状态机中设置
		/*标定和极性设置*/
		joint[0].setJointMapping(k1, 0);
		joint[1].setJointMapping(1, 164);
		joint[2].setJointMapping(1, 0);
		joint[3].setJointMapping(-1, 0);
		joint[4].setJointMapping(-Reduction_ratio, 0);
		joint[5].setJointMapping(Reduction_ratio, 0);

		special_actuator3[0].calibrate();
		special_actuator3[1].calibrate();
		special_actuator3[2].calibrate();
		special_actuator3[3].calibrate();

		/*关节软限位*/
		joint[0].setJointParam(-170, 170, 10, -10);
		joint[1].setJointParam(10, 160, -10, 10);	
		joint[2].setJointParam(-180, 180, -10, 10);
		joint[3].setJointParam(-180, 180, -10, 10);
		joint[4].setJointParam(-360, 360, -10, 10); // J5关节在力矩复位后还要重新再限位一下
		joint[5].setJointParam(-360, 360, -10, 10);

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

/**
 * @brief <freertos> 电机控制任务
 */
void tskMotor(void *arg)
{
	/*	pre load for task	*/
	Motor_CAN_COB Tx_Buff_DJI_Can2 = {};

	drive_dm_motor[0].bindTxQueueHandle(CAN1_TxPort);
	drive_dm_motor[1].bindTxQueueHandle(CAN1_TxPort);
	drive_dm_motor[2].bindTxQueueHandle(CAN1_TxPort);
	drive_dm_motor[3].bindTxQueueHandle(CAN1_TxPort);

	motor[0].setPidParameters({.v_Kp = 0, .v_Min = -20, .v_Max = 20, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -10, .a_Max = 10, .a_IntSep = 0, .a_IntMax = 2});
	motor[1].setPidParameters({.v_Kp = 2.4, .v_Min = -10, .v_Max = 10, .a_Kp = 0.3, .a_Ki = 0.02, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[2].setPidParameters({.v_Kp = 3.5f, .v_Min = -20, .v_Max = 20, .a_Kp = 1.25f, .a_Ki = 0.15f, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 5});
	motor[3].setPidParameters({.v_Kp = 1.9f, .v_Min = -10, .v_Max = 10, .a_Kp = 0.726f, .a_Ki = 0.04f, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[4].setPidParameters({.v_Kp = 10.0, .v_Min = -10000, .v_Max = 10000, .a_Kp = 10, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 100});
	motor[5].setPidParameters({.v_Kp = 10.0, .v_Min = -10000, .v_Max = 10000, .a_Kp = 10, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 100});

	vTaskDelay(1000);
	for(int i = 0; i < 4; i++)
	{
		drive_dm_motor[i].enableMotor();
	}

	DM8009.setEncoderOffset(0x6CEE);
	DM10010.setEncoderOffset(0x60E7);
	DM4340.setEncoderOffset(0x64FF);
	DM4310.setEncoderOffset(0x9326);

	angle_kp = 0.1f;
    angle_ki = 0.0f;
    angle_kd = 0.0f;
    angle_Out_Max = 20.0f;
    angle_I_Max = 2.0f;
    speed_kp = 1.0f;
    speed_Out_Max = 10.0f;

    // angle_kp1 = 10.0f;
    // angle_ki1 = 0.0f;
    // angle_kd1 = 0.0f;
    // angle_Out_Max1 = 6000.0f;
    // angle_I_Max1 = 100.0f;
    // speed_kp1 = 10.0f;
    // speed_Out_Max1 = 10000.0f;

 
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();
	for (;;)
	{
		/* wait for next circle */
		vTaskDelayUntil(&xLastWakeTime_t, 1);

		current_chassis_height = constrain(reBottomdata.chassis_height,
												chassis_height_min,
												chassis_height_max);

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

		for (uint8_t i = 0; i < 6; i++)
		{
			motor[i].update(); // 更新电机数据
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
			uint8_t recenter_req = (j1_validation_recenter_req_debug != 0.0f) ? 1U : 0U;
			InertiaCompension_ValidationStep(joint[0].getCurrentAngle(), joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), motor[0].getSpeed(), j1_validation_spring_k_debug, j1_validation_torque_limit_debug, j1_validation_speed_limit_debug, 0.001f, recenter_req, &j1_validation_cmd, &j1_validation_inertia_total, &j1_validation_k_dynamic, &j1_validation_theta_center);
			j1_validation_cmd_debug = j1_validation_cmd;
			j1_validation_inertia_total_debug = j1_validation_inertia_total;
			j1_validation_k_dynamic_debug = j1_validation_k_dynamic;
			j1_validation_theta_center_debug = j1_validation_theta_center;
			j1_validation_recenter_req_debug = 0.0f;
			joint[0].setMode(JointMode::torque);
			motor[0].forceSetMode(MotorMode::torque);
			motor[0].setTarget(j1_validation_cmd);
			// 验证时关闭J1惯量前馈，避免混入原补偿路径。
			motor[0].setInertia_feedforward(0.0f);
			}
		}
#endif
		
		if(all_motor_connected)
		{
			// 加上全补偿
			// 注意：当代码在某处进程中断停止运行，motor[i].getStatus()会变为error，导致在使用motor[i].getAngle()等函数时卡在死循环中！！！
			for(int i = 0; i < 6; i++)
			{
				joint[i].updateJointData();
			}

			// if (Robot_Prepared)
			// {
				// DR16_Debug_Ctrl();
				UpdateMotor0PidByPose();

				GravityCompension_Calculate(joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), &Gravity_feedforward[1], &Gravity_feedforward[2]);
				motor[1].setGravity_feedforward(Gravity_feedforward[1] * gravity_J2);
				motor[2].setGravity_feedforward(Gravity_feedforward[2] * gravity_J3);

				InertiaCompension_Calculate(joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), f_torque1.f(motor[0].v_pid.Out), f_omega1.f(motor[0].getSpeed()), &Inertia_feedforward[0]);
				// 正常模式下，J1惯量补偿前馈由模型输出并乘总增益后注入。
				motor[0].setInertia_feedforward(Inertia_feedforward[0] * inertia_J1);

			// }

			BackEMFCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &BackEMF_feedforward[0], &BackEMF_feedforward[1], &BackEMF_feedforward[2], &BackEMF_feedforward[3], &BackEMF_feedforward[4], &BackEMF_feedforward[5]);

			CoulombCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &Coulomb_feedforward[0], &Coulomb_feedforward[1], &Coulomb_feedforward[2], &Coulomb_feedforward[3], &Coulomb_feedforward[4], &Coulomb_feedforward[5]);

			ViscousCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &Viscous_feedforward[0], &Viscous_feedforward[1], &Viscous_feedforward[2], &Viscous_feedforward[3], &Viscous_feedforward[4], &Viscous_feedforward[5]);
			
			for(int i = 0; i < 6; i++)
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
		}
		else
		{
			if (Robot_Prepared)
			{
				if (motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready)
				{
					joint[1].updateJointData();
					joint[2].updateJointData();
					UpdateMotor0PidByPose();

					GravityCompension_Calculate(joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), &Gravity_feedforward[1], &Gravity_feedforward[2]);
					motor[1].setGravity_feedforward(Gravity_feedforward[1] * gravity_J2);
					motor[2].setGravity_feedforward(Gravity_feedforward[2] * gravity_J3);
				}
				else
				{
					motor[1].setGravity_feedforward(0.0f);
					motor[2].setGravity_feedforward(0.0f);
				}

				if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready)
				{
					InertiaCompension_Calculate(joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), f_torque1.f(motor[0].v_pid.Out), motor[0].getSpeed(), &Inertia_feedforward[0]);
					motor[0].setInertia_feedforward(Inertia_feedforward[0] * inertia_J1);
				}
				else
				{
					motor[0].setInertia_feedforward(0.0f);
				}

			}

			for (int i = 0; i < 6; i++)
			{
				if (motor[i].getStatus() == MotorStatus::ready)
				{
					motor[i].execute();
				}
				else
				{
					if (i < 4)
					{
						drive_dm_motor[i].setTorque(0.0f);
					}
					else
					{
						drive_dji_motor_2006[i - 4].setTorque(0.0f);
					}
				}
			}
		}

		gripper.executeOutput(); // 夹爪执行输出

		motor_dji::MotorMsgPack(Tx_Buff_DJI_Can2, Dji2006_1, Dji2006_2);
		xQueueSend(CAN2_TxPort, &Tx_Buff_DJI_Can2.Id200, 0);
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
	Vec3 bottom_latest_speed = {0.0f, 0.0f, 0.0f};
	Vec3 bottom_pending_speed = {0.0f, 0.0f, 0.0f};
	uint8_t bottom_pending_valid = 0U;
	uint8_t bottom_period_tick = 0U;
	xLastWakeTime_t = xTaskGetTickCount();
	vTaskDelay(1000);
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
			for (int i = 0; i < 6; i++)
			{
				vision_latest_frame.joint_current[i] = joint[i].getCurrentAngle();
				if (i == 4)
				{
					vision_latest_frame.torque_current[i] = mfsJ5.f(joint[i].getCurrentTorque());
				}
				else if (i == 5)
				{
					vision_latest_frame.torque_current[i] = mfsJ6.f(joint[i].getCurrentTorque());
				}
				else
				{
					vision_latest_frame.torque_current[i] = joint[i].getCurrentTorque();
				}
			}
			vision_latest_frame.chassis_height_current = current_chassis_height;
			vision_latest_frame.seq_num = seq_num++;
			vision_latest_frame.comm_flag = reVisiondata.comm_flag;

			if (vision_pending_valid == 0U)
			{
				memcpy(&toVisiondata, &vision_latest_frame, sizeof(toVisiondata));
				vision_pending_valid = 1U;
			}

			if (vision_pending_valid != 0U)
			{
				// 返回USBD_BUSY则重复发送
				uint8_t tx_result = Visiondata_Transmist();
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
				else if (tx_result != USBD_BUSY)
				{
					memcpy(&toVisiondata, &vision_latest_frame, sizeof(toVisiondata));
					vision_pending_valid = 1U;
				}
			}
					
		}
		else
		{
			vision_pending_valid = 0U;
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