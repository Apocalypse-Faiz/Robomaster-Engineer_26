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
	vTaskDelayUntil(&xLastWakeTime_t, 50);
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
		// 末端夹爪状态观察
		if (gripper.isObjectGripped())
		{
			WS_indicator[1] = WS_White;
		}
		else
		{
			WS_indicator[1] = WS_Black;
		}
		// 储矿夹爪状态观察
		if (reBottomdata.L_Ore == 1)
		{
			WS_indicator[5] = WS_White;
		}
		else
		{
			WS_indicator[5] = WS_Black;
		}

		if (reBottomdata.R_Ore == 1)
		{
			WS_indicator[4] = WS_White;
		}
		else
		{
			WS_indicator[4] = WS_Black;
		}

		// 电机状态观察
		if (DM8009.GetMotorState() != 0 || DM4310.GetMotorState() != 0)
		{
			for (int i = 0; i < 6; i++)
			{
				WS_indicator[i] = WS_Red;
			}
		}

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

	}
}

void CheckMotorStatus()
{
		for (uint8_t i = 0; i < 6; i++)
		{
			if (motor[i].getStatus() == MotorStatus::ready)
			{
				// 状态正常时设置为angle模式
				motor[i].setMode(MotorMode::angle);
			}
		}
	
	if(motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready && motor[6].getStatus() == MotorStatus::ready)
	{
		all_motor_connected = true;
	}
	else
	{
		all_motor_connected = false;
	}

	// return all_motor_connected;

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
		for (uint8_t i = 0; i < 6; i++)
		{
			joint[i].updateJointData();
		}

		// J3的动态限位
		if (joint[1].getCurrentAngle() <= 90)
		{
			joint[2].setJointParam(-80, 60, -10, 10);
		}
		else if (joint[1].getCurrentAngle() > 90 && joint[1].getCurrentAngle() <= 180)
		{
			joint[2].setJointParam(-(180 - joint[1].getCurrentAngle() + 50 -90), 60, -10, 10);
		}

		return;
	}

	float Reduction_ratio1 = 0.0667 * 0.0526; // 1/15 DJI3508的减速比
	float Reduction_ratio2 = 0.0278 * 0.5; // 1/36 DJI2006的减速比

	if (motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready)
	{

		// 以下设置映射中的偏置b是通过DM电机返回的多圈角度进行各种判断设置对应偏置（单圈内不同地方上电，同一地方反馈角度不同），实际正确方法应该是通过单圈角度来设置偏置，希望新赛季队员能够进行优化。

		// 只有初次上电的时候才会设置，复位之前暂时使用这些初始值，复位状态机中重新设置
		// 对于DM电机初次上电时在这里就已经设置好了，DJI电机需要在复位状态机中设置
		/*标定和极性设置*/
		joint[0].setJointMapping(-1, 0);
		joint[1].setJointMapping(-1, 0);
		joint[2].setJointMapping(-1, 0);
		joint[3].setJointMapping(-1, 0);
		joint[4].setJointMapping(Reduction_ratio2, 0);
		joint[5].setJointMapping(-Reduction_ratio2, 0);

		special_actuator3[0].calibrate();
		special_actuator3[1].calibrate();
		special_actuator3[2].calibrate();
		special_actuator3[3].calibrate();

		/*关节软限位*/
		joint[0].setJointParam(-170, 170, 10, -10);
		joint[1].setJointParam(10, 164, -10, 10);
		joint[2].setJointParam(-180, 180, -10, 10);
		joint[3].setJointParam(-180, 180, -10, 10);
		joint[4].setJointParam(-360, 360, -10, 10); // J5关节在力矩复位后还要重新再限位一下
		joint[5].setJointParam(-360, 360, -10, 10);

		gripper.init(5000.0f, 4500.0f, 1800.0f, Reduction_ratio2, 0, 90.0f, -90.0f);

		Set_ok = 1;
	}
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

	motor[0].setPidParameters({.v_Kp = 0, .v_Min = -10, .v_Max = 10, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -10, .a_Max = 10, .a_IntSep = 0, .a_IntMax = 2});
	motor[1].setPidParameters({.v_Kp = 0, .v_Min = -15000, .v_Max = 15000, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 10});
	motor[2].setPidParameters({.v_Kp = 0, .v_Min = -15000, .v_Max = 15000, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 10});
	motor[3].setPidParameters({.v_Kp = 0, .v_Min = -10, .v_Max = 10, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -20, .a_Max = 20, .a_IntSep = 0, .a_IntMax = 2});
	motor[4].setPidParameters({.v_Kp = 0.0, .v_Min = -10000, .v_Max = 10000, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 100});
	motor[5].setPidParameters({.v_Kp = 0.0, .v_Min = -10000, .v_Max = 10000, .a_Kp = 0, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 100});
	motor_gripper.setPidParameters({.v_Kp = 10.0, .v_Min = -10000, .v_Max = 10000, .a_Kp = 7.5, .a_Ki = 0, .a_Kd = 0, .a_Min = -6000, .a_Max = 6000, .a_IntSep = 0, .a_IntMax = 100});

	vTaskDelay(1000);
	for(int i = 0; i < 4; i++)
	{
		drive_dm_motor[i].enableMotor();
	}

	// DM8009.setEncoderOffset(0x6D6A);
	// DM4310.setEncoderOffset(0x7C25);

	angle_kp = 0.11f;
    angle_ki = 0.01f;
    angle_kd = 0.0f;
    angle_Out_Max = 10.0f;
    angle_I_Max = 2.0f;
    speed_kp = 4.2f;
    speed_Out_Max = 10.0f;

    // angle_kp1 = 7.5f;
    // angle_ki1 = 1.0f;
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

		for (uint8_t i = 0; i < 6; i++)
		{
			motor[i].update(); // 更新电机数据
		}

		gripper.update(); // 更新夹爪数据
		gripper.updateControl(); // 更新夹爪控制逻辑
		
		// CheckMotorConnect();

		Joint_limitSet();
		
		if(all_motor_connected)
		{
			// 加上全补偿
			// 注意：当代码在某处进程中断停止运行，motor[i].getStatus()会变为error，导致在使用motor[i].getAngle()等函数时卡在死循环中！！！
			for(int i = 0; i < 6; i++)
			{
				joint[i].updateJointData();
			}


			if (Robot_Prepared)
			{
				// DR16_Debug_Ctrl();
				GravityCompension_Calculate(joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), &Gravity_feedforward[1], &Gravity_feedforward[2]);
				motor[1].setGravity_feedforward(Gravity_feedforward[1] * gravity_J2);
				motor[2].setGravity_feedforward(Gravity_feedforward[2] * gravity_J3);

				InertiaCompension_Calculate(joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), f_torque1.f(motor[0].v_pid.Out), &Inertia_feedforward[0]);
				motor[0].setInertia_feedforward(Inertia_feedforward[0] * inertia_J1);

				GripperGravityCompension_Calculate(joint[4].getCurrentAngle(), &Gravity_feedforward[4]);
				motor[4].setGravity_feedforward(Gravity_feedforward[4] * gravity_J5);
			}

			BackEMFCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &BackEMF_feedforward[0], &BackEMF_feedforward[1], &BackEMF_feedforward[2], &BackEMF_feedforward[3], &BackEMF_feedforward[4], &BackEMF_feedforward[5]);

			CoulombCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &Coulomb_feedforward[0], &Coulomb_feedforward[1], &Coulomb_feedforward[2], &Coulomb_feedforward[3], &Coulomb_feedforward[4], &Coulomb_feedforward[5]);

			ViscousCompension_Calculate(motor[0].getSpeed(), motor[1].getSpeed(), motor[2].getSpeed(), motor[3].getSpeed(), motor[4].getSpeed(), motor[5].getSpeed(), &Viscous_feedforward[0], &Viscous_feedforward[1], &Viscous_feedforward[2], &Viscous_feedforward[3], &Viscous_feedforward[4], &Viscous_feedforward[5]);
			
			for(int i = 0; i < 6; i++)
			{
				motor[i].setBackEMF_feedforward(BackEMF_feedforward[i]);
				motor[i].setCoulomb_feedforward(Coulomb_feedforward[i]);
				motor[i].setViscous_feedforward(Viscous_feedforward[i]);
			}


		}
		else
		{
			
			for (int i = 0; i < 2; i++)
			{
				drive_dji_motor_2006[i].setTorque(0.0f);
			}
			for (int i = 0; i < 4; i++)
			{
				drive_dm_motor[i].setTorque(0.0f);
			}

			CheckMotorStatus();
		}
  
		motor[0].execute();
		motor[1].execute();
		motor[2].execute();
		motor[3].execute();
		motor[4].execute();
		motor[5].execute();
		gripper.executeOutput(); // 夹爪执行输出

		//can test
		// DM8009.setTorque(0.0f);
		// DM4310.setTorque(0.0f);
		// Dji3508_1.Out = 0.0f;

		// Dji3508_2.Out = 0.0f;
		// Dji2006_1.Out = 0.0f;
		// Dji2006_2.Out = 0.0f;
		// Dji2006_gripper.Out = 0.0f;

		motor_dji::MotorMsgPack(Tx_Buff_DJI_Can2, Dji2006_1, Dji2006_2, Dji2006_gripper);
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
			}
			vision_latest_frame.seq_num = seq_num++;
			vision_latest_frame.timestamp = HAL_GetTick();

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