/**
 **********************************************************************************
 * @file   : Service_Debug.cpp
 * @brief  : Debug support file.This file provides access ports to debug.
 **********************************************************************************
 *
 **/
/* Includes ------------------------------------------------------------------*/
#include "internal.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TaskHandle_t UpperMonitor_Handle;
TaskHandle_t VofaMonitor_Handle;
TaskHandle_t AsuwaveMonitor_Handle;
/* Private function declarations ---------------------------------------------*/
void Task_UpperMonitor(void *arg);
void Task_VofaMonitor(void *arg);
void Task_AsuwaveMonitor(void *arg);
/* Function prototypes -------------------------------------------------------*/
/**
 * @brief  Initialize debug service based on Asuwave.
 */
void Service_Debug_Init(void)
{
#if USE_SRML_UPPER_MONITOR
	xTaskCreate(Task_UpperMonitor, "tskUpperMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &UpperMonitor_Handle);
#endif

#if USE_SRML_VOFA_MONITOR
	xTaskCreate(Task_VofaMonitor, "tskVofaMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &VofaMonitor_Handle);
#endif

#if USE_SRML_ASUWAVE_MONITOR
	xTaskCreate(Task_AsuwaveMonitor, "tskAsuwaveMonitor", Small_Stack_Size + Tiny_Stack_Size, NULL, PriorityRealtime, &AsuwaveMonitor_Handle);		
#endif
}

#if USE_SRML_UPPER_MONITOR
void Task_UpperMonitor(void *arg)
{
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	UpperMonitor::init(4);// 开启上位机修改变量功能，与发送无关，不使用也能照常发送
	// 分配编号给被修改的变量
	// UpperMonitor::bind_Modified_Var(0, ...);
	// UpperMonitor::bind_Modified_Var(1, ...);
	// UpperMonitor::bind_Modified_Var(2, ...);
	// UpperMonitor::bind_Modified_Var(3, ...);
	// UpperMonitor::bind_Modified_Var(4, ...);

	/* Infinite loop */
	for (;;)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		/* 在此处传入需要观察的变量，第一个参数为通道的起始编号 */
		// UpperMonitor::setDatas(0, mpu_receive.pitch, mpu_receive.yaw, mpu_receive.roll);
		//UpperMonitor::setDatas(3, data3, data4, data5);
		//UpperMonitor::setDatas(6, data6, data7, data8, data9);
		/* 选择串口id */
		UpperMonitor::send(4);
	}
}
#endif /* USE_SRML_UPEER_MONITOR */

#if USE_SRML_VOFA_MONITOR
void Task_VofaMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	/* Infinite loop */
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		/* 在此处传入需要观察的变量，第一个参数为通道的起始编号 */
		// VofaMonitor::setDatas(0, mpu_receive.pitch, mpu_receive.yaw, mpu_receive.roll);
		//VofaMonitor::setDatas(3, data3, data4, data5);
		//VofaMonitor::setDatas(6, data6, data7, data8, data9);

		// VofaMonitor::setDatas(0, toBottomdata.chassis_control_struct.x, toBottomdata.chassis_control_struct.y, toBottomdata.chassis_control_struct.z, reBottomdata.L_gripper, reBottomdata.R_gripper);
		// VofaMonitor::setDatas(0, reVisiondata.seq_num, 
		// 	reVisiondata.joint_target[0], 
		// 	toVisiondata.joint_current[0],
		// 	reVisiondata.joint_target[1], 
		// 	toVisiondata.joint_current[1],
		// 	reVisiondata.joint_target[2], 
		// 	toVisiondata.joint_current[2],
		// 	reVisiondata.joint_target[3], 
		// 	reVisiondata.joint_target[4], 
		// 	reVisiondata.joint_target[5],
		// 	vision_crc_stats.total_received,
		// 	vision_crc_stats.crc_errors,
		// 	vision_crc_stats.error_rate);

		// VofaMonitor::setDatas(0, Gimbal_Lift.getAngle(), Gimbal_Lift.target_angle, Gimbal_Lift.getSpeed(), Gimbal_Lift.a_pid.Out, Gimbal_Lift.v_pid.Out, Gimbal_Lift.getTorque());

		if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready && motor[6].getStatus() == MotorStatus::ready)
		{
			for (uint8_t i = 0; i < 7; i++)
			{
				joint[i].updateJointData();
			}

			// VofaMonitor::setDatas(0, toBottomdata.ChassisState, reBottomdata.ChassisState);
			// VofaMonitor::setDatas(0, joint[0].data.target_angle, joint[1].data.target_angle, joint[2].data.target_angle, joint[3].data.target_angle, joint[4].data.target_angle, joint[5].data.target_angle, joint[6].data.target_angle,joint[0].getCurrentAngle(), joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), joint[3].getCurrentAngle(), joint[4].getCurrentAngle(), joint[5].getCurrentAngle(), joint[6].getCurrentAngle());
			// VofaMonitor::setDatas(0, VT13.Datapack_custom.joint_angle[0], VT13.Datapack_custom.joint_angle[1], VT13.Datapack_custom.joint_angle[2], VT13.Datapack_custom.joint_angle[3], VT13.Datapack_custom.joint_angle[4], VT13.Datapack_custom.joint_angle[5], VT13.Datapack_custom.joint_angle[6]);
			// VofaMonitor::setDatas(0, joint[0].data.target_angle, joint[1].data.target_angle, joint[2].data.target_angle, joint[3].data.target_angle, joint[4].data.target_angle, joint[5].data.target_angle, joint[6].data.target_angle);
			VofaMonitor::setDatas(0, joint[0].getCurrentTorque(), joint[1].getCurrentTorque(), joint[2].getCurrentTorque(), joint[3].getCurrentTorque(), joint[4].getCurrentTorque(), joint[5].getCurrentTorque(), joint[6].getCurrentTorque(), joint[0].getCurrentAngle(), joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), joint[3].getCurrentAngle(), joint[4].getCurrentAngle(), joint[5].getCurrentAngle(), joint[6].getCurrentAngle());
			// VofaMonitor::setDatas(0, joint[0].getCurrentAngle(), joint[1].getCurrentAngle(), joint[2].getCurrentAngle(), joint[3].getCurrentAngle(), joint[4].getCurrentAngle(), joint[5].getCurrentAngle(), current_chassis_height, joint[0].getCurrentTorque(),joint[1].getCurrentTorque(), joint[2].getCurrentTorque());
			// VofaMonitor::setDatas(0, joint[0].getCurrentAngle(), joint[0].getCurrentTorque(), motor[0].Inertia_feedforward);
			// VofaMonitor::setDatas(0, debug_target_angle, 
			// 	joint[5].data.target_angle,
			// 	joint[5].data.current_angle,
			// 	joint[5].data.current_speed,
			// 	motor[5].getAngle(),
			// 	motor[5].getSpeed(),
			// 	motor[5].a_pid.Out,
			// 	motor[5].a_pid.I_Term, 
			// 	motor[5].v_pid.Out,
			// 	motor[5].getTorque());

				// VofaMonitor::setDatas(0, debug_target_angle, 
				// joint[4].data.target_angle,
				// joint[4].data.current_angle,
				// joint[4].data.current_speed,
				// motor[4].getAngle(),
				// motor[4].getSpeed(),
				// motor[4].a_pid.Out,
				// motor[4].a_pid.I_Term, 
				// motor[4].v_pid.Out);

				// VofaMonitor::setDatas(0, debug_target_angle, 
				// 	joint[3].data.target_angle,
				// 	joint[3].data.current_angle,
				// 	joint[3].data.current_speed,
				// 	motor[3].getAngle(),
				// 	motor[3].getSpeed(),
				// 	motor[3].a_pid.Out,
				// 	motor[3].a_pid.I_Term, 
				// 	motor[3].v_pid.Out);

				// VofaMonitor::setDatas(0, debug_target_angle, 
				// 	joint[6].data.target_angle,
				// 	joint[6].data.current_angle,
				// 	joint[6].data.current_speed,
				// 	motor[6].getAngle(),
				// 	motor[6].getSpeed(),
				// 	motor[6].a_pid.Out,
				// 	motor[6].a_pid.I_Term, 
				// 	motor[6].v_pid.Out);

				// VofaMonitor::setDatas(0, debug_target_angle, 
				// 	joint[2].data.target_angle,
				// 	joint[2].data.current_angle,
				// 	joint[2].data.current_speed,
				// 	motor[2].getAngle(),
				// 	motor[2].getSpeed(),
				// 	motor[2].a_pid.Out,
				// 	motor[2].a_pid.I_Term, 
				// 	motor[2].v_pid.Out);

				// VofaMonitor::setDatas(0, debug_target_angle, 
				// 	joint[1].data.target_angle,
				// 	joint[1].data.current_angle,
				// 	joint[1].data.current_speed,
				// 	motor[1].getAngle(),
				// 	motor[1].getSpeed(),
				// 	motor[1].a_pid.Out,
				// 	motor[1].a_pid.I_Term, 
				// 	motor[1].v_pid.Out);

				// VofaMonitor::setDatas(0, debug_target_angle, 
				// 	joint[0].data.target_angle,
				// 	joint[0].data.current_angle,
				// 	joint[0].data.current_speed,
				// 	motor[0].getAngle(),
				// 	motor[0].getSpeed(),
				// 	motor[0].a_pid.Out,
				// 	motor[0].a_pid.I_Term, 
				// 	motor[0].v_pid.Out,
				// 	Inertia_feedforward[0]);

				// VofaMonitor::setDatas(0, f_speed2.f(motor[1].getSpeed()), f_torque2.f(motor[1].getTorque() - motor[1].Gravity_feedforward), f_speed3.f(motor[2].getSpeed()), f_torque3.f(motor[2].getTorque() - motor[2].Gravity_feedforward));

				// VofaMonitor::setDatas(0, abs(motor[5].getTorque()), abs(motor[2].getTorque()), abs(motor[1].getTorque()),
				// mfsL1.f(abs(motor[5].getTorque())), mfsL2.f(abs(motor[2].getTorque())), mfsL3.f(abs(motor[1].getTorque())));

				// VofaMonitor::setDatas(0, motor[4].getAngle(),motor[4].target_angle, motor[5].getAngle(), motor[5].target_angle, joint[4].getCurrentAngle(), joint[5].getCurrentAngle(), joint[4].getTargetAngle(), joint[5].getTargetAngle());

				
		}
		
		/* 选择串口id */
		VofaMonitor::send(5);
	}
}
#endif	/* USE_SRML_VOFA_MONITOR */

#if USE_SRML_ASUWAVE_MONITOR	
void Task_AsuwaveMonitor(void *arg){
	/* Cache for Task */

	/* Pre-Load for task */
	TickType_t xLastWakeTime_t;
	xLastWakeTime_t = xTaskGetTickCount();

	AsuwaveMonitor::init(1, xTaskGetTickCount);

	/* Infinite loop */
	while(1)
	{
		/* Wait for the next cycle */
		vTaskDelayUntil(&xLastWakeTime_t, 5);

		AsuwaveMonitor::send(); // 发送数据给上位机
	}
}
#endif /* USE_SRML_ASUWAVE_MONITOR */
/************************ COPYRIGHT SCUT-ROBOTLAB *****END OF FILE*************/
