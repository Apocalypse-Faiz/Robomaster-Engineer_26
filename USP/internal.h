 /*
 * internal.h
 *
 *  Created on: Apr 7, 2021
 *      Author: M3chD09
 */

#ifndef _INTERNAL_H_
#define _INTERNAL_H_

#include "can.h"
#include "usart.h"
#include "tim.h"
#include "spi.h"

#include "SRML.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "Motor.h"
#include "DM_motor.h"
#include "DJI_motor.h"
#include "Special_Actuator1.h"
#include "Special_Actuator2.h"
#include "Special_Actuator3.h"
#include "JointActuator.h"
#include "Joint.h"
#include "Trajectory.h"
#include "cqc.h"
#include "Compension.h"
#include "ChassisCom.h"
#include "VisualCom.h"
#include "Claw.h"
#include "WS2812.h"
#include "Indicator.h"

#include "SEGGER_SYSVIEW.h"

#if USE_SRML_LOG
#include "Drivers/Devices/Log/inc/srml_log.h"
#include "Drivers/Devices/Log/inc/srml_utils.h"
#include "Drivers/Devices/Log/utils/log_send.h"
#endif

#if USE_SRML_BMI088  
/** TODO:  待BMI088库测试完善，加入srml.h目录里*/
/** TODO:  25赛季完全删除注释部分，停止对老版BMI解算的支持*/
//#include "../USP/BMIDriver/Mahony.hpp"
//#include "../USP/BMIDriver/Drv_BMI088.hpp"
#endif
#ifdef __cplusplus
extern "C" {
#endif

/* Macro Definitions ---------------------------------------------------------*/
#define Tiny_Stack_Size       64
#define Small_Stack_Size      128
#define Normal_Stack_Size     256
#define Large_Stack_Size      512
#define Huge_Stack_Size       1024
	
#define PriorityVeryLow       1
#define PriorityLow           2
#define PriorityBelowNormal   3
#define PriorityNormal        4
#define PriorityAboveNormal   5
#define PriorityHigh          6
#define PrioritySuperHigh     7
#define PriorityRealtime      8
/* HAL Handlers --------------------------------------------------------------*/
/* RTOS Resources ------------------------------------------------------------*/
/* Task */
extern TaskHandle_t Rx_Referee_Handle;
#if USE_SRML_FS_I6X
extern TaskHandle_t FS_I6X_Handle;
#endif // !USE_SRML_FS_I6X
/* Queues */
extern QueueHandle_t USART_TxPort;
extern QueueHandle_t USART_RxPort;
extern QueueHandle_t CAN1_TxPort;				
extern QueueHandle_t CAN1_RxPort;		
extern QueueHandle_t CAN2_TxPort;
extern QueueHandle_t CAN2_RxPort;

#if USE_SRML_DR16
extern QueueHandle_t DR16_QueueHandle;
#endif

#if USE_SRML_VT13
    extern QueueHandle_t VT13_QueueHandle;
#endif
/* Semaphores */
/* mutex */
#if USE_SRML_DR16
extern SemaphoreHandle_t DR16_mutex;
#endif

#if USE_SRML_VT13
    extern SemaphoreHandle_t VT13_mutex;
#endif
/* Mutexes */
/* Notifications */
/* Other Resources -----------------------------------------------------------*/
#if USE_SRML_VIRTUAL_COM
extern uint8_t VirtualCom_Rx_Buff[VIRTUALCOM_RX_BUFFER_SIZE]; // 虚拟串口缓存区
#endif

#if  USE_SRML_MPU6050
extern mpu_rec_s mpu_receive; //mpu6050传感器
#endif

#if USE_SRML_BMI088
/** TODO:  25赛季完全删除，停止对老版BMI解算的支持*/
//extern BMI088SensorClassdef BMI088;
//extern MahonyFilterClassdef BMI088_AHRS;
#endif

#if  USE_SRML_DR16
extern DR16_Classdef DR16;		//遥控器DR16类
#endif

#if USE_SRML_VT13
    extern VT13_Classdef VT13; // VT13遥控器类
#endif

#if USE_SRML_FS_I6X
extern FS_I6X_Classdef remote;
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
extern referee_Classdef Referee;//裁判系统类
#endif

extern int debug_can_over1;
extern int debug_can_over2;
extern int debug_uart_over;
extern float gravity_J2;
extern float gravity_J3;
extern float gravity_J5;
extern float inertia_J1;

extern Motor_DM_classdef DM8009; // 达妙电机类，ID为1
extern Motor_DM_classdef DM10010; // 达妙电机类，ID为2
extern Motor_DM_classdef DM4340; // 达妙电机类，ID为3
extern Motor_DM_classdef DM4310; // 达妙电机类，ID为4
extern Motor_C610 Dji2006_1;
extern Motor_C610 Dji2006_2;

extern DM_Motor drive_dm_motor[4]; // 达妙电机对象数组
extern DJI_Motor_2006 drive_dji_motor_2006[2]; // 大疆2006电机对象数组
extern Servo_Classdef servo_gripper; // 夹爪舵机对象

extern Motor motor[6]; // 电机对象数组
extern Special_Actuator2 special_actuator2; // 特殊执行器2对象对象
extern Special_Actuator3 special_actuator3[4]; // 特殊执行器3对象数组
extern JointActuator joint_actuator[5]; // 关节执行器对象数组
extern TrajectoryPlanning joint_trajectory[6]; // 关节轨迹规划对象数组
extern TrajectoryPointArray joint_trajectory_point_array[6]; // 关节轨迹点对象数组
extern Joint joint[6]; // 关节对象数组
extern Gripper gripper; // 夹爪对象

extern float Gravity_feedforward[6]; // 重力前馈数组
extern float BackEMF_feedforward[6]; // 反电动势前馈数组
extern float Coulomb_feedforward[6]; // 库仑前馈数组
extern float Viscous_feedforward[6]; // 黏性前馈数组
extern float Inertia_feedforward[6]; // 惯量补偿数组

extern bool all_motor_connected; // 所有电机连接状态
extern uint8_t OreBit[2];                                                    // 初始矿石位为空，0为空，1为有矿，OreBit[0]为左矿石位，OreBit[1]为右矿石位
extern uint8_t UI_Mode;          // UI模式
extern uint8_t Robot_Prepared; // 机器人准备状态，0：未准备好，1：准备好

extern float chassis_height_min;
extern float chassis_height_max;
extern float chassis_height_step;
extern float current_chassis_height;
extern float target_chassis_height;
extern bool target_height_initialized;
extern bool align_to_target_active;

extern ToBottomDataStructdef toBottomdata; // 发送到底盘的数据结构体
extern ReBottomDataStructdef reBottomdata; // 从底盘接收的数据结构体
extern ReBottomDataStructdef temp_reBottomdata; // 作为Buffer的底盘数据结构体
extern ToVisionDataStruct toVisiondata;
extern ReVisionDataStruct reVisiondata;
extern Indicator_Classdef Indicator;     // 指示灯

extern float speed_kp, speed_Out_Max, angle_kp, angle_ki, angle_kd, angle_Out_Max, angle_I_Max, debug_target_angle;
extern float speed_kp1, speed_Out_Max1, angle_kp1, angle_ki1, angle_kd1, angle_Out_Max1, angle_I_Max1, debug_target_angle1;

extern MeanFilter<10> mfsL1, mfsL2;
extern MeanFilter<50> f_torque1, f_omega1;
extern MeanFilter<50> mfsJ5, mfsJ6;
extern float j5_reset_target, j6_reset_target;

extern TaskHandle_t StateMachine_Handle; // 状态机任务

/* Exported function declarations --------------------------------------------*/
void Service_Debug_Init(void);
void Service_Communication_Init(void);
void Service_Devices_Init(void);

void User_CAN1_RxCpltCallback(CAN_COB *CAN_RxCOB);
void User_CAN2_RxCpltCallback(CAN_COB *CAN_RxCOB);

extern User_Uart_Callback UART1_RxCpltCallback;
extern User_Uart_Callback UART2_RxCpltCallback;
extern User_Uart_Callback UART3_RxCpltCallback;
extern User_Uart_Callback UART4_RxCpltCallback;
extern User_Uart_Callback UART5_RxCpltCallback;
extern User_Uart_Callback UART6_RxCpltCallback;

#if USE_SRML_DR16
uint32_t DR16_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#if USE_SRML_VT13
    uint32_t VT13_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif // !USE_SRML_VT13

#if USE_SRML_FS_I6X
uint32_t FS_I6X_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
uint32_t Referee_recv_Callback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#if USE_SRML_VIRTUAL_COM
void User_VirtualComRecCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);
#endif

#ifdef __cplusplus
}
#endif

#endif /* _INTERNAL_H_ */
