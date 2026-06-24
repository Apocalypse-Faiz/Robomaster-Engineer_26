/**
 ******************************************************************************
 * @file   System_DataPool.cpp
 * @brief  All used resources are contained in this file.
 ******************************************************************************
 * @note
 *  - User can define datas including variables ,structs ,and arrays in
 *    this file, which are used in deffrient tasks or services.
 **/
#include "internal.h"

/* RTOS Resources ------------------------------------------------------------*/
/* Queues */
QueueHandle_t USART_TxPort; //	串口发送队列
QueueHandle_t USART_RxPort; // 串口接收队列
QueueHandle_t CAN1_TxPort;  //	can1 发送队列
QueueHandle_t CAN1_RxPort;  //	can1 接收队列
QueueHandle_t CAN2_TxPort;  //	can2 发送队列
QueueHandle_t CAN2_RxPort;  //	can2 接收队列
#if USE_SRML_DR16
QueueHandle_t DR16_QueueHandle; //	dr16（串口） 接收队列
#endif

#if USE_SRML_VT13
QueueHandle_t VT13_QueueHandle; // VT13接收队列
#endif
/* Semaphores */

/* Mutexes */
#if USE_SRML_DR16
SemaphoreHandle_t DR16_mutex; //	dr16互斥量
#endif

#if USE_SRML_VT13
SemaphoreHandle_t VT13_mutex;
#endif
/* Notifications */

/* Other Resources -----------------------------------------------------------*/
#if USE_SRML_VIRTUAL_COM
uint8_t VirtualCom_Rx_Buff[VIRTUALCOM_RX_BUFFER_SIZE];
#endif

#if USE_SRML_MPU6050
__CCM mpu_rec_s mpu_receive; // mpu6050数据
#endif

#if USE_SRML_DR16
__CCM DR16_Classdef DR16; // 遥控器DR16类
#endif

#if USE_SRML_VT13
__CCM VT13_Classdef VT13; // VT13遥控器类
#endif

#if USE_SRML_FS_I6X
__CCM FS_I6X_Classdef remote;
#endif // !USE_SRML_FS_I6X

#if USE_SRML_REFEREE
__SRAM referee_Classdef Referee;
SemaphoreHandle_t Referee_VT_mutex; //图传链路缓冲区 写锁
#endif
#if USE_SRML_BMI088
/** TODO:  25赛季完全删除，停止对第二代解算库支持*/
// /** BMI088传感器驱动模块 */
// BMI088SensorClassdef BMI088 = BMI088SensorClassdef(&hspi1, BMI088_ACC_NSS_GPIO_Port, BMI088_ACC_NSS_Pin, BMI088_GYRO_NSS_GPIO_Port, BMI088_GYRO_NSS_Pin);
// /** BMI088解算模块 */
// MahonyFilterClassdef BMI088_AHRS;
#endif

int debug_can_over1 = 0;
int debug_can_over2 = 0;
int debug_uart_over = 0;

Motor_DM_classdef DM4340P(3); // 达妙电机类，ID为3
Motor_DM_classdef DM8009(2); // 达妙电机类，ID为2
Motor_DM_classdef DM4340_1(1); // 达妙电机类，ID为1

Motor_DM_classdef DM4340_2(1); // 达妙电机类，ID为1
Motor_DM_classdef DM4310_1(2); // 达妙电机类，ID为2
Motor_DM_classdef DM4310_2(3); // 达妙电机类，ID为4
Motor_DM_classdef DM4310_3(4); // 达妙电机类，ID为4

DM_Motor drive_dm_motor[7] = {DM_Motor(&DM4340P), DM_Motor(&DM8009), DM_Motor(&DM4340_1), DM_Motor(&DM4340_2), DM_Motor(&DM4310_1), DM_Motor(&DM4310_2), DM_Motor(&DM4310_3)}; // 达妙电机对象数组

Motor motor[7] = {
    Motor(&drive_dm_motor[0]),
    Motor(&drive_dm_motor[1]),
    Motor(&drive_dm_motor[2]),
    Motor(&drive_dm_motor[3]),
    Motor(&drive_dm_motor[4]),
    Motor(&drive_dm_motor[5]),
    Motor(&drive_dm_motor[6])}; // 电机对象数组

Servo_Classdef servo_gripper(&htim2, TIM_CHANNEL_2, 20000, 0.025f, 0.125f, 360); // 夹爪舵机对象
Gripper gripper(&servo_gripper); // 夹爪对象

Special_Actuator2 special_actuator2(&motor[4], &motor[5]); // 双电机特殊执行器对象

Special_Actuator3 special_actuator3[7] = {
    Special_Actuator3(&motor[0]),
    Special_Actuator3(&motor[1]),
    Special_Actuator3(&motor[2]),
    Special_Actuator3(&motor[3]),
    Special_Actuator3(&motor[4]),
    Special_Actuator3(&motor[5]),
    Special_Actuator3(&motor[6])
};

JointActuator joint_actuator[7] = {
    JointActuator(&special_actuator3[0]),
    JointActuator(&special_actuator3[1]),
    JointActuator(&special_actuator3[2]),
    JointActuator(&special_actuator3[3]),
    JointActuator(&special_actuator3[4]),
    JointActuator(&special_actuator3[5]),
    JointActuator(&special_actuator3[6])}; // 关节执行器对象数组

TrajectoryPlanning joint_trajectory[7] = {
    TrajectoryPlanning(new CQC()),
    TrajectoryPlanning(new CQC()),
    TrajectoryPlanning(new CQC()),
    TrajectoryPlanning(new CQC()),
    TrajectoryPlanning(new CQC()),
    TrajectoryPlanning(new CQC()),
    TrajectoryPlanning(new CQC())
}; // 关节轨迹规划对象数组

TrajectoryPointArray joint_trajectory_point_array[7] = {
    TrajectoryPointArray(50),
    TrajectoryPointArray(50),
    TrajectoryPointArray(50),
    TrajectoryPointArray(50),
    TrajectoryPointArray(50),
    TrajectoryPointArray(50),
    TrajectoryPointArray(50)
}; // 关节轨迹点对象数组

Joint joint[7] = {
    Joint(&joint_actuator[0], &joint_trajectory[0], 0),
    Joint(&joint_actuator[1], &joint_trajectory[1], 0),
    Joint(&joint_actuator[2], &joint_trajectory[2], 0),
    Joint(&joint_actuator[3], &joint_trajectory[3], 0),
    Joint(&joint_actuator[4], &joint_trajectory[4], 0),
    Joint(&joint_actuator[5], &joint_trajectory[5], 0),
    Joint(&joint_actuator[6], &joint_trajectory[6], 0)
}; // 关节对象数组

float Gravity_feedforward[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 重力前馈数组
float BackEMF_feedforward[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 电流前馈数组
float Coulomb_feedforward[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 库仑前馈数组
float Viscous_feedforward[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 黏性前馈数组
float Inertia_feedforward[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // 惯量补偿数组

bool all_motor_connected = false; // 所有电机连接状态
uint8_t OreBit[4] = {0, 0, 0, 0};                                                                                                        // 初始矿石位为空，0为空，1为有矿，OreBit[0]为左矿石位，OreBit[1]为右矿石位
uint8_t UI_Mode;          // UI模式
uint8_t Robot_Prepared = 1; // 机器人准备状态，DM电机无需力矩复位，默认已准备好
Indicator_Classdef Indicator(&htim1, &hdma_tim1_ch1, TIM_CHANNEL_1);           // 指示灯
float gravity_J2 = 0.56f;
float gravity_J3 = -1.7f;
float gravity_J4 = 1.7f;   // J4 重力补偿增益 (TODO: 需实机调参, 初始建议 0.1~0.5)
float gravity_J5 = 1.2f;   // J5 roll 重力补偿增益 (J6电机外置Y偏置产生, TODO: 实机调参)
float gravity_J6 = 1.7f;  // J6 重力补偿增益 (TODO: 需实机调参, 初始建议 0.05~0.3)
float inertia_J1 = 0.0f;

float chassis_height_min = 0.203f;
float chassis_height_max = 0.458f;
float chassis_height_step = 0.03f;
float current_chassis_height = 0.203f;
float target_chassis_height = 0.203f;
bool target_height_initialized = false;
bool align_to_target_active = false;

ToBottomDataStructdef toBottomdata; // 发送到底盘的数据结构体
ReBottomDataStructdef reBottomdata; // 从底盘接收的数据结构体
ReBottomDataStructdef temp_reBottomdata; // 作为Buffer的底盘数据结构体

ToVisionDataStruct toVisiondata;
ReVisionDataStruct reVisiondata;
uint8_t vision_comm_flag_override = 0U;
bool vision_comm_flag_override_valid = false;

// PID调参
float speed_kp, speed_Out_Max, angle_kp, angle_ki, angle_kd, angle_Out_Max, angle_I_Max, debug_target_angle = 0.000f;
float speed_kp1, speed_Out_Max1, angle_kp1, angle_ki1, angle_kd1, angle_Out_Max1, angle_I_Max1, debug_target_angle1 = 0.000f;

MeanFilter<50> f_torque1, f_omega1;

uint8_t ArmMode;

uint8_t DTR_state;
uint8_t tx_result; 

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
