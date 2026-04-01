/**
  ******************************************************************************
  * @file    srml_config_template.h
  * @brief   SRML configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to srml_config.h.
  * @version 1.0.0
  * @par Change Log：
  * <table>
  * <tr><th>Date        <th>Version  <th>Author         <th>Description
  * <tr><td>2019-06-12  <td> 0.0.1   <td>               <td>Creator
  * <tr><td>2023-09     <td> 1.0.0   <td>yjh            <td>Updater
  * <tr><td>2023-10-18  <td> 1.1.0   <td>lrc            <td>Updater
  ******************************************************************************
  * MOST IMPORTANTLY, this library is not open source for developers from other
  * schools' robot team currently. Plz make sure your code is under your supervision.
  *
  * Thank for @mannychen @HuanpengLu and other pioneers who have put forward such
  * architechure, and the endeavors of all developers.
  *
  * By downloading, copying, installing or using the software you agree to this license.
  * If you do not agree to this license, do not download, install,
  * copy or use the software.
  *
  *                          License Agreement
  *                For SCUT RobotLab Middleware Layer Library
  *
  * Copyright (c) 2019 - ~, SCUT RobotLab Development Team, all rights reserved.
  */

#ifndef __SRML_CONFIG_H__
#define __SRML_CONFIG_H__

/* ########################## Module Selection ############################## */
/**
  * @brief This is the list of modules to be used in the SRML.
  *        Change the value to 1 to use it.
  */

 #define IIC_DELAY_TIME 0	//软件IIC延时时间

 /**选择Uart的缓冲区是否使用malloc申请，若选择0则会根据UARTx_RX_BUFFER_SIZE在drv_uart.cpp内创建静态数组
 * 使用FreeRTOS的heap4替换了malloc后才能使用该功能，否则可能会出事
 * 考虑可能在裸机上跑，因此添加创建静态数组的选项
 * 出于空间节省考虑，当SRML_UARTBUFF_MALLOC为0时，请把没有使用的串口的UARTx_RX_BUFFER_SIZE定义为0
 * SRML_UARTBUFF_MALLOC为1时，缓冲区大小只跟Uart_Init的传参相关
 * 若没有定义SRML_UARTBUFF_MALLOC，效果等同于SRML_UARTBUFF_MALLOC为1
*/
#define SRML_UARTBUFF_MALLOC 1  

#define UART1_RX_BUFFER_SIZE 128
#define UART2_RX_BUFFER_SIZE 128
#define UART3_RX_BUFFER_SIZE 128
#define UART4_RX_BUFFER_SIZE 128
#define UART5_RX_BUFFER_SIZE 128
#define UART6_RX_BUFFER_SIZE 256
#define UART7_RX_BUFFER_SIZE 0
#define UART8_RX_BUFFER_SIZE 0
#define UART9_RX_BUFFER_SIZE 0
#define UART10_RX_BUFFER_SIZE 0

#define VIRTUALCOM_RX_BUFFER_SIZE 64

#define VOFAMONITOR_MAX_DATA_NUM 25 //Vofa上位机最大发送变量数
/* Drivers ----------------------------------------------------*/
// On-chip Peripheral
#define USE_SRML_CAN                      1   
#define USE_SRML_TIMER                    1   // 不能置零，drv_uart和inv_mpu等底层库有用到
#define USE_SRML_UART                     1   // 不能置零，与底层的耦合度很高
#define USE_SRML_FLASH                    1
#define USE_SRML_I2C                      1
#define USE_SRML_SPI                      0
#define USE_SRML_VIRTUAL_COM              1   // 虚拟串口
// Motor
#define USE_SRML_MOTOR_DJI                1   // RM电机
#define USE_SRML_MF9025_V2                0   // 瓴控MF9025  
#define USE_SRML_MOTOR_DM                 1   // 达妙电机
#define USE_SRML_HT04                     0   // 海泰电机
#define USE_SRML_CYBER_GEAR               1   // 小米微电机
// IMU
#define USE_SRML_MPU6050                  0
#define USE_SRML_LPMS_BE2                 0   // 阿路比IMU
#define USE_SRML_BMX055                   0   // 九轴IMU
#define USE_SRML_BMI088                   0   // BMI088
// Others
#define USE_SRML_DR16                     1
#define USE_SRML_FS_I6X                   0
#define USE_SRML_REFEREE                  0
#define USE_SRML_VT13                     1 
// Unusual Devices
#define USE_SRML_FATFS                    0
#define USE_SRML_W25Qx                    0
#define USE_SRML_VSEC                     0
#define USE_SRML_MOTOR_AK80               0   // 性价比极低已淘汰
// Log
#define USE_SRML_LOG                      1

/* Middlewares -----------------------------------------------*/
// Abstract_class_lib
#define USE_SRML_ABS_LIB                  1   // 各种抽象库
// Algorithm
#define USE_SRML_PID                      1   // PID计算器,若开启，滤波器库也得开启，两个库有耦合
#define USE_SRML_FILTER                   1   // 滤波器库
#define USE_SRML_KALMAN                   0   // 卡尔曼滤波，依赖矩阵运算库，需要开启USE_SRML_MATH
#define USE_SRML_DIFF_CALCULATER          1   // 微分计算器
#define USE_SRML_MATH                     0   // 移植的数学运算库，包括矩阵、欧拉角、四元数等运算
#define USE_SRML_STATISTICS               1   // 统计类
// Module
#define USE_SRML_DIGITAL_POWER            0   // 数字电源
#define USE_SRML_DIGITAL_POWER_V2         0   // 数字电源V2
#define USE_SRML_DIGITAL_POWER_V3         0   // 数字电源V3 
#define USE_SRML_MECENUM_CHASSIS          0   // 新麦轮底盘库
#define USE_SRML_CHASSIS                  0   // 旧底盘库
#define USE_SRML_MOTOR_CTRL               0   // 电机控制库（dji电机库与pid库结合）
#define USE_SRML_OLD_POW_CTRL             0   // 旧功控库
// Utility
#define USE_SRML_VOFA_MONITOR             1   // Vofa上位机
#define USE_SRML_UPPER_MONITOR            0   // Upper上位机
#define USE_SRML_ASUWAVE_MONITOR          0   // Asuwave上位机
#define USE_SRML_LIST                     0
#define USE_SRML_MYASSERT                 0
#define USE_SRML_SYSANALYSIS              0
#define USE_SRML_SYSLOG                   0
#define USE_SRML_EASYLOG                  0   // easylog日志库
// Protocol
#define USE_SRML_SERIAL_LINE_IP           0   

#endif /* __SRML_CONFIG_H__ */

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
