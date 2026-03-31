/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Servo.h
  * @author  cqy 3049623863@qq.com
  * @brief
  * @date    2024/01/17 16:51:09
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2024-01-17 <td> 1.0 <td>S.B. <td>Creator
  * </table>
  *
  ==============================================================================
                               How to use this Lib
  ==============================================================================
    @note
      -# 简化的舵机控制库
      -#
    @warning
      -#
      -#
  ******************************************************************************
  * @attention
  *
  * if you had modified this file, please make sure your code does not have many
  * bugs, update the version Number, write dowm your name and the date, the most
  * important is make sure the users will have clear and definite understanding
  * through your new brief.
  *
  * <h2><center>&copy; Copyright (c) 2019 - ~, SCUT-RobotLab Development Team.
  * All rights reserved.</center></h2>
  ******************************************************************************
  */
#pragma once

#ifdef __cplusplus

/* Includes ------------------------------------------------------------------*/
#include "main.h" //使用HAL库的定时器PWM输出
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
class Servo_Classdef
{
public:
    Servo_Classdef(TIM_HandleTypeDef *_htim, uint32_t channel, uint32_t tim_period, float duty_min, float duty_max, uint32_t angle_max) : htim(_htim), Channel(channel), Tim_Period(tim_period),
                                                                                                                                          Duty_Min(duty_min), Duty_Max(duty_max), Angle_Max(angle_max) {};
    ~Servo_Classdef() {};
    /*开启PWM输出*/
    void PWM_Start();
    // 停止PWM的输出
    void PWM_Stop();
    void setTarget(float target);
    void setPWMValue(uint32_t value);

private:
    TIM_HandleTypeDef *htim; // 定时器句柄
    uint32_t Channel;
    uint32_t Tim_Period;

    float Duty_Min;     // 最小占空比
    float Duty_Max;     // 最大占空比
    uint32_t Angle_Max; // 最大转动角度(0~360度)

    uint32_t PWM_Val; // pwm值
};
/* Exported function declarations --------------------------------------------*/

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
