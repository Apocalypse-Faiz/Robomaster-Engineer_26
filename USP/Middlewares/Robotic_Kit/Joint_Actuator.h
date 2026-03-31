/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Joint_Actuator.cpp
  * @author  cqy 3049623863@qq.com 
  * @brief   
  * @date    2024/01/16 17:13:20
  * @version 1.0
  * @par Change Log:
  * <table>
  * <tr><th>Date <th>Version <th>Author <th>Description
  * <tr><td>2024-01-16 <td> 1.0 <td>S.B. <td>Creator
  * </table>
  *
  ==============================================================================
                               How to use this Lib  
  ==============================================================================
    @note
      -# 由于23赛季的机械臂库适配旧的SRML库，24赛季使用新的SRML库，本库适配新的SRML库
      -# 本库在23赛季的机械臂库的基础上进行修改，用于获得关节与驱动器之间的映射
      -# 该库简化了很多旧的机械臂库的定义，关节到驱动器的映射以及驱动器到关节的反馈完全由用户自定义给出
    @warning
      -# 这只是简化版本，还有很多细节待完善
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
#include "srml_std_lib.h"
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/********************* 抽象关节 *********************/
/**
 * 抽象关节类型：旋转关节、滑动关节
 */
enum class E_Joint_Type
{
    Revolute = 0,
    Sliding,
};
/**
 * @brief 抽象关节参数包
 *
 */
typedef struct S_Joint_Param
{
    /**
     * @brief 关节参数结构体构造函数
     *
     * @param _k 关节控制量 到 驱动器控制量 的比例关系
     * @param _b 关节控制量 到 驱动器控制量 的基准值
     * @param _min_angle 关节角（或偏移量）限幅
     * @param _max_angle
     * @param _min_speed 关节速度（转速或直线速度）限幅
     * @param _max_speed
     * @param _min_torque 关节力矩限幅
     * @param _max_torque
     */
    S_Joint_Param(float _k = 1.0f,
                  float _b = 0.0f,
                  float _min_angle = -(float)(INT32_MAX),
                  float _max_angle = (float)(INT32_MAX),
                  float _min_speed = -(float)(INT32_MAX),
                  float _max_speed = (float)(INT32_MAX),
                  float _min_torque = -(float)(INT32_MAX),
                  float _max_torque = (float)(INT32_MAX)) : k(_k),
                                               b(_b),
                                               min_angle(_min_angle),
                                               max_angle(_max_angle),
                                               min_speed(_min_speed),
                                               max_speed(_max_speed),
                                               min_torque(_min_torque),
                                               max_torque(_min_torque){};
    /* 关节控制量 到 驱动器控制量 的线性关系
        即：关节控制量 * k + b = 驱动器控制量
        一般的，k由齿轮间的减速比确定 */
    float k = 1.0f, b = 0.0f;

    /* 关节角（或偏移量）限幅 */
    float min_angle;
    float max_angle;
    /* 关节速度（转速或直线速度）限幅 */
    float min_speed;
    float max_speed;
    /* 关节力矩限幅 */
    float min_torque;
    float max_torque;
} S_Joint_Param;

/* Exported function declarations --------------------------------------------*/
class C_Joint
{
public:
    C_Joint(){};
    C_Joint(const S_Joint_Param &_Param) : Param(_Param){};
    C_Joint(const S_Joint_Param &_Param, float (*_getActuatorTargetCustomptr)(float &, float &, float &), float (*_getJointCurrentCustomptr)(float &, float &))
        : Param(_Param), getActuatorTargetCustomptr(_getActuatorTargetCustomptr), getJointCurrentCustomptr(_getJointCurrentCustomptr){};

    // 配置关节参数
    void setParam(const S_Joint_Param &_Param) { Param = _Param; }

    // 由用户自定义函数获得关节映射到驱动器的目标
    float getActuatTargetCustom(float &_target)
    {
        return getActuatorTargetCustomptr(_target, Param.k, Param.b);
    }
    // 由用户自定义的函数获得驱动器到关节的当前值
    float getJointCurrentCustom() { return getJointCurrentCustomptr(Param.k, Param.b); }

    // 获取关节参数
    const S_Joint_Param &getParam() { return Param; }

    // 设置关节偏移量，用于关节零点的标定
    void setOffset(float _offset)
    {
        this->offset = _offset;
    }
    float getOffset()
    {
        return this->offset;
    }

private:
    float offset;
    S_Joint_Param Param;
    float (*getActuatorTargetCustomptr)(float &_target, float &_k, float &_b); // 自定义设置目标函数
    float (*getJointCurrentCustomptr)(float &_k, float &_b);                   // 自定义设置目标函数
};

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
