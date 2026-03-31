/**
 ******************************************************************************
 * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
 * @file    Robotic_Math.h
 * @author  cqy 3049623863@qq.com
 * @brief   Header file
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
#include <math.h>
#include "main.h"
#include "arm_math.h" //使用dsp库，开启浮点数运算硬件加速
/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
/********************* 旋转矩阵、欧拉角、四元数 *********************/

typedef struct S_RotationMatrix S_RotationMatrix;
typedef struct S_Position S_Position;
typedef struct S_EulerAngle S_EulerAngle;
typedef struct S_Quarternion S_Quaternion;

#ifndef Radians
// 角度转弧度
#define Radians(degrees) ((degrees) * PI / 180.0f)
// double degrees2radians(double degrees)
// {
//   return degrees * PI / 180.0f;
// }
#endif

#ifndef Degrees
// 弧度转角度
#define Degrees(radians) ((radians) * 180.0f / PI)
// double radians2degrees(double radians)
// {
//   return radians * 180.0f / PI;
// }
#endif

/**
 * @brief 旋转矩阵
 *
 */
typedef struct S_RotationMatrix
{
  double data[9];

  S_RotationMatrix() { memset(data, 0, sizeof(data)); }

  S_RotationMatrix(const S_RotationMatrix &r)
  {
    if (&r != this)
    {
      memcpy(data, r.data, sizeof(r.data));
    }
  }

  S_RotationMatrix(const double *ptr_data)
  {
    memcpy(data, ptr_data, sizeof(data));
  }

  S_RotationMatrix(const double &a11, const double &a12, const double &a13,
                   const double &a21, const double &a22, const double &a23,
                   const double &a31, const double &a32, const double &a33);

  double &R(const uint8_t &row, const uint8_t &coloumn); // 第row行、第coloumn的元素

  const S_RotationMatrix inverse();   // 求逆
  const S_RotationMatrix transpose(); // 转置
  const S_Quarternion toQuat();       // 转四元数
  const S_EulerAngle toEuler();       // 转zyx欧拉角 经过验证，这更像是XYZ固定角
} S_RotationMatrix;

S_RotationMatrix operator+(S_RotationMatrix &a, S_RotationMatrix &b);
S_RotationMatrix operator-(S_RotationMatrix &a, S_RotationMatrix &b);
S_RotationMatrix operator*(S_RotationMatrix &a, S_RotationMatrix &b);
S_RotationMatrix operator/(S_RotationMatrix &a, S_RotationMatrix &b);
S_Position operator*(S_RotationMatrix &a, S_Position &b);

/**
 * @brief ZYX欧拉角（绕自身坐标系zyx旋转），弧度制。
 *        而陀螺仪上的欧拉角yaw、pitch、roll实际上是机器人学导论里的XYZ固定角。
 *        特别的，两者解算出的旋转矩阵相同。
 */
typedef struct S_EulerAngle
{
  double alpha, beta, gamma;

  S_EulerAngle() {};
  S_EulerAngle(const S_EulerAngle &e);
  S_EulerAngle(const double &_alpha, const double &_beta, const double &_gamma);

  S_Quarternion toQuat() const;                                                // 转四元数
  S_RotationMatrix toRotm() const;                                             // 转旋转矩阵
  void toYPR(double &_yaw, double &_pitch, double &_roll) const;               // zyx欧拉角转yaw,pitch,roll
  void fromYPR(const double &_yaw, const double &_pitch, const double &_roll); // yaw,pitch,roll转zyx欧拉角

private:
  double yaw, pitch, roll;
} S_EulerAngle;

S_EulerAngle operator+(const S_EulerAngle &a, const S_EulerAngle &b);
S_EulerAngle operator-(const S_EulerAngle &a, const S_EulerAngle &b);
double operator*(const S_EulerAngle &a, const S_EulerAngle &b);

/**
 * @brief 四元数 q = [w x y z]
 *
 */
typedef struct S_Quarternion
{
  double q[4];

  S_Quarternion() {};
  S_Quarternion(const S_Quarternion &qq);
  S_Quarternion(const double q1, const double q2, const double q3, const double q4);

  S_RotationMatrix toRotm() const;
  S_EulerAngle toEuler() const;

  S_Quarternion conjugate() const; // 求共轭
  double norm() const;             // 求模长
  S_Quarternion normalize() const; // 四元数标准化
  S_Quarternion inverse() const;   // 求逆

} S_Quarternion;

S_Quarternion operator+(const S_Quarternion &a, const S_Quarternion &b);
S_Quarternion operator-(const S_Quarternion &a, const S_Quarternion &b);
S_Quarternion operator*(const S_Quarternion &a, const S_Quarternion &b);

/**
 * @brief 位置xyz
 *
 */
typedef struct S_Position
{
  double x, y, z;
  S_Position()
  {
    x = 0;
    y = 0;
    z = 0;
  }

  S_Position(const S_Position &p)
  {
    if (&p != this)
    {
      x = p.x, y = p.y, z = p.z;
    }
  }

  S_Position(const double &_x, const double &_y, const double &_z)
  {
    x = _x, y = _y, z = _z;
  };

  // 表示位置矢量第n个元素，n = 1 ~ 3
  double &P(const uint8_t &n)
  {
    return (n == 1) ? x : ((n == 2) ? y : z);
  }
} S_Position;

S_Position operator+(const S_Position &a, const S_Position &b);
S_Position operator-(const S_Position &a, const S_Position &b);
double operator*(const S_Position &a, const S_Position &b);

/**
 * @brief 位姿Pose，4X4的齐次变换矩阵
 * @note 支持的运算：
      -# 位置xyz + 姿态Orientation（ zyx欧拉角或xyz固定角(yaw、pitch、roll) ）==> 齐次变换矩阵
      -# 齐次变换矩阵 <==> 位置S_Position(x、y、z）
      -# 齐次变换矩阵 <==> 姿态Orientation(XYZ固定角，习惯定义上的欧拉角yaw、pitch、roll，角度制)
      -# 齐次变换矩阵 <==> 姿态Orientation(ZYX欧拉角，弧度制)
      -# 矩阵加减乘法,求逆
 */
typedef struct S_Pose
{
  S_RotationMatrix rotm;
  S_Position pos;

  S_Pose() {};
  S_Pose(const S_Pose &_pose);
  S_Pose(const double &_x, const double &_y, const double &_z,
         const double &_yaw, const double &_pitch, const double &_roll);

  double &P(const uint8_t &row, const uint8_t &coloumn);

  const S_Pose inverse(); // 求逆

  void toXYZ(double &x, double &y, double &z) const;                // 转位置矢量
  void toYPR(double &yaw, double &pitch, double &roll);             // 转yaw,pitch,roll，角度制
  S_EulerAngle toEuler(double &alpha, double &beta, double &gamma); // 转zyx欧拉角，弧度制

  void fromXYZ(const double &x, const double &y, const double &z);              // 位置矢量转齐次变换矩阵
  void fromYPR(const double &yaw, const double &pitch, const double &roll);     // yaw,pitch,roll转齐次变换矩阵
  void fromEuler(const double &alpha, const double &beta, const double &gamma); // zyx欧拉角转齐次变换矩阵

private:
  double x, y, z; // 位置矢量

  /* 姿态矢量 */
  double yaw, pitch, roll;   // 习惯定义的欧拉角（陀螺仪角，机器人学里的XYZ固定角），角度制
  double alpha, beta, gamma; // 机器人学里的zyx欧拉角，弧度制

} S_Pose;

S_Pose operator+(const S_Pose &a, const S_Pose &b);
S_Pose operator-(const S_Pose &a, const S_Pose &b);
S_Pose operator*(const S_Pose &a, const S_Pose &b);
S_Position operator*(const S_RotationMatrix &a, const S_Position &b);

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
