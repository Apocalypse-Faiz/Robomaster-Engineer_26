/**
  ******************************************************************************
  * Copyright (c) 2019 - ~, SCUT-RobotLab Development Team
  * @file    Robotic_Math .cpp
  * @author  cqy 3049623863@qq.com
  * @brief
  * @date    2024/01/16 19:04:15
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
      -# 该库为23赛季机械臂库中的数学运算部分，将工程常用的四元数，旋转矩阵等的转换封为一个数学运算库
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

/* Includes ------------------------------------------------------------------*/
#include "Robotic_Math.h"

/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/********************************** 旋转矩阵 **********************************/

S_RotationMatrix::S_RotationMatrix(const double &a11, const double &a12, const double &a13,
                                   const double &a21, const double &a22, const double &a23,
                                   const double &a31, const double &a32, const double &a33)
{
  data[0] = a11, data[1] = a12, data[2] = a13,
  data[3] = a21, data[4] = a22, data[5] = a23,
  data[6] = a31, data[7] = a32, data[8] = a33;
}

S_RotationMatrix operator+(S_RotationMatrix &a, S_RotationMatrix &b)
{
  S_RotationMatrix rotm;
  for (uint8_t i = 1; i < 4; i++)
    for (uint8_t j = 1; j < 4; j++)
      rotm.R(i, j) = a.R(i, j) + b.R(i, j);
  return rotm;
}
S_RotationMatrix operator-(S_RotationMatrix &a, S_RotationMatrix &b)
{
  S_RotationMatrix rotm;
  for (uint8_t i = 1; i < 4; i++)
    for (uint8_t j = 1; j < 4; j++)
      rotm.R(i, j) = a.R(i, j) - b.R(i, j);
  return rotm;
}
S_RotationMatrix operator*(S_RotationMatrix &a, S_RotationMatrix &b)
{
  S_RotationMatrix rotm;
  for (uint8_t i = 1; i < 4; i++)
    for (uint8_t j = 1; j < 4; j++)
    {
      rotm.R(i, j) = 0;
      for (uint8_t k = 1; k < 4; k++)
        rotm.R(i, j) += a.R(i, k) * b.R(k, j);
    }
  // for (uint8_t i = 0; i < 3; i++)
  //   for (uint8_t j = 0; j < 3; j++)
  //   {
  //     rotm.data[i*3+j] = 0;
  //     for (uint8_t k = 0; k < 3; k++)
  //      rotm.data[i*3+j] += a.data[i*3+k]  * b.data[k*3+j];
  //   }
  return rotm;
}
S_RotationMatrix operator/(S_RotationMatrix &a, S_RotationMatrix &b)
{
  S_RotationMatrix rotm;
  rotm = b.inverse();
  return a * rotm;
}

S_Position operator*(S_RotationMatrix &a, S_Position &b)
{
  S_Position pos;
  for (uint8_t i = 1; i < 4; i++)
    for (uint8_t j = 1; j < 4; j++)
      pos.P(i) += a.R(i, j) * b.P(j);
  return pos;
}

/**
 * @brief 获取或修改第row行、第coloumn列数据，跟操作数组元素方法一样
 *
 * @param row 从1开始数起
 * @param coloumn 从1开始数起
 * @return double& 可被修改的矩阵元素
 */
double &S_RotationMatrix::R(const uint8_t &row, const uint8_t &coloumn)
{
  // if (row < 1 || row > 3 || coloumn < 1 || coloumn > 3) return 0.0f;
  return data[3 * (row - 1) + (coloumn - 1)];
}
// 求逆
const S_RotationMatrix S_RotationMatrix::inverse()
{
  double det = R(1, 1) * (R(2, 2) * R(3, 3) - R(2, 3) * R(3, 2)) - R(1, 2) * (R(2, 3) * R(3, 1) - R(2, 1) * R(3, 3)) + R(1, 3) * (R(2, 1) * R(3, 2) - R(2, 2) * R(3, 1));
  double inv_det = 1.0f / det;
  S_RotationMatrix r((R(2, 2) * R(3, 3) - R(2, 3) * R(3, 2)) * inv_det,
                     (R(1, 3) * R(3, 2) - R(1, 2) * R(3, 3)) * inv_det,
                     (R(1, 2) * R(2, 3) - R(1, 3) * R(2, 2)) * inv_det,
                     (R(2, 3) * R(3, 1) - R(2, 1) * R(3, 3)) * inv_det,
                     (R(1, 1) * R(3, 3) - R(1, 3) * R(3, 1)) * inv_det,
                     (R(1, 3) * R(2, 1) - R(1, 1) * R(2, 3)) * inv_det,
                     (R(2, 1) * R(3, 2) - R(2, 2) * R(3, 1)) * inv_det,
                     (R(1, 2) * R(3, 1) - R(1, 1) * R(3, 2)) * inv_det,
                     (R(1, 1) * R(2, 2) - R(1, 2) * R(2, 1)) * inv_det);
  return r;
}
// 转置
const S_RotationMatrix S_RotationMatrix::transpose()
{
  S_RotationMatrix r;
  for (uint8_t i = 0; i < 3; i++)
    for (uint8_t j = 0; j < 3; j++)
      r.data[i * 3 + j] = R(j + 1, i + 1);
  return r;
}
// 转四元数
const S_Quarternion S_RotationMatrix::toQuat()
{
  S_Quarternion q;
  double tr = data[0] + data[4] + data[8];
  if (tr > 0)
  {
    double S = sqrt(tr + 1.0f) * 2.0f; // S=4*qw
    q.q[0] = 0.25f * S;
    q.q[1] = (R(3, 2) - R(2, 3)) / S;
    q.q[2] = (R(1, 3) - R(3, 1)) / S;
    q.q[3] = (R(2, 1) - R(1, 2)) / S;
  }
  else if ((R(1, 1) > R(2, 2)) && (R(1, 1) > R(3, 3)))
  {
    double S = sqrt(1.0f + R(1, 1) - R(2, 2) - R(3, 3)) * 2.0f; // S=4*qx
    q.q[0] = (R(3, 2) - R(2, 3)) / S;
    q.q[1] = 0.25f * S;
    q.q[2] = (R(1, 2) + R(2, 1)) / S;
    q.q[3] = (R(1, 3) + R(3, 1)) / S;
  }
  else if (R(2, 2) > R(3, 3))
  {
    double S = sqrt(1.0f + R(2, 2) - R(1, 1) - R(3, 3)) * 2.0f; // S=4*qy
    q.q[0] = (R(1, 3) - R(3, 1)) / S;
    q.q[1] = (R(1, 2) + R(2, 1)) / S;
    q.q[2] = 0.25f * S;
    q.q[3] = (R(2, 3) + R(3, 2)) / S;
  }
  else
  {
    double S = sqrt(1.0f + R(3, 3) - R(1, 1) - R(2, 2)) * 2.0f; // S=4*qz
    q.q[0] = (R(2, 1) - R(1, 2)) / S;
    q.q[1] = (R(1, 3) + R(3, 1)) / S;
    q.q[2] = (R(2, 3) + R(3, 1)) / S;
    q.q[3] = 0.25f * S;
  }

  return q.normalize();
}
// 转zyx欧拉角
const S_EulerAngle S_RotationMatrix::toEuler()
{
  S_EulerAngle e;
  e.beta = atan2(-R(3, 1), sqrt(R(1, 1) * R(1, 1) + R(2, 1) * R(2, 1)));
  if (abs(e.beta) < PI / 2.0f)
  {
    double cos_beta = cos(e.beta);
    e.alpha = atan2(R(2, 1) / cos_beta, R(1, 1) / cos_beta);
    e.gamma = atan2(R(3, 2) / cos_beta, R(3, 3) / cos_beta);
  }
  else
  {
    e.alpha = atan2(-R(2, 3), R(2, 2));
    e.gamma = 0.0f;
  }
  return e;
}
/********************************** 旋转矩阵 **********************************/

/*********************************** 欧拉角 ***********************************/

S_EulerAngle::S_EulerAngle(const S_EulerAngle &e)
{
  this->alpha = e.alpha;
  this->beta = e.beta;
  this->gamma = e.gamma;
}

S_EulerAngle::S_EulerAngle(const double &_alpha, const double &_beta, const double &_gamma)
{
  alpha = _alpha, beta = _beta, gamma = _gamma;
}

S_EulerAngle operator+(const S_EulerAngle &a, const S_EulerAngle &b)
{
  S_EulerAngle e;
  e.alpha = a.alpha + b.alpha;
  e.beta = a.beta + b.beta;
  e.gamma = a.gamma + b.gamma;
  return e;
}

S_EulerAngle operator-(const S_EulerAngle &a, const S_EulerAngle &b)
{
  S_EulerAngle e;
  e.alpha = a.alpha + b.alpha;
  e.beta = a.beta + b.beta;
  e.gamma = a.gamma + b.gamma;
  return e;
}

double operator*(const S_EulerAngle &a, const S_EulerAngle &b)
{
  double sum = 0.0f;
  sum = a.alpha * b.alpha + a.beta * b.beta + a.gamma * b.gamma;
  return sum;
}
// 转四元数
S_Quarternion S_EulerAngle::toQuat() const
{
  S_Quarternion q;
  double ca = cos(alpha * 0.5f);
  double sa = sin(alpha * 0.5f);
  double cb = cos(beta * 0.5f);
  double sb = sin(beta * 0.5f);
  double cg = cos(gamma * 0.5f);
  double sg = sin(gamma * 0.5f);

  q.q[0] = ca * cb * cg + sa * sb * sg;
  q.q[1] = ca * cb * sg - sa * sb * cg;
  q.q[2] = ca * sb * cg + sa * cb * sg;
  q.q[3] = sa * cb * cg - ca * sb * sg;

  q = q.normalize();

  return q;
}
// 转旋转矩阵
S_RotationMatrix S_EulerAngle::toRotm() const
{
  double cx = cos(gamma);
  double sx = sin(gamma);
  double cy = cos(beta);
  double sy = sin(beta);
  double cz = cos(alpha);
  double sz = sin(alpha);

  S_RotationMatrix r(cy * cz,
                     sx * sy * cz - cx * sz,
                     cx * sy * cz + sx * sz,
                     cy * sz,
                     sx * sy * sz + cx * cz,
                     cx * sy * sz - sx * cz,
                     -sy,
                     sx * cy,
                     cx * cy);

  return r;
}
// zyx欧拉角转yaw,pitch,roll
void S_EulerAngle::toYPR(double &_yaw, double &_pitch, double &_roll) const
{
  _yaw = Degrees(alpha), _pitch = Degrees(beta), _roll = Degrees(gamma);
}
// yaw,pitch,roll转zyx欧拉角
void S_EulerAngle::fromYPR(const double &_yaw, const double &_pitch, const double &_roll)
{
  alpha = Radians(_yaw), beta = Radians(_pitch), gamma = Radians(_roll);
}
/*********************************** 欧拉角 ***********************************/

/*********************************** 四元数 ***********************************/

S_Quarternion::S_Quarternion(const S_Quarternion &qq)
{
  q[0] = qq.q[0], q[1] = qq.q[1], q[2] = qq.q[2], q[3] = qq.q[3];
  *this = this->normalize();
}

S_Quarternion::S_Quarternion(const double q1, const double q2, const double q3, const double q4)
{
  q[0] = q1, q[1] = q2, q[2] = q3, q[3] = q4;
  *this = this->normalize();
}

S_Quarternion operator+(const S_Quarternion &a, const S_Quarternion &b)
{
  S_Quarternion c;
  c.q[0] = a.q[0] + b.q[0];
  c.q[1] = a.q[1] + b.q[1];
  c.q[2] = a.q[2] + b.q[2];
  c.q[3] = a.q[3] + b.q[3];
  return c.normalize();
}
S_Quarternion operator-(const S_Quarternion &a, const S_Quarternion &b)
{
  S_Quarternion c;
  c.q[0] = a.q[0] - b.q[0];
  c.q[1] = a.q[1] - b.q[1];
  c.q[2] = a.q[2] - b.q[2];
  c.q[3] = a.q[3] - b.q[3];
  return c.normalize();
}
S_Quarternion operator*(const S_Quarternion &a, const S_Quarternion &b)
{
  const double &w1 = a.q[0], &x1 = a.q[1], &y1 = a.q[2], &z1 = a.q[3];
  const double &w2 = b.q[0], &x2 = b.q[1], &y2 = b.q[2], &z2 = b.q[3];
  S_Quarternion c;

  c.q[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
  c.q[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
  c.q[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
  c.q[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;

  return c.normalize();
}
// 求共轭
S_Quarternion S_Quarternion::conjugate() const
{
  S_Quarternion qq;
  qq.q[0] = q[0];
  qq.q[1] = -q[1];
  qq.q[2] = -q[2];
  qq.q[3] = -q[3];
  return qq;
}
// 求模长
double S_Quarternion::norm() const
{
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}
// 四元数标准化
S_Quarternion S_Quarternion::normalize() const
{
  S_Quarternion qq;
  double n = this->norm();
  qq.q[0] = q[0] / n;
  qq.q[1] = q[1] / n;
  qq.q[2] = q[2] / n;
  qq.q[3] = q[3] / n;
  return qq;
}
// 求逆
S_Quarternion S_Quarternion::inverse() const
{
  S_Quarternion qq;
  double n = this->norm();
  qq.q[0] = q[0] / n;
  qq.q[1] = -q[1] / n;
  qq.q[2] = -q[2] / n;
  qq.q[3] = -q[3] / n;
  return qq;
}
// 转旋转矩阵
S_RotationMatrix S_Quarternion::toRotm() const
{
  double qw = q[0];
  double qx = q[1];
  double qy = q[2];
  double qz = q[3];

  double temp[9] = {0};

  temp[0] = 1 - 2 * qy * qy - 2 * qz * qz;
  temp[1] = 2 * qx * qy - 2 * qz * qw;
  temp[2] = 2 * qx * qz + 2 * qy * qw;
  temp[3] = 2 * qx * qy + 2 * qz * qw;
  temp[4] = 1 - 2 * qx * qx - 2 * qz * qz;
  temp[5] = 2 * qy * qz - 2 * qx * qw;
  temp[6] = 2 * qx * qz - 2 * qy * qw;
  temp[7] = 2 * qy * qz + 2 * qx * qw;
  temp[8] = 1 - 2 * qx * qx - 2 * qy * qy;

  S_RotationMatrix r(temp);
  return r;
}
// 转zyx欧拉角, 弧度制
S_EulerAngle S_Quarternion::toEuler() const
{
  S_EulerAngle e;

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  e.alpha = atan2(siny_cosp, cosy_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (abs(sinp) >= 1)
    e.beta = copysign(PI / 2.0f, sinp); // use 90 degrees if out of range
  else
    e.beta = asin(sinp);

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  e.gamma = atan2(sinr_cosp, cosr_cosp);

  // double sqw = q[0] * q[0];
  // double sqx = q[1] * q[1];
  // double sqy = q[2] * q[2];
  // double sqz = q[3] * q[3];
  // double unit = sqx + sqy + sqz + sqw; // 四元数单位化
  // double test = q[1] * q[2] + q[3] * q[0];
  // if (test > 0.499f * unit)
  // { // 两极情况
  //   e.alpha = 2 * atan2(q[1], q[0]);
  //   e.beta = PI / 2;
  //   e.gamma = 0;
  // }
  // else if (test < -0.499f * unit)
  // {
  //   e.alpha = -2 * atan2(q[1], q[0]);
  //   e.beta = -PI / 2;
  //   e.gamma = 0;
  // }
  // else
  // {
  //   e.alpha = atan2(2 * q[2] * q[0] - 2 * q[1] * q[3], sqx - sqy - sqz + sqw);
  //   e.beta = asin(2 * test / unit);
  //   e.gamma = atan2(2 * q[1] * q[0] - 2 * q[2] * q[3], -sqx + sqy - sqz + sqw);
  // }

  return e;
}

/*********************************** 四元数 ***********************************/

/*********************************** 位置矢量 ***********************************/

S_Position operator+(const S_Position &a, const S_Position &b)
{
  S_Position c;
  c.x = a.x + b.x;
  c.y = a.y + b.y;
  c.z = a.z + b.z;
  return c;
}
S_Position operator-(const S_Position &a, const S_Position &b)
{
  S_Position c;
  c.x = a.x - b.x;
  c.y = a.y - b.y;
  c.z = a.z - b.z;
  return c;
}
double operator*(const S_Position &a, const S_Position &b)
{
  double sum = 0.0f;
  sum = a.x * b.x + a.y * b.y + a.z * b.z;
  return sum;
}
/*********************************** 位置矢量 ***********************************/

/******************************** 齐次变换矩阵 *********************************/

S_Pose::S_Pose(const S_Pose &_pose)
{
  rotm = _pose.rotm;
  pos = _pose.pos;
}

S_Pose::S_Pose(const double &_x, const double &_y, const double &_z,
               const double &_yaw, const double &_pitch, const double &_roll)
{
  pos.x = _x, pos.y = _y, pos.z = _z;
  S_EulerAngle e(Radians(_yaw), Radians(_roll), Radians(_pitch));
  rotm = e.toRotm();
}

/**
 * @brief 获取或修改第row行、第coloumn列数据，跟操作数组元素方法一样
 *
 * @param row 从1开始数起
 * @param coloumn 从1开始数起
 * @return double& 可被修改的矩阵元素
 */
double &S_Pose::P(const uint8_t &row, const uint8_t &coloumn)
{
  static double temp = 0.0f;
  if (row == 4 && coloumn == 4)
    return temp = 1.0f;
  if (row == 4)
    return temp = 0.0f;
  if (coloumn == 4)
    return pos.P(row);         // 位于位置矢量
  return rotm.R(row, coloumn); // 位于旋转矩阵
}
// 求逆
const S_Pose S_Pose::inverse()
{
  S_Pose p;
  p.rotm = rotm.inverse();
  S_Position zero(0, 0, 0);
  p.pos = zero - rotm * pos;
  return p;
}
// 转位置矢量
void S_Pose::toXYZ(double &x, double &y, double &z) const
{
  x = pos.x, y = pos.y, z = pos.z;
}
// 转yaw,pitch,roll，角度制
void S_Pose::toYPR(double &yaw, double &pitch, double &roll)
{
  S_EulerAngle e = rotm.toEuler();
  yaw = Degrees(e.alpha);
  roll = Degrees(e.beta);
  pitch = Degrees(e.gamma);
}
// 转zyx欧拉角，弧度制
S_EulerAngle S_Pose::toEuler(double &alpha, double &beta, double &gamma)
{
  S_EulerAngle e = rotm.toEuler();
  alpha = e.alpha, beta = e.beta, gamma = e.gamma;
  return e;
}
// 位置矢量转齐次变换矩阵
void S_Pose::fromXYZ(const double &_x, const double &_y, const double &_z)
{
  pos.x = _x, pos.y = _y, pos.z = _z;
}
// yaw,pitch,roll转齐次变换矩阵
void S_Pose::fromYPR(const double &_yaw, const double &_pitch, const double &_roll)
{
  S_EulerAngle e(Radians(_yaw), Radians(_pitch), Radians(_roll));
  rotm = e.toRotm();
}
// zyx欧拉角转齐次变换矩阵
void S_Pose::fromEuler(const double &_alpha, const double &_beta, const double &_gamma)
{
  S_EulerAngle e(_alpha, _beta, _gamma);
  rotm = e.toRotm();
}

S_Pose operator+(S_Pose &a, S_Pose &b)
{
  S_Pose c;
  c.rotm = a.rotm + b.rotm;
  c.pos = a.pos + b.pos;
  return c;
}

S_Pose operator-(S_Pose &a, S_Pose &b)
{
  S_Pose c;
  c.rotm = a.rotm - b.rotm;
  c.pos = a.pos - b.pos;
  return c;
}

S_Pose operator*(S_Pose &a, S_Pose &b)
{
  S_Pose c;
  c.rotm = a.rotm * b.rotm;
  c.pos = a.rotm * b.pos + a.pos;
  return c;
}

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
