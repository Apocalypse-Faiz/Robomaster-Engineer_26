#include "stdint.h"
#include "srml_std_lib.h"
#include "math.h"

/**
 * @brief 7-DOF 重力补偿计算（完整版）
 *
 * @param _J2  Link 2 绝对俯仰角 (度), 0°=水平前伸, >0°=上仰
 * @param _J3  Link 3 相对 Link 2 俯仰角 (度), 0°=共线, >0°=相对上仰
 * @param _J4  Link 4 相对 Link 3 俯仰角 (度), 0°=共线, >0°=相对上仰
 * @param _J5  J5 roll 角 (度), 0°=腕部竖直, >0°=从后看顺时针
 * @param _J6  Wrist 相对 Link 4 俯仰角 (度), 0°=共线, >0°=相对上仰
 * @param tau_J2 输出: J2 肩关节重力力矩 (Nm)
 * @param tau_J3 输出: J3 肘关节重力力矩 (Nm)
 * @param tau_J4 输出: J4 pitch 关节重力力矩 (Nm)
 * @param tau_J5 输出: J5 roll 关节重力力矩 (Nm, 由 wrist 质心 Y 偏置 LCWY 产生)
 * @param tau_J6 输出: J6 腕部 pitch 关节重力力矩 (Nm)
 *
 * @note 坐标系与角度参考方向
 *   ========================
 *   世界系 {0}: Z₀↑ (竖直向上), X₀→ (前向), Y₀⊗ (向纸面内)
 *   重力矢量 g⃗ = [0, 0, -9.81]ᵀ
 *
 *   绝对俯仰角: 0°=水平前伸(∥X₀Y₀), >0°=上仰(绕+Y₀右手定则)
 *     (上仰为正 → 内部使用 Ry(-α) 旋转: X₀ = x·cosα - z·sinα)
 *
 *   J5 Roll: 0°=腕部竖直, >0°=从后看顺时针
 *   相对俯仰: 0°=两连杆共线, >0°=远端相对近端上仰
 *
 *   内部累加: abs_i = abs_{i-1} + rel_i
 *   调用侧负责 MDH→本函数约定的转换 (见 MDH2Gravity_xxx)
 *
 * @note 物理参数说明 — 质量体划分
 *   ================================
 *   约定: 连杆 i 质量 = 结构件 i + 电机 i+1
 *     M2  = 结构件 2 + J3 电机           (质心在 {2} 系)
 *     M3  = 结构件 3 + J4 电机           (质心在 {3} 系)
 *     M4  = 结构件 4 + J5 电机           (质心在 {4} 系)
 *     MW  = J6电机 + J7电机 + 夹爪 + 负载 (质心在 wrist/{6} 系)
 *
 *   所有参数值需实机测量标定 (当前为估算值).
 */
void GravityCompension_Calculate(float _J2, float _J3, float _J4, float _J5, float _J6,
                                  float *tau_J2, float *tau_J3, float *tau_J4, float *tau_J5, float *tau_J6);

void BackEMFCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *backEMF0, float *backEMF1, float *backEMF2, float *backEMF3, float *backEMF4, float *backEMF5);
void CoulombCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *coulomb0, float *coulomb1, float *coulomb2, float *coulomb3, float *coulomb4, float *coulomb5);
void ViscousCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *viscous0, float *viscous1, float *viscous2, float *viscous3, float *viscous4, float *viscous5);
void GripperGravityCompension_Calculate(float _J5, float *torque_J5);

/**
 * @brief 7-DOF J1 (Yaw) 转动惯量补偿计算 (纯函数, 无内部状态)
 *
 * @param _J2  Link 2 绝对俯仰角 (度), 0°=水平前伸, >0°=上仰
 * @param _J3  Link 3 相对 Link 2 俯仰角 (度), 0°=共线, >0°=相对上仰
 * @param _J4  Link 4 相对 Link 3 俯仰角 (度), 0°=共线, >0°=相对上仰
 * @param _J5  J5 roll 角 (度), 0°=腕部竖直, >0°=从后看顺时针
 * @param _J6  Wrist 相对 Link 4 俯仰角 (度), 0°=共线, >0°=相对上仰
 * @param alpha_J1           预估计的 J1 角加速度 (由 EstimateJ1Acceleration 提供)
 * @param Motor_Set_torque_J1 J1 PID 输出力矩 (Nm), 用于死区判断
 * @param torque_J1           输出: J1 惯量补偿力矩 (Nm)
 *
 * @note 坐标系与角度参考方向 — 与重力补偿完全一致
 *   ==============================================
 *   世界系 {0}: Z₀↑(J1旋转轴), X₀→前, Y₀⊗
 *   绝对俯仰: 0°=水平前伸, >0°=上仰 (Ry(-α) 展开)
 *   相对俯仰: 0°=两连杆共线, >0°=远端相对近端上仰
 *   J5 Roll:  0°=腕部竖直, >0°=从后看顺时针
 *   内部累加: abs_i = abs_{i-1} + rel_i
 *
 * @note 惯量模型 — 平行轴定理
 *   ==========================
 *   Ieq = I1_BASE + Σ mᵢ·ρᵢ² + Σ I_rot(θᵢ)
 *   平动项: m·ρ², ρ 为质心到 Z₀ 轴的水平距离
 *   转动项: I_xx·sin²θ + I_zz·cos²θ - 2·I_xz·sinθ·cosθ
 *
 *   Wrist 的 ρ 计算使用 {4} 系正交分解 (Rx·Ry 链),
 *   正确处理 J5≠0 时 J6 pitch 产生的 Y₀ 方向偏移.
 *
 * @note 质量体划分 — 与重力补偿一致
 *   M2=结构件2+J3电机, M3=结构件3+J4电机, M4=结构件4+J5电机, MW=J6+J7+夹爪+负载
 *
 * @note 本函数为纯计算 (无 static 状态), 可安全多次调用.
 */
void InertiaCompension_Calculate(float _J2, float _J3, float _J4, float _J5, float _J6,
                                  float alpha_J1,            // 预估计的 J1 角加速度
                                  float Motor_Set_torque_J1,  // 用于死区判断
                                  float *torque_J1);

/**
 * @brief J1 角加速度估计 (速度数值微分 + 一阶低通滤波)
 * @param motor_speed_j1  J1 电机当前速度
 * @return 滤波后的 J1 角加速度估计值
 * @note  含 static 状态, 每控制周期只能调用一次
 */
float EstimateJ1Acceleration(float motor_speed_j1);

void InertiaCompension_ValidationStep(float _J1, float _J2, float _J3, float _J4, float _J5, float _J6, float motor_speed_j1, float spring_k, float torque_limit, float speed_limit, uint8_t recenter_request, float *cmd_torque_j1, float *inertia_total_j1, float *k_dynamic_j1, float *theta1_center_deg, float *measured_period_ms);
