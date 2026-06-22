#include "stdint.h"
#include "srml_std_lib.h"
#include "math.h"

/**
 * @brief 7-DOF 重力补偿计算（完整版）
 *
 * @param _J2  J2 电机当前角度 (度)
 * @param _J3  J3 电机当前角度 (度)
 * @param _J4  J4 电机当前角度 (度)
 * @param _J5  J5 电机当前角度 (度)
 * @param _J6  J6 电机当前角度 (度)
 * @param tau_J2 输出: J2 肩关节重力力矩 (Nm)
 * @param tau_J3 输出: J3 肘关节重力力矩 (Nm)
 * @param tau_J4 输出: J4 pitch 关节重力力矩 (Nm)
 * @param tau_J6 输出: J6 腕部 pitch 关节重力力矩 (Nm, 含 J5 roll 调制)
 *
 * @note 坐标系与角度参考方向
 *   ========================
 *   世界系 {0}: Z↑ (竖直向上), X→ (前向伸展方向), Y⊗ (垂直于 sagittal 平面, 向纸面内)
 *   重力矢量 g = [0, 0, -9.81]ᵀ, 沿 -Z 方向
 *
 *   绝对俯仰角 (Pitch): 绕 +Y 轴旋转, 右手定则
 *     α = 0°  → 连杆水平前伸 (∥ X₀), cos α = 1, 重力力臂最大
 *     α > 0°  → 上仰 (连杆末端高于根部)
 *     α < 0°  → 下俯 (连杆末端低于根部)
 *
 *   J5 Roll 角: 绕连杆纵轴 (≈X₄) 旋转
 *     φ₅ = 0°   → 腕部竖直朝上, J6 pitch 轴 ∥ Y₀, J6 俯仰产生最大重力力矩
 *     φ₅ = ±90° → 腕部水平, J6 pitch 轴 ∥ Z₀, J6 俯仰不产生重力力矩
 *     φ₅ > 0°   → 从后方看向末端, 顺时针旋转为正
 *
 *   角度映射 (电机读数 → 绝对俯仰角):
 *     α₂ = _J2                 (假设 J2 零位 = 水平前伸)
 *     α₃ = -(_J3 + 90°)        (J3 零位偏置 90°, 方向反转)
 *     α₄ = _J4 + J4_ZERO_OFFSET (需根据机械零位标定)
 *     φ₅ = _J5 + J5_ZERO_OFFSET (需根据腕部直立位置标定)
 *     α₆ = _J6 + J6_ZERO_OFFSET (需根据末端水平位置标定)
 *
 * @note 物理参数说明
 *   ==============
 *   质量模型将机械臂分为 4 个质量体:
 *     Link 2   — J2 之后的连杆 2 本体                       (m2, 质心在 {2} 系)
 *     Link 3   — J3 之后的连杆 3 本体                       (m3, 质心在 {3} 系)
 *     Link 4   — J4 之后的连杆 4 本体                       (m4, 质心在 {4} 系)
 *     Wrist    — J5/J6/J7 电机 + 夹爪 + 负载的等效质量体     (mw, 质心在 wrist 系, 位于 J6 之后)
 *
 *   各连杆系: Z 沿连杆纵轴 (向上), X 沿连杆纵轴 (向前)
 *   连杆长度 Lᵢ 为关节 i 到 关节 i+1 的距离
 *
 *   ⚠️ 当前 m2, L2, lc2x, lc2z 沿用旧版实测值;
 *      m3 已从旧版的"等效连杆"重新分配为仅连杆 3 本体的质量;
 *      mw 为新拆分出的腕部质量, m4 为新增连杆 4 质量;
 *      所有估算值均标有 TODO, 需实机测量后更新。
 */
void GravityCompension_Calculate(float _J2, float _J3, float _J4, float _J5, float _J6,
                                  float *tau_J2, float *tau_J3, float *tau_J4, float *tau_J6);

void BackEMFCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *backEMF0, float *backEMF1, float *backEMF2, float *backEMF3, float *backEMF4, float *backEMF5);
void CoulombCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *coulomb0, float *coulomb1, float *coulomb2, float *coulomb3, float *coulomb4, float *coulomb5);
void ViscousCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *viscous0, float *viscous1, float *viscous2, float *viscous3, float *viscous4, float *viscous5);
void GripperGravityCompension_Calculate(float _J5, float *torque_J5);

/**
 * @brief 7-DOF J1 (Yaw) 转动惯量补偿计算（完整版）
 *
 * @param _J2  J2 电机当前角度 (度)
 * @param _J3  J3 电机当前角度 (度)
 * @param _J4  J4 电机当前角度 (度)
 * @param _J5  J5 电机当前角度 (度)
 * @param _J6  J6 电机当前角度 (度)
 * @param Motor_Set_torque_J1 J1 电机当前 PID 输出的设定力矩 (Nm), 用于死区判断
 * @param motor_speed_j1      J1 电机当前速度 (rad/s 或其他速度单位), 用于微分估计角加速度
 * @param torque_J1           输出: J1 惯量补偿力矩 (Nm)
 *
 * @note 坐标系与角度参考方向
 *   ========================
 *   世界系 {0}:  Z₀ ↑ (竖直向上, J1 旋转轴)
 *                X₀ → (前方, 当 J2=0° 时机械臂沿此方向伸展)
 *                Y₀ ⊗ (垂直于 sagittal 平面)
 *   J1 绕 Z₀ 旋转, 右手定则 (拇指向上, 四指逆时针).
 *
 *   绝对俯仰角 θᵢ: 连杆 i 绕 +Y 轴 (纸面内) 的旋转, 右手定则
 *     θᵢ = 0°  → 连杆水平前伸 (∥ X₀Y₀ 平面)
 *     θᵢ > 0°  → 上仰
 *
 *   J5 Roll 角 φ₅: 绕连杆 4 纵轴 (≈X₄) 旋转
 *     φ₅ = 0°   → 腕部竖直, 末端在 sagittal 平面内
 *     φ₅ = 90°  → 腕部水平侧向, 末端垂直于 sagittal 平面
 *
 * @note 惯量模型 — 平行轴定理
 *   ==========================
 *   绕 J1 (Z₀) 的总等效惯量 = Σ( I_i_trans + I_i_rot ), 其中:
 *
 *     I_trans = m · ρ²
 *       平动项: 把质量视为质心处的质点, ρ 为质心到 Z₀ 轴的水平距离
 *       ρ² = X₀² + Y₀² (世界系 XY 平面内的投影距离平方)
 *
 *     I_rot = [0,0,1] · R · I_local · Rᵀ · [0,0,1]ᵀ
 *       转动项: 连杆自身惯性张量 I_local 经旋转矩阵 R 投影到 Z₀ 轴的分量
 *       展开: I_rot = I_xx·sin²θ + I_zz·cos²θ - 2·I_xz·sinθ·cosθ
 *       (θ 为该连杆的绝对俯仰角)
 *
 *   质量体划分 (4 个体):
 *     Link 2   — J2 之后的结构件, 惯性张量在 {2} 系
 *     Link 3   — J3 之后的结构件, 惯性张量在 {3} 系
 *     Link 4   — J4 之后的结构件, 惯性张量在 {4} 系
 *     Wrist    — J5~J7 电机 + 夹爪 + 负载, 惯性张量在 wrist 系 (与 {6} 系对齐)
 *
 *   J5 Roll 对 ρ_wrist 的影响:
 *     腕部质心在 wrist 系中为 [lcwx, 0, lcwz] (Y 向对称, lcwy≈0).
 *     经 J5 roll φ₅ 后, lcwz 分量在 YZ 平面内旋转,
 *     在世界系产生 Y₀ 方向的偏移: ΔY = lcwz·sin φ₅.
 *     因此: ρ²_wrist = (ρ_x)² + (lcwz·sin φ₅)²
 *     其中 ρ_x 为 sagittal 平面内的水平距离 (由 J2~J6 pitch 决定).
 *
 * @note 惯量前馈力矩 = Ieq · α_J1
 *   α_J1 为 J1 角加速度, 由 EstimateJ1Acceleration() 单独估计 (每周期调用一次).
 *   前馈力矩加到电机 PID 输出上, 乘以调参增益 inertia_J1.
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

void InertiaCompension_ValidationStep(float _J1, float _J2, float _J3, float _J4, float _J5, float _J6, float spring_k, float torque_limit, float speed_limit, float dt, uint8_t recenter_request, float *cmd_torque_j1, float *inertia_total_j1, float *k_dynamic_j1, float *theta1_center_deg);
