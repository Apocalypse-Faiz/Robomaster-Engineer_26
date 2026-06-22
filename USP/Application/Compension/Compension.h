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
void InertiaCompension_Calculate(float _J2, float _J3, float Motor_Rec_torque_J1, float motor_speed_j1, float *torque_J1);
void InertiaCompension_ValidationStep(float _J1, float _J2, float _J3, float motor_speed_j1, float spring_k, float torque_limit, float speed_limit, float dt, uint8_t recenter_request, float *cmd_torque_j1, float *inertia_total_j1, float *k_dynamic_j1, float *theta1_center_deg);
