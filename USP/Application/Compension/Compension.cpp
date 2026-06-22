#include "Compension.h"

// ============================================================================
// 7-DOF 重力补偿 — 拉格朗日法完整推导 (2025)
// ============================================================================
//
// 坐标系定义 (详见 Compension.h)
// ─────────────────────────────────
//   世界系 {0}:  Z₀ ↑ (竖直向上)
//                X₀ → (前方, 机械臂伸展方向)
//                Y₀ ⊗ (垂直于 sagittal 平面, 向纸面内)
//   重力矢量: g⃗ = [0, 0, -g]ᵀ (g = 9.81 m/s²)
//
//   绝对俯仰角 αᵢ: 连杆 i 与水平面 (X₀Y₀ 平面) 的夹角
//      αᵢ = 0° → 水平前伸, 重力力臂 = 全长 → 力矩最大
//      αᵢ > 0° → 上仰 (cos 减小, 力臂缩小)
//
//   重力力矩通用公式:  τ = m·g·(质心在水平方向的投影距离)
//     = m·g·(lcx·cos α + lcz·sin α)
//
//   推导方法: 势能 U = Σ mᵢ·g·hᵢ,  τⱼ = -∂U/∂αⱼ
//   其中 hᵢ 为质心在世界系中的高度 (Z 坐标), 链条式法则展开得各关节力矩.
//
// 质量体划分
// ─────────
//   Link 2: J2 之后的结构件, 长度 L₂, 质心在 {2} 系 ʳc₂ = [lc2x, 0, lc2z]
//   Link 3: J3 之后的结构件, 长度 L₃, 质心在 {3} 系 ʳc₃ = [lc3x, 0, lc3z]
//   Link 4: J4 之后的结构件, 长度 L₄, 质心在 {4} 系 ʳc₄ = [lc4x, 0, lc4z]
//   Wrist : J5~J7 电机 + 夹爪 + 负载的等效体, 质心在 wrist 系 ʳcw = [lcwx, 0, lcwz]
//           (wrist 系定义为 J6 之后的末端坐标系, 与 {6} 系对齐)
//
// J5 Roll 调制效应
// ────────────────
//   J5 绕连杆纵轴旋转, 改变了 J6 pitch 轴的方向.
//   当 φ₅ = 0° (腕部竖直): J6 ∥ Y₀, J6 俯仰全部转化为质心高度变化
//   当 φ₅ = ±90° (腕部水平): J6 ∥ Z₀, J6 俯仰 = 水平面内摆动, 不改变高度
//   调制因子: cos(φ₅) — 将 J6 的有效俯仰分量投影到重力方向
//
//   更精确地: 腕部质心经过 J5 roll 后, 其高度对 J6 俯仰的偏导为:
//     ∂h_wrist/∂α₆ = (lcwx·cos α₆ + lcwz·sin α₆) · cos φ₅
//   即 J6 补偿项需乘以 cos φ₅.
// ============================================================================

/**  ---------- 物理参数 (需实机测量标定) ----------  **/

// 重力加速度 (m/s²)
static const float G = 9.81f;

// ── 长度参数 (m) ──────────────────────────────────
// 坐标系: 在对应连杆的本地坐标系中测量, Z 沿竖直, X 沿纵轴向前
static const float L2 = 0.3185f;  // J2 → J3 距离, 连杆 2 长度 (沿用实测值)
static const float L3 = 0.30f;    // J3 → J4 距离, 连杆 3 长度 (TODO: 需实测)
static const float L4 = 0.15f;    // J4 → J5 距离, 连杆 4 长度 (TODO: 需实测)

// ── 质量参数 (kg) ─────────────────────────────────
static const float M2 = 1.643f;   // 连杆 2 质量 (沿用实测值)
static const float M3 = 1.50f;    // 连杆 3 质量 (TODO: 旧版等效质量为 3.846,
                                  //   现拆分为 M3 + wrist, 需实测分离)
static const float M4 = 0.60f;    // 连杆 4 质量 (TODO: 需实测)
static const float MW = 2.50f;    // 腕部等效质量, 含 J5~J7 电机 + 夹爪 + 典型负载
                                  //   (TODO: 需实测, 含不同负载时可能需要动态更新)

// ── 质心位置 (m) ──────────────────────────────────
// 各质心在对应连杆本地坐标系中的坐标 [x, 0, z]
// X 沿连杆纵轴向前 (远离关节), Z 向上 (垂直于连杆纵轴)
//
// Link 2 质心 (在 {2} 系中):
static const float LC2X = 0.16f;     // X 分量: 沿连杆 2 纵轴, 距 J2 轴的距离 (沿用实测值)
static const float LC2Z = 0.0f;      // Z 分量: 连杆 2 通常对称, Z 偏移 ≈ 0

// Link 3 质心 (在 {3} 系中):
static const float LC3X = 0.12f;     // X 分量 (TODO: 沿用旧版 lc3x=0.119194 的近似值)
static const float LC3Z = -0.009f;   // Z 分量 (沿用旧版实测值, 质心略低于连杆中心线)

// Link 4 质心 (在 {4} 系中):
static const float LC4X = 0.07f;     // X 分量 (TODO: 需实测, 暂取 L4 的一半)
static const float LC4Z = 0.0f;      // Z 分量 (TODO: 需实测, 连杆 4 通常较对称)

// Wrist 质心 (在 wrist 系中, wrist 系与 {6} 系对齐, 位于 J6 之后):
static const float LCWX = 0.08f;     // X 分量: 腕部质心沿末端纵轴前伸 (TODO: 需实测)
static const float LCWZ = 0.02f;     // Z 分量: 腕部质心略高于末端中心线 (TODO: 需实测)

// ── 角度零位偏置 (度) ─────────────────────────────
// 电机读数 + 偏置 = 绝对俯仰角 (0° = 水平前伸)
// (TODO: 均需根据实际机械零位标定)
static const float J4_ZERO_OFFSET = 0.0f;   // J4: 电机角 → 绝对俯仰的偏置
static const float J5_ZERO_OFFSET = 0.0f;   // J5: 电机角 → roll 角的偏置 (0° = 腕部竖直)
static const float J6_ZERO_OFFSET = 0.0f;   // J6: 电机角 → 绝对俯仰的偏置


void GravityCompension_Calculate(float _J2, float _J3, float _J4, float _J5, float _J6,
                                  float *tau_J2, float *tau_J3, float *tau_J4, float *tau_J6)
{
    if (tau_J2 == nullptr || tau_J3 == nullptr || tau_J4 == nullptr || tau_J6 == nullptr)
    {
        return;
    }

    // ========================================================================
    // Step 1: 角度转换 — 电机读数 → 绝对俯仰角 / roll 角 (弧度)
    // ========================================================================

    // J2: 直接对应 (假设机械零位 = 水平前伸)
    const float a2 = _J2 * PI / 180.0f;

    // J3: 电机零位与水平前伸差 90°, 方向反转
    //     _J3 = -90° → a3 = 0° (水平前伸), _J3 = 0° → a3 = -90° (竖直下垂)
    const float a3 = -(_J3 + 90.0f) * PI / 180.0f;

    // J4: 绝对俯仰角 = 电机读数 + 零位偏置
    const float a4 = (_J4 + J4_ZERO_OFFSET) * PI / 180.0f;

    // J5: Roll 角 = 电机读数 + 零位偏置
    const float p5 = (_J5 + J5_ZERO_OFFSET) * PI / 180.0f;

    // J6: 绝对俯仰角 = 电机读数 + 零位偏置
    const float a6 = (_J6 + J6_ZERO_OFFSET) * PI / 180.0f;

    // ========================================================================
    // Step 2: 预计算三角函数
    // ========================================================================
    const float c2 = cosf(a2), s2 = sinf(a2);
    const float c3 = cosf(a3), s3 = sinf(a3);
    const float c4 = cosf(a4), s4 = sinf(a4);
    const float c6 = cosf(a6), s6 = sinf(a6);
    const float cp5 = cosf(p5);   // J5 roll 调制因子: cos(φ₅)

    // ========================================================================
    // Step 3: 各质量体对自身关节的力臂
    // ========================================================================
    // 力臂公式: arm = lcx·cos α + lcz·sin α
    //   含义: 质心在世界系 X₀ 轴上的投影距离 = 重力力臂
    //   当 α=0° (水平前伸): cos=1, sin=0 → arm = lcx (全长)
    //   当 α=90° (竖直向上): cos=0, sin=1 → arm = lcz (侧向偏移成为力臂)

    // Link 2: 质心在 {2} 系, 对 J2 的力臂
    const float arm2 = LC2X * c2 + LC2Z * s2;

    // Link 3: 质心在 {3} 系, 对 J3 的力臂
    const float arm3_J3 = LC3X * c3 + LC3Z * s3;
    // Link 3: 质心在 {3} 系, 对 J2 的力臂 (= L₂ 全长 + {3}系中质心投影)
    const float arm3_J2 = L2 * c2 + arm3_J3;

    // Link 4: 质心在 {4} 系, 对 J4 的力臂
    const float arm4_J4 = LC4X * c4 + LC4Z * s4;
    // Link 4: 对 J3 的力臂 (= L₃ 全长投影 + 对J4的力臂投影)
    const float arm4_J3 = L3 * c3 + arm4_J4;
    // Link 4: 对 J2 的力臂 (= L₂ 全长投影 + L₃ 全长投影 + 对J4的力臂投影)
    const float arm4_J2 = L2 * c2 + L3 * c3 + arm4_J4;

    // Wrist: 腕部质心在 wrist 系, 对 J6 的力臂
    //      乘以 cp5 = cos(φ₅) 是 J5 roll 调制效应:
    //      J5=0°  → cp5=1 → J6 俯仰全效 → 力矩最大
    //      J5=90° → cp5=0 → J6 在水平面内摆动 → 无重力贡献
    const float armW_J6 = (LCWX * c6 + LCWZ * s6) * cp5;

    // Wrist: 对 J4 的力臂 (= L₄ 全长投影 + 对J6的力臂)
    const float armW_J4 = L4 * c4 + armW_J6;
    // Wrist: 对 J3 的力臂 (= L₃ 全长投影 + L₄ 全长投影 + 对J6的力臂)
    const float armW_J3 = L3 * c3 + L4 * c4 + armW_J6;
    // Wrist: 对 J2 的力臂 (= L₂ 全长投影 + L₃ 全长投影 + L₄ 全长投影 + 对J6的力臂)
    const float armW_J2 = L2 * c2 + L3 * c3 + L4 * c4 + armW_J6;

    // ========================================================================
    // Step 4: 力矩合成 — τⱼ = Σ mᵢ · g · armᵢ_about_ⱼ
    // ========================================================================
    // 每个关节的力矩 = 该关节之后所有质量体的重力贡献之和
    // 因为重力势能 U = Σ mᵢ·g·hᵢ(α₂,α₃,α₄,α₆), 而 τⱼ = ∂U/∂αⱼ = Σ mᵢ·g·(∂hᵢ/∂αⱼ)
    // 其中 ∂hᵢ/∂αⱼ = armᵢ_about_ⱼ (链式法则: 每个力臂项都是其后的累积)

    // J6 (腕部 pitch): 仅 wrist 自身有贡献
    *tau_J6 = G * MW * armW_J6;

    // J4 (肘后 pitch): Link 4 + Wrist 的贡献
    *tau_J4 = G * (M4 * arm4_J4 + MW * armW_J4);

    // J3 (肘关节): Link 3 + Link 4 + Wrist 的贡献
    *tau_J3 = G * (M3 * arm3_J3 + M4 * arm4_J3 + MW * armW_J3);

    // J2 (肩关节): Link 2 + Link 3 + Link 4 + Wrist 的贡献
    *tau_J2 = G * (M2 * arm2 + M3 * arm3_J2 + M4 * arm4_J2 + MW * armW_J2);
}

void BackEMFCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *backEMF0, float *backEMF1, float *backEMF2, float *backEMF3, float *backEMF4, float *backEMF5)
{
    float _ke0;
    float _ke1;
    float _ke2;
    float _ke3;
    float _ke4;
    float _ke5;

    *backEMF0 = _ke0 * speed0;
    *backEMF1 = _ke1 * speed1;
    *backEMF2 = _ke2 * speed2;
    *backEMF3 = _ke3 * speed3;
    *backEMF4 = _ke4 * speed4;
    *backEMF5 = _ke5 * speed5;
}

void CoulombCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *coulomb0, float *coulomb1, float *coulomb2, float *coulomb3, float *coulomb4, float *coulomb5)
{
    float _coulomb0;
    float _coulomb1;
    float _coulomb2;
    float _coulomb3;
    float _coulomb4;
    float _coulomb5;

    if(speed0 > 0)
        *coulomb0 = _coulomb0;
    else if(speed0 < 0)
        *coulomb0 = -_coulomb0;
    else
        *coulomb0 = 0;

    if(speed1 > 0)
        *coulomb1 = _coulomb1;
    else if(speed1 < 0)
        *coulomb1 = -_coulomb1;
    else
        *coulomb1 = 0;

    if(speed2 > 0)
        *coulomb2 = _coulomb2;
    else if(speed2 < 0)
        *coulomb2 = -_coulomb2;
    else
        *coulomb2 = 0;

    if(speed3 > 0)
        *coulomb3 = _coulomb3;
    else if(speed3 < 0)
        *coulomb3 = -_coulomb3;
    else
        *coulomb3 = 0;

    if(speed4 > 0)
        *coulomb4 = _coulomb4;
    else if(speed4 < 0)
        *coulomb4 = -_coulomb4;
    else
        *coulomb4 = 0;

    if(speed5 > 0)
        *coulomb5 = _coulomb5;
    else if(speed5 < 0)
        *coulomb5 = -_coulomb5;
    else
        *coulomb5 = 0;

}

void ViscousCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *viscous0, float *viscous1, float *viscous2, float *viscous3, float *viscous4, float *viscous5)
{
    float _viscous0;
    float _viscous1;
    float _viscous2;
    float _viscous3;
    float _viscous4;
    float _viscous5;

    *viscous0 = _viscous0 * speed0;
    *viscous1 = _viscous1 * speed1;
    *viscous2 = _viscous2 * speed2;
    *viscous3 = _viscous3 * speed3;
    *viscous4 = _viscous4 * speed4;
    *viscous5 = _viscous5 * speed5;
}

void GripperGravityCompension_Calculate(float _J5, float *torque_J5)
{
    float gripper_pitch_theta = _J5 - 90.000f;

    float c5 = cos(gripper_pitch_theta);
    float s5 = sin(gripper_pitch_theta);

    float g;
    float m5;
    float xc5;
    float yc5;

    *torque_J5 = c5 * g * m5 * xc5 - g * m5 * s5 * yc5;
   
}

void InertiaCompension_Calculate(float _J2, float _J3, float Motor_Set_torque_J1, float motor_speed_j1, float *torque_J1)
{
    // J1惯量补偿：由J2/J3姿态决定等效转动惯量，再按当前J1控制输出进行前馈缩放。
    float theta2 = _J2 * PI / 180.0f;
    float theta3 = (_J3 + 90.0f) * PI / 180.0f;

    float c2 = cos(theta2);
    float s2 = sin(theta2);
    float c3 = cos(theta3);
    float s3 = sin(theta3);
    float c23 = cos(theta2 + theta3);
    float s23 = sin(theta2 + theta3);

    // 质心坐标
    float xc2 = 0.212567;
    float zc2 = 0.013110;
    float xc3 = 0.128812;
    float zc3 = -0.009699;

    float I2_xx = 0.001098159;
    float I2_zz = 0.007049618;
    float I2_xz = 0.000378630;
    float m2 = 0.552;
    float I3_xx = 0.018638961;
    float I3_zz = 0.043604552;
    float I3_xz = -0.007311936;
    float m3 = 2.957;

    // 质心到J1转轴方向的等效力臂
    float rho2;
    float rho3;

    // 平行轴分解：平动惯量项 + 重心处转动惯量项
    float I2_trans;
    float I2_rot;
    float I3_trans;
    float I3_rot;

    float I1_base = 0.010794353; // J1基座

    float l2 = 0.3185;

    float motor_i = 0.00256225;

    float set_torque;
    float Ieq;

    // 由电机速度微分得到J1角加速度（控制周期默认1ms）。
    static uint8_t speed_diff_inited = 0U;
    static float speed_prev = 0.0f;
    static float alpha_lpf = 0.0f;
    static uint32_t speed_tick_prev_ms = 0U;
    float speed_diff_dt = 0.001f;
    const float alpha_lpf_gain = 0.2f;
    float alpha_raw = 0.0f;
    float alpha_j1 = 0.0f;
    uint32_t speed_tick_now_ms = HAL_GetTick();

    if (speed_diff_inited == 0U)
    {
        speed_prev = motor_speed_j1;
        speed_diff_inited = 1U;
        alpha_lpf = 0.0f;
        speed_tick_prev_ms = speed_tick_now_ms;
    }
    else
    {
        uint32_t tick_delta_ms = 0U;
        if (speed_tick_now_ms >= speed_tick_prev_ms)
        {
            tick_delta_ms = speed_tick_now_ms - speed_tick_prev_ms;
        }
        speed_tick_prev_ms = speed_tick_now_ms;

        if (tick_delta_ms > 0U)
        {
            speed_diff_dt = (float)tick_delta_ms * 0.001f;
        }

        alpha_raw = (motor_speed_j1 - speed_prev) / speed_diff_dt;
        speed_prev = motor_speed_j1;
        alpha_lpf += alpha_lpf_gain * (alpha_raw - alpha_lpf);
    }
    alpha_j1 = alpha_lpf;

    rho2 = xc2 * c2 - zc2 * s2;
    I2_trans = m2 * rho2 * rho2;
    I2_rot = I2_xx * s2 * s2 + I2_zz * c2 * c2 - 2 * I2_xz * s2 * c2;

    rho3 = l2 * c2 + xc3 * c23 - zc3 * s23;
    I3_trans = m3 * rho3 * rho3;
    I3_rot = I3_xx * s23 * s23 + I3_zz * c23 * c23 - 2 * I3_xz * s23 * c23;

    if (fabsf(Motor_Set_torque_J1) < 0.1f && fabsf(alpha_j1) < 1e-3f)
    {
        set_torque = 0.0f;
    }
    else 
    {
        set_torque = Motor_Set_torque_J1;
    }

    Ieq = I2_rot + I2_trans + I3_rot + I3_trans + I1_base;

    // 输出的是给J1使用的惯量前馈项，后续在任务里还会乘以总增益做实机调参。
    *torque_J1 = alpha_j1 * Ieq;

}

/**
 * @brief J1转动惯量补偿验证（虚拟弹簧法，纯力矩模式）
 *
 * @details
 * 每个控制周期根据当前姿态计算 J1 总等效惯量 I_total，构造动态刚度
 * K_dynamic = spring_k * I_total，再按虚拟弹簧力矩
 * tau_out = -K_dynamic * (theta1_actual - theta1_center) 输出纯力矩命令。
 *
 * @param _J1 当前J1角度（度）
 * @param _J2 当前J2角度（度）
 * @param _J3 当前J3角度（度）
 * @param motor_speed_j1 当前J1速度
 * @param spring_k 动态刚度比例系数（建议初始 10）
 * @param torque_limit 力矩限幅（Nm）
 * @param speed_limit 速度阈值，超过后强制输出0
 * @param dt 控制周期（秒）
 * @param recenter_request 非0时将当前J1角度重新设为弹簧中心
 * @param cmd_torque_j1 输出：本周期下发给J1的力矩
 * @param inertia_total_j1 输出：当前姿态下估计的总等效惯量 I_total
 * @param k_dynamic_j1 输出：当前动态刚度 K_dynamic
 * @param theta1_center_deg 输出：当前弹簧中心角度（度）
 */
void InertiaCompension_ValidationStep(float _J1, float _J2, float _J3, float motor_speed_j1, float spring_k, float torque_limit, float speed_limit, float dt, uint8_t recenter_request, float *cmd_torque_j1, float *inertia_total_j1, float *k_dynamic_j1, float *theta1_center_deg)
{
    if (cmd_torque_j1 == nullptr || inertia_total_j1 == nullptr || k_dynamic_j1 == nullptr || theta1_center_deg == nullptr || dt <= 0.0f)
    {
        return;
    }

    static uint8_t center_inited = 0U;
    static float theta1_center = 0.0f;

    if (center_inited == 0U || recenter_request != 0U)
    {
        theta1_center = _J1;
        center_inited = 1U;
    }

    // 利用惯量补偿函数反推出姿态相关惯量项，再加上电机转子惯量。
    // 由于补偿函数有小力矩死区，这里使用 1.0Nm 探测力矩避免被死区截断。
    const float motor_i = 0.00256225f;
    const float probe_torque = 1.0f;
    float inertia_ff_probe = 0.0f;
    InertiaCompension_Calculate(_J2, _J3, probe_torque, motor_speed_j1, &inertia_ff_probe);

    const float inertia_link = inertia_ff_probe * motor_i / probe_torque;
    const float inertia_total = inertia_link + motor_i;
    const float k_dynamic = spring_k * inertia_total;

    const float theta_err_rad = (_J1 - theta1_center) * PI / 180.0f;
    float torque_out = -k_dynamic * theta_err_rad;

    if (fabsf(motor_speed_j1) > speed_limit)
    {
        torque_out = 0.0f;
    }
    else
    {
        if (torque_out > torque_limit)
        {
            torque_out = torque_limit;
        }
        else if (torque_out < -torque_limit)
        {
            torque_out = -torque_limit;
        }
    }

    *cmd_torque_j1 = torque_out;
    *inertia_total_j1 = inertia_total;
    *k_dynamic_j1 = k_dynamic;
    *theta1_center_deg = theta1_center;
}
