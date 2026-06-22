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
static const float M4 = 0.60f;    // 连杆 4 质量: 结构件 4 + J5 电机 (TODO: 需实测)
static const float MW = 2.50f;    // 腕部等效质量: J6电机 + J7电机 + 夹爪 + 负载 (不含J5, J5已归M4)
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

// 注: MDH→本函数角度约定的转换全部由调用侧 MDH2xxx() 完成, 本函数不做偏置处理


void GravityCompension_Calculate(float _J2, float _J3, float _J4, float _J5, float _J6,
                                  float *tau_J2, float *tau_J3, float *tau_J4, float *tau_J6)
{
    if (tau_J2 == nullptr || tau_J3 == nullptr || tau_J4 == nullptr || tau_J6 == nullptr)
    {
        return;
    }

    // ========================================================================
    // Step 1: 输入角度 → 绝对俯仰角 / roll 角 (弧度)
    // ========================================================================
    //
    // 角度约定 (与惯量补偿完全一致):
    //   _J2: Link 2 的绝对俯仰角 (度), 0°=水平前伸, >0°=上仰
    //   _J3: Link 3 相对 Link 2 的俯仰角 (度), 0°=共线, >0°=Link3相对Link2上仰
    //   _J4: Link 4 相对 Link 3 的俯仰角 (度), 0°=共线, >0°=Link4相对Link3上仰
    //   _J5: J5 的 roll 角 (度), 0°=腕部竖直, >0°=从后方看顺时针
    //   _J6: Wrist 相对 Link 4 的俯仰角 (度), 0°=共线, >0°=Wrist相对Link4上仰
    //
    //   累加得绝对俯仰角 (相对于世界系 X₀Y₀ 平面):
    //     abs_i = abs_{i-1} + rel_i

    // J2: 绝对俯仰角
    const float a2 = _J2 * PI / 180.0f;

    // J3→J6: 相对角链式累加为绝对俯仰角
    const float a3 = (_J2 + _J3) * PI / 180.0f;                   // Link 3 绝对
    const float a4 = (_J2 + _J3 + _J4) * PI / 180.0f;             // Link 4 绝对

    // J5: roll 角 (独立, 不参与 pitch 累加)
    const float p5 = _J5 * PI / 180.0f;

    // J6: 绝对俯仰角 (wrist)
    const float a6 = (_J2 + _J3 + _J4 + _J6) * PI / 180.0f;

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
    // 力臂公式: arm = lcx·cos α - lcz·sin α
    //   含义: 质心在世界系 X₀ 轴上的投影距离 = 重力力臂
    //   当 α=0° (水平前伸): cos=1, sin=0 → arm = lcx (全长)
    //   当 α=90° (竖直向上): cos=0, sin=1 → arm = -lcz (lcz>0时质心在根部后方)

    // Link 2: 质心在 {2} 系, 对 J2 的力臂
    const float arm2 = LC2X * c2 - LC2Z * s2;

    // Link 3: 质心在 {3} 系, 对 J3 的力臂
    const float arm3_J3 = LC3X * c3 - LC3Z * s3;
    // Link 3: 质心在 {3} 系, 对 J2 的力臂 (= L₂ 全长 + {3}系中质心投影)
    const float arm3_J2 = L2 * c2 + arm3_J3;

    // Link 4: 质心在 {4} 系, 对 J4 的力臂
    const float arm4_J4 = LC4X * c4 - LC4Z * s4;
    // Link 4: 对 J3 的力臂 (= L₃ 全长投影 + 对J4的力臂投影)
    const float arm4_J3 = L3 * c3 + arm4_J4;
    // Link 4: 对 J2 的力臂 (= L₂ 全长投影 + L₃ 全长投影 + 对J4的力臂投影)
    const float arm4_J2 = L2 * c2 + L3 * c3 + arm4_J4;

    // Wrist: 腕部质心在 wrist 系, 对 J6 的力臂
    //      乘以 cp5 = cos(φ₅) 是 J5 roll 调制效应:
    //      J5=0°  → cp5=1 → J6 俯仰全效 → 力矩最大
    //      J5=90° → cp5=0 → J6 在水平面内摆动 → 无重力贡献
    const float armW_J6 = (LCWX * c6 - LCWZ * s6) * cp5;

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

// ============================================================================
// 7-DOF J1 转动惯量补偿 — 平行轴定理 + 惯性张量投影 (2025)
// ============================================================================
//
// 坐标系定义 (详见 Compension.h)
// ─────────────────────────────────
//   世界系 {0}:  Z₀ ↑ (J1 旋转轴, 竖直向上)
//                X₀ → (前方, J2=0° 时伸展方向)
//                Y₀ ⊗ (垂直于 sagittal 平面)
//
//   各连杆坐标系 {i}: Xᵢ 沿连杆纵轴向前, Zᵢ 沿连杆"向上" (垂直于纵轴)
//   旋转矩阵 Ry(θ) 绕 Y₀ 旋转(pitch):
//         [cos θ   0   sin θ]
//   Ry =  [  0     1     0  ]
//         [-sin θ  0   cos θ]
//
//   绕 J1 (Z₀) 的转动惯量:
//     I_about_Z0 = [0,0,1] · (R·I_local·Rᵀ) · [0,0,1]ᵀ
//     展开为: I_xx·sin²θ_abs + I_zz·cos²θ_abs - 2·I_xz·sinθ_abs·cosθ_abs
//     其中 θ_abs 为连杆的绝对俯仰角 (相对于世界系 X₀Y₀ 平面).
//
// 推导方法
// ────────
//   1. 平行轴定理: 将每个质量体的惯量分解为
//        I_trans = m · ρ²          (质心平动项, ρ 为质心到 Z₀ 的水平距离)
//        I_rot   = 惯性张量投影     (连杆本体绕 Z₀ 的分量)
//   2. 总等效惯量 Ieq = I1_base + Σ (I_trans_i + I_rot_i)
//   3. 惯量前馈力矩 = Ieq · α_J1 (α_J1 为 J1 角加速度)
//
// J5 Roll 对腕部质心水平距离的影响
// ────────────────────────────────
//   腕部质心在 wrist 系中: ʳcw = [lcwx, 0, lcwz]ᵀ (Y 向对称, lcwy≈0)
//   经 J5 roll φ₅ 后, lcwz 分量在 YZ 平面内旋转:
//     世界系贡献: ΔY = lcwz · sin φ₅
//   因此 ρ²_wrist = ρ²_x + (lcwz·sin φ₅)²
//   其中 ρ_x 为 sagittal 平面内 pitch 关节决定的水平距离.
//
//   当 J5=0° (腕部竖直): sin φ₅=0 → 纯 sagittal 平面内, Y 贡献=0
//   当 J5=±90° (腕部水平): sin φ₅=±1 → lcwz 产生 Y 向偏移 → ρ² 增大
//   (此效应在臂折叠时最显著, 因为 ρ_x 此时较小)
// ============================================================================

/**  ---------- 物理参数 (需实机测量标定) ----------  **/

// ── 长度参数 (m) ──────────────────────────────────
// 坐标系: 沿连杆纵轴 (Xᵢ 方向) 测量
static const float I_L2 = 0.3185f;   // J2 → J3 距离 (沿用实测值)
static const float I_L3 = 0.30f;     // J3 → J4 距离 (TODO: 需实测)
static const float I_L4 = 0.15f;     // J4 → J5 距离 (TODO: 需实测)

// ── 质量参数 (kg) ─────────────────────────────────
// 7-DOF 模型: 将 m3 拆分为 link3 + link4 + wrist
static const float I_M2 = 0.552f;    // 连杆 2 质量 (沿用实测值)
static const float I_M3 = 1.00f;     // 连杆 3 质量 (TODO: 需实测, 旧版等效 m3=2.957 拆分)
static const float I_M4 = 0.50f;     // 连杆 4 质量: 结构件 4 + J5 电机 (TODO: 需实测)
static const float I_MW = 1.457f;    // 腕部等效质量: J6电机+J7电机+夹爪+负载 (不含J5)
                                      //   (TODO: 需实测, I_M3+I_M4+I_MW 应 ≈ 2.957)

// ── 质心位置 (m), 在各连杆本地坐标系中 ─────────────
// X 沿连杆纵轴向前 (远离关节), Z 垂直于连杆纵轴向上
//
// Link 2 质心 (在 {2} 系中):
static const float I_XC2 = 0.212567f;  // X₂: 距 J2 轴沿连杆方向 (沿用实测值)
static const float I_ZC2 = 0.013110f;  // Z₂: 垂直于连杆, 质心略偏上 (沿用实测值)

// Link 3 质心 (在 {3} 系中):
static const float I_XC3 = 0.12f;      // X₃: 距 J3 轴沿连杆方向 (TODO: 沿用旧版近似值 0.128812)
static const float I_ZC3 = -0.0097f;   // Z₃: 质心略偏下 (沿用旧版实测值 -0.009699)

// Link 4 质心 (在 {4} 系中):
static const float I_XC4 = 0.07f;      // X₄: 距 J4 轴 (TODO: 需实测, 暂取 L4/2)
static const float I_ZC4 = 0.0f;       // Z₄: 连杆 4 通常对称 (TODO: 需实测)

// Wrist 质心 (在 wrist 系中, 与 {6} 系对齐, 位于 J6 之后):
static const float I_XCW = 0.08f;      // Xw: 腕部质心沿末端纵轴前伸 (TODO: 需实测)
static const float I_ZCW = 0.02f;      // Zw: 腕部质心略高于末端中心线 (TODO: 需实测)

// ── 惯性张量 (kg·m²), 在各连杆本地坐标系中 ─────────
// 格式: [I_xx, 0, I_xz; 0, I_yy, 0; I_xz, 0, I_zz]
// Y 向对称 → I_xy = I_yz = 0, I_yy 对绕 Z₀ 投影无贡献
//
// Link 2 惯性张量 (在 {2} 系中, 沿用实测值):
static const float I2_XX = 0.001098159f;
static const float I2_ZZ = 0.007049618f;
static const float I2_XZ = 0.000378630f;

// Link 3 惯性张量 (在 {3} 系中):
//   (TODO: 旧版等效张量 I3_xx=0.018639, I3_zz=0.043605, I3_xz=-0.007312
//    含后续所有质量. 新版应为仅连杆 3 本体的张量, 需重新测量)
static const float I3_XX = 0.008f;     // (TODO: 需实测, 暂取旧版 ~40%)
static const float I3_ZZ = 0.020f;     // (TODO: 需实测)
static const float I3_XZ = -0.003f;    // (TODO: 需实测)

// Link 4 惯性张量 (在 {4} 系中):
static const float I4_XX = 0.003f;     // (TODO: 需实测, 连杆 4 较短, 惯量较小)
static const float I4_ZZ = 0.008f;     // (TODO: 需实测)
static const float I4_XZ = 0.0f;       // (TODO: 需实测, 近似对称)

// Wrist 惯性张量 (在 wrist 系中):
//   (TODO: 需实测, 腕部质量集中, 惯量主要由夹爪+负载的延伸决定)
static const float IW_XX = 0.005f;     // (TODO: 需实测)
static const float IW_ZZ = 0.012f;     // (TODO: 需实测)
static const float IW_XZ = 0.0f;       // (TODO: 需实测, 近似对称)

// ── J1 基座惯量 (kg·m²) ──────────────────────────
// 包含 J1 电机转子 + 基座结构件绕 Z₀ 的固定惯量
static const float I1_BASE = 0.010794353f;  // J1基座惯量: J1转子 + 旋转结构件 + J2电机定子
static const float MOTOR_I_J1 = 0.00256225f; // J1 电机转子惯量 (已含在 I1_BASE 中, 此处保留供 ValidationStep 使用)

// 注: MDH→本函数角度约定的转换全部由调用侧 MDH2xxx() 完成, 本函数不做偏置处理


void InertiaCompension_Calculate(float _J2, float _J3, float _J4, float _J5, float _J6,
                                  float Motor_Set_torque_J1, float motor_speed_j1, float *torque_J1)
{
    if (torque_J1 == nullptr)
    {
        return;
    }

    // ========================================================================
    // Step 1: 输入角度 → 绝对俯仰角 / roll 角 (弧度)
    // ========================================================================
    //
    // ╔══════════════════════════════════════════════════════════════════════╗
    // ║  角度参数约定 (调用侧必须在传参前完成 MDH→本函数约定的转换)        ║
    // ╠══════════════════════════════════════════════════════════════════════╣
    // ║                                                                    ║
    // ║  坐标系: 世界系 Z₀↑, X₀→前, Y₀⊗                                    ║
    // ║                                                                    ║
    // ║  _J2 — Link 2 的绝对俯仰角 (度)                                    ║
    // ║        0° = 连杆 2 水平前伸 (∥ X₀Y₀ 平面)                          ║
    // ║        >0° = 上仰 (末端高于根部, 绕 +Y₀ 右手定则)                   ║
    // ║        <0° = 下俯                                                  ║
    // ║        转换: _J2 = abs_pitch_J2_deg                                ║
    // ║               (调用侧从 MDH J2 角换算得到)                          ║
    // ║                                                                    ║
    // ║  _J3 — Link 3 相对 Link 2 的俯仰角 (度), 与重力补偿的 _J3 约定一致   ║
    // ║        0° = Link 3 与 Link 2 共线                                    ║
    // ║        >0° = Link 3 相对 Link 2 上仰 (绕 +Y₀ 右手定则)               ║
    // ║        内部: abs_t3 = (_J2 + _J3) · π/180                           ║
    // ║        转换: _J3 = rel_pitch_J3_deg                                  ║
    // ║               (调用侧从 MDH J3 角换算得到)                            ║
    // ║                                                                    ║
    // ║  _J4 — Link 4 相对 Link 3 的俯仰角 (度), 与重力补偿的 _J4 约定一致   ║
    // ║        0° = Link 4 与 Link 3 共线                                    ║
    // ║        >0° = Link 4 相对 Link 3 上仰 (绕 +Y₀ 右手定则)               ║
    // ║        内部: abs_t4 = (_J2+_J3+_J4) · π/180                         ║
    // ║        转换: _J4 = rel_pitch_J4_deg                                  ║
    // ║               (调用侧从 MDH J4 角换算得到, 注意 k=-1 方向反转)       ║
    // ║                                                                    ║
    // ║  _J5 — J5 的 roll 角 (度), 绕连杆纵轴 (≈X₄) 的旋转                  ║
    // ║        _J5 = 0° → 腕部竖直朝上, J6 pitch 轴 ∥ Y₀                    ║
    // ║        >0° → 从后方看向末端, 顺时针旋转为正                          ║
    // ║        _J5 = ±90° → 腕部水平                                        ║
    // ║        转换: _J5 = roll_J5_deg                                      ║
    // ║               (调用侧从 MDH J5 角换算得到, 注意 k=-1 方向反转)       ║
    // ║                                                                    ║
    // ║  _J6 — Wrist 相对 Link 4 的俯仰角 (度), 与重力补偿的 _J6 约定一致   ║
    // ║        0° = Wrist 与 Link 4 共线                                     ║
    // ║        >0° = Wrist 相对 Link 4 上仰 (绕 +Y₀ 右手定则)               ║
    // ║        内部: abs_tw = (_J2+_J3+_J4+_J6) · π/180                     ║
    // ║        转换: _J6 = rel_pitch_J6_deg                                  ║
    // ║               (调用侧从 MDH J6 角换算得到, 注意 k=-1 方向反转)       ║
    // ║                                                                    ║
    // ║     调用侧负责: MDH关节角 → 上述约定角度的转换                       ║
    // ║     本函数不做任何 MDH 相关的偏置处理                                ║
    // ╚══════════════════════════════════════════════════════════════════════╝

    // 角度累加: J2 为绝对俯仰, J3/J4/J6 为相对俯仰 (与重力补偿完全一致)
    //   abs_i = abs_{i-1} + rel_i

    // J2: 绝对俯仰角
    const float abs_t2 = _J2 * PI / 180.0f;

    // J3→J6: 相对角链式累加为绝对俯仰角
    const float abs_t3 = (_J2 + _J3) * PI / 180.0f;                 // Link 3 绝对
    const float abs_t4 = (_J2 + _J3 + _J4) * PI / 180.0f;           // Link 4 绝对

    // J5: roll 角 (独立, 不参与 pitch 累加)
    const float p5 = _J5 * PI / 180.0f;

    // J6: 绝对俯仰角 (wrist)
    const float abs_tw = (_J2 + _J3 + _J4 + _J6) * PI / 180.0f;

    // ========================================================================
    // Step 2: 预计算三角函数
    // ========================================================================
    const float c2  = cosf(abs_t2),  s2  = sinf(abs_t2);
    const float c3  = cosf(abs_t3),  s3  = sinf(abs_t3);
    const float c4  = cosf(abs_t4),  s4  = sinf(abs_t4);
    const float cw  = cosf(abs_tw),  sw  = sinf(abs_tw);
    const float sp5 = sinf(p5);   // J5 roll: sin(φ₅), 用于腕部 Y 偏移

    // ========================================================================
    // Step 3: 平动惯量 — 各质量体质心到 J1 轴 (Z₀) 的水平距离 ρ
    // ========================================================================
    // 公式: ρ = |世界系 X₀ 分量|, 由旋转矩阵投影得到:
    //   X₀ = 连杆长度累积投影 + 质心 X·cos(θ_abs) - 质心 Z·sin(θ_abs)
    //
    // 符号约定: 力臂公式 arm = x·cosθ - z·sinθ 统一使用减号
    //   (上仰为正, 旋转矩阵 Ry(-θ) 展开)
    //   代码: ρ = L_accum·cos + xc·cos - zc·sin
    //     (与重力补偿一致: arm = lcx·cos α - lcz·sin α)

    // Link 2: 质心到 J1 的水平距离
    const float rho2 = I_XC2 * c2 - I_ZC2 * s2;
    //   当 t2=0° (水平前伸): rho2 = xc2 (质心全在前方)
    //   当 t2=90° (竖直向上): rho2 = -zc2 (zc2>0时质心在根部后方)

    // Link 3: 质心到 J1 的水平距离
    //   = J2 全长投影 + 质心在 {3} 系中的投影
    const float rho3 = I_L2 * c2 + I_XC3 * c3 - I_ZC3 * s3;

    // Link 4: 质心到 J1 的水平距离
    //   = J2 全长投影 + J3 全长投影 + 质心在 {4} 系中的投影
    const float rho4 = I_L2 * c2 + I_L3 * c3 + I_XC4 * c4 - I_ZC4 * s4;

    // Wrist: 质心到 J1 的水平距离 (含 J5 roll 调制)
    //   Sagittal 平面内分量:
    const float rhoW_x = I_L2 * c2 + I_L3 * c3 + I_L4 * c4 + I_XCW * cw - I_ZCW * sw;
    //   J5 roll 引入的 Y 向偏移: 腕部质心 Z 分量经 roll 旋转后在 Y₀ 方向的投影
    const float rhoW_y = I_ZCW * sp5;  // lcwz·sin(φ₅), 当 φ₅=0 时此项为 0
    //   总水平距离平方: ρ² = ρ_x² + ρ_y²
    const float rhoW_sq = rhoW_x * rhoW_x + rhoW_y * rhoW_y;

    // 平动惯量 = m · ρ²
    const float I2_trans = I_M2 * rho2 * rho2;
    const float I3_trans = I_M3 * rho3 * rho3;
    const float I4_trans = I_M4 * rho4 * rho4;
    const float IW_trans = I_MW * rhoW_sq;

    // ========================================================================
    // Step 4: 转动惯量 — 惯性张量向 Z₀ 轴投影
    // ========================================================================
    // 公式: I_rot(θ) = I_xx·sin²θ + I_zz·cos²θ - 2·I_xz·sinθ·cosθ
    //
    // 推导: 连杆本地惯性张量 I_local, 经 Ry(θ) 旋转到世界系后,
    //       绕 Z₀ 轴的转动惯量 = e_zᵀ · (R·I·Rᵀ) · e_z
    //       = [0,0,1] · [c 0 s; 0 1 0; -s 0 c] · [Ixx 0 Ixz; 0 Iyy 0; Ixz 0 Izz]
    //         · [c 0 -s; 0 1 0; s 0 c] · [0,0,1]ᵀ
    //       = I_xx·s² + I_zz·c² - 2·I_xz·s·c
    //
    // 物理含义:
    //   θ=0° (水平前伸): I_rot = I_zz (连杆 Z 轴正对世界 Z₀)
    //   θ=90° (竖直向上): I_rot = I_xx (连杆 X 轴正对世界 Z₀)
    //   交叉项 I_xz 在中间姿态提供平滑过渡

    // Link 2
    const float I2_rot = I2_XX * s2 * s2 + I2_ZZ * c2 * c2 - 2.0f * I2_XZ * s2 * c2;

    // Link 3
    const float I3_rot = I3_XX * s3 * s3 + I3_ZZ * c3 * c3 - 2.0f * I3_XZ * s3 * c3;

    // Link 4
    const float I4_rot = I4_XX * s4 * s4 + I4_ZZ * c4 * c4 - 2.0f * I4_XZ * s4 * c4;

    // Wrist: 近似使用 wrist 系惯性张量 + 绝对俯仰角投影
    //   (J5 roll 会改变 wrist 系相对于世界系的姿态, 使投影更复杂.
    //    但因 wrist 本体惯量小, 且平动项 ρ² 占主导, 此近似误差可接受.)
    const float IW_rot = IW_XX * sw * sw + IW_ZZ * cw * cw - 2.0f * IW_XZ * sw * cw;

    // ========================================================================
    // Step 5: 总等效惯量 Ieq
    // ========================================================================
    // Ieq = J1 基座 + 各质量体的 (平动 + 转动) 贡献
    const float Ieq = I1_BASE
                    + I2_trans + I2_rot
                    + I3_trans + I3_rot
                    + I4_trans + I4_rot
                    + IW_trans + IW_rot;

    // ========================================================================
    // Step 6: J1 角加速度估计 (速度数值微分 + 低通滤波)
    // ========================================================================
    // 从 J1 电机速度通过数值微分得到角加速度 α_J1,
    // 再经一阶低通滤波抑制微分噪声.

    static uint8_t  speed_diff_inited = 0U;
    static float    speed_prev       = 0.0f;
    static float    alpha_lpf        = 0.0f;
    static uint32_t speed_tick_prev_ms = 0U;

    float speed_diff_dt  = 0.001f;           // 默认 1ms 控制周期
    const float ALPHA_LPF_GAIN = 0.2f;       // 低通滤波增益 (0~1, 越小越平滑)
    float alpha_raw  = 0.0f;
    float alpha_j1   = 0.0f;
    uint32_t speed_tick_now_ms = HAL_GetTick();

    if (speed_diff_inited == 0U)
    {
        // 首次调用: 初始化历史值, 不输出加速度
        speed_prev = motor_speed_j1;
        speed_diff_inited = 1U;
        alpha_lpf = 0.0f;
        speed_tick_prev_ms = speed_tick_now_ms;
    }
    else
    {
        // 计算实际时间间隔 (处理定时器溢出)
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

        // 数值微分
        alpha_raw = (motor_speed_j1 - speed_prev) / speed_diff_dt;
        speed_prev = motor_speed_j1;

        // 一阶低通滤波: α_lpf ← α_lpf + gain · (α_raw - α_lpf)
        alpha_lpf += ALPHA_LPF_GAIN * (alpha_raw - alpha_lpf);
    }
    alpha_j1 = alpha_lpf;

    // ========================================================================
    // Step 7: 死区保护 — 静止时关闭惯量前馈
    // ========================================================================
    // 当 J1 几乎静止且 PID 输出很小时, 关闭惯量前馈,
    // 避免速度微分噪声和极小转矩指令下的不必要补偿.
    //
    // 注意: 调用侧还会乘以调参增益 inertia_J1 (=0.5) 做最终缩放.

    if (fabsf(Motor_Set_torque_J1) < 0.1f && fabsf(alpha_j1) < 1e-3f)
    {
        *torque_J1 = 0.0f;
    }
    else
    {
        *torque_J1 = alpha_j1 * Ieq;
    }
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
void InertiaCompension_ValidationStep(float _J1, float _J2, float _J3, float _J4, float _J5, float _J6, float motor_speed_j1, float spring_k, float torque_limit, float speed_limit, float dt, uint8_t recenter_request, float *cmd_torque_j1, float *inertia_total_j1, float *k_dynamic_j1, float *theta1_center_deg)
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
    InertiaCompension_Calculate(_J2, _J3, _J4, _J5, _J6, probe_torque, motor_speed_j1, &inertia_ff_probe);

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
