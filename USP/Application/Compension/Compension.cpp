#include "Compension.h"

void GravityCompension_Calculate(float _J2, float _J3, float *torque_J2, float *torque_J3)
{
    if (torque_J2 == nullptr || torque_J3 == nullptr)
    {
        return;
    }

    const float theta2 = _J2 * PI / 180.0f;
    const float theta3 = -(_J3 + 90.0f) * PI / 180.0f;
    const float c2 = cosf(theta2);
    const float s2 = sinf(theta2);
    const float c23 = cosf(theta2 + theta3);
    const float s23 = sinf(theta2 + theta3);

    const float g = 9.81f;
    const float m2 = 1.643f;
    const float m3 = 3.846f;
    const float l2 = 0.3185f;
    const float xc2 = 0.16f;
    const float yc2 = 0.0f;
    const float xc3 = 0.119194f;
    const float yc3 = -0.008744f;

    const float arm2 = xc2 * c2 - yc2 * s2;
    const float arm3_about_j3 = xc3 * c23 - yc3 * s23;
    const float arm3_about_j2 = l2 * c2 + arm3_about_j3;

    *torque_J3 = g * m3 * arm3_about_j3;
    *torque_J2 = g * (m2 * arm2 + m3 * arm3_about_j2);
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
