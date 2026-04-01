#include "Compension.h"

void GravityCompension_Calculate(float _J2, float _J3, float *torque_J2, float *torque_J3)
{
    float theta2 = _J2 * PI / 180.0f;
    float theta3 = (_J3 - 90.0f) * PI / 180.0f;

    float c2 = cos(theta2);
    float s2 = sin(theta2);
    float c23 = cos(theta2 + theta3);
    float s23 = sin(theta2 + theta3);
    float c3 = cos(theta3);
    float s3 = sin(theta3);

    float g = 9.81;
    float m2 = 2.0; 
    float m3 = 2.858;  
    float xc3 = 0.14174;
    float yc3 = 0.00069;
    float xc2 = 0.19;
    float a2 = 0.38;

    *torque_J2 = c2 * g * m2 * xc2 + c23 * g * m3 * xc3 - g * m3 * s23 * yc3 + a2 * c3 * c23 * g * m3 + a2 * g * m3 * s3 * s23;
    *torque_J3 = c23 * g * m3 * xc3 - g * m3 * s23 * yc3;
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

void InertiaCompension_Calculate(float _J2, float _J3, float Motor_Set_torque_J1, float *torque_J1)
{
    float theta2 = _J2 * PI / 180.0f;
    float theta3 = (_J3 - 90.0f) * PI / 180.0f;

    float c2 = cos(theta2);
    float s2 = sin(theta2);
    float c3 = cos(theta3);
    float s3 = sin(theta3);
    float c23 = cos(theta2 + theta3);
    float s23 = sin(theta2 + theta3);

    // 质心坐标
    float xc2 = 0.19;
    float zc2 = 0;
    float xc3 = 0.14174;
    float zc3 = 0.00069;

    float I2_xx = 0.074343;
    float I2_zz = 0.036804;
    float I2_xz = 0.007355;
    float m2 = 2.0;
    float I3_xx = 0.015646;
    float I3_zz = 0.061199;
    float I3_xz = 0.011734;
    float m3 = 2.858;

    // 质心到参考系旋转轴的距离
    float rho2;
    float rho3;

    // 平动惯量与自身转动惯量
    float I2_trans;
    float I2_rot;
    float I3_trans;
    float I3_rot;

    float l2 = 0.38;
    float motor_i = 0.00256225;

    float set_torque;

    rho2 = xc2 * c2 - zc2 * s2;
    I2_trans = m2 * rho2 * rho2;
    I2_rot = I2_xx * s2 * s2 + I2_zz * c2 * c2 - 2 * I2_xz * s2 * c2;

    rho3 = l2 * c2 + xc3 * c23 - zc3 * s23;
    I3_trans = m3 * rho3 * rho3;
    I3_rot = I3_xx * s23 * s23 + I3_zz * c23 * c23 - 2 * I3_xz * s23 * c23;

    if (fabsf(Motor_Set_torque_J1) < 0.1)
    {
        set_torque = 0.0f;
    }
    else 
    {
        set_torque = Motor_Set_torque_J1;
    }

    *torque_J1 = set_torque / motor_i * (I2_rot + I2_trans + I3_rot + I3_trans);

}
