#include "stdint.h"
#include "srml_std_lib.h"
#include "math.h"

void GravityCompension_Calculate(float _J2, float _J3, float *torque_J2, float *torque_J3);
void BackEMFCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *backEMF0, float *backEMF1, float *backEMF2, float *backEMF3, float *backEMF4, float *backEMF5);
void CoulombCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *coulomb0, float *coulomb1, float *coulomb2, float *coulomb3, float *coulomb4, float *coulomb5);
void ViscousCompension_Calculate(float speed0, float speed1, float speed2, float speed3, float speed4, float speed5, float *viscous0, float *viscous1, float *viscous2, float *viscous3, float *viscous4, float *viscous5);
void GripperGravityCompension_Calculate(float _J5, float *torque_J5);
void InertiaCompension_Calculate(float _J2, float _J3, float Motor_Rec_torque_J1, float motor_speed_j1, float *torque_J1);
void InertiaCompension_ValidationStep(float _J1, float _J2, float _J3, float motor_speed_j1, float spring_k, float torque_limit, float speed_limit, float dt, uint8_t recenter_request, float *cmd_torque_j1, float *inertia_total_j1, float *k_dynamic_j1, float *theta1_center_deg);
