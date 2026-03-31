#include "Servo.h"

void Servo_Classdef::PWM_Start()
{
    HAL_TIM_PWM_Start(htim, Channel);
}
// 停止PWM的输出
void Servo_Classdef::PWM_Stop()
{
    HAL_TIM_PWM_Stop(htim, Channel);
}
void Servo_Classdef::setTarget(float target)
{
    if (target < 0 || target > Angle_Max)
        return; // 目标角度不在合理范围内
    /*目标角度转换成对应定时器PWM值*/
    float A = (Duty_Max - Duty_Min) / Angle_Max;
    PWM_Val = (unsigned int)((A * target + Duty_Min) * Tim_Period);

    // HAL_TIM_PWM_Start(htim, Channel);
    __HAL_TIM_SetCompare(htim, Channel, PWM_Val); // PWM输出
}
void Servo_Classdef::setPWMValue(uint32_t value)
{
    PWM_Val = value;
    __HAL_TIM_SetCompare(htim, Channel, PWM_Val); // PWM输出
}

void Servo_Control_example()
{
    // 变量创建实例： 20000为ARR+1  360.0f就是你控制的角度范围 周期为20ms，而控制范围只在0.5ms~2.5ms之间，所以占空比的范围也只是0.025~0.125之间，20*0.025=0.5
    // Servo_Classdef servo_1(&htim3, TIM_CHANNEL_4, 20000, 0.025, 0.125, 360.0f);

    // 操控舵机，输入的target_angle在上面输入的0~360度之内
    //  float target_angle;
    //  servo_1.setTarget(target_angle);
    //  servo_1.PWM_Start();
}
