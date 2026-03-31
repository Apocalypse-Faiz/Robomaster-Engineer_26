#include "LK_motor.h"

void LK_Motor::bindRxTxQueueHandle(QueueHandle_t &_CAN_TxPort)
{
    this->lk_motor->init(_CAN_TxPort);
}
void LK_Motor::setOffset(float _offset)
{
    setAngleOffset(_offset);
}
void LK_Motor::setPolar(int polar)
{
    // 极性设置在瓴控上位机中设置
}
void LK_Motor::update(Motor &motor)
{
    // lk_motor->update(id, data);
    switch (status)
    {
    case MotorStatus::error:
        if (lk_motor->getData().encoder != 0.0f)
        {
            status = MotorStatus::ready;
            motor.current_angle = MultiTurnRecord(lk_motor->getData().encoder * RATIO_ENCODER_TO_ANGLE); // 本地多圈
            motor.current_speed = lk_motor->getData().speed * RATIO_ENCODER_TO_ANGLE;
            motor.current_torque = lk_motor->getData().current; // 并不是力矩而是电流
        }
        break;
    case MotorStatus::ready:
        motor.current_angle = MultiTurnRecord(lk_motor->getData().encoder * RATIO_ENCODER_TO_ANGLE);
        motor.current_speed = lk_motor->getData().speed * RATIO_ENCODER_TO_ANGLE;
        motor.current_torque = lk_motor->getData().current; // 并不是力矩而是电流
        break;
    default:
        for (;;)
            ;
        break;
    }
}
void LK_Motor::setTorque(float target_torque)
{
    if (motor_type == 0)
    {
        lk_motor->iqCloseControl(target_torque);
    }
    else if (motor_type == 1)
    {
        lk_motor->iqOpenControl(target_torque);
    }
}

void LK_Motor::enableMotor()
{
    vTaskDelay(2);
    lk_motor->cleanErrorState();
    vTaskDelay(2);
    lk_motor->startMotor();
}
