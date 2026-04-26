#include "DJI_motor.h"

void DJI_Motor_2006::bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort)
{
    this->motor_msg = _motor_msg;
    this->CAN_TxPort = _CAN_TxPort;
    // 对于DJI电机，不需要绑定发送队列，会在执行任务中放到统一一个包中对多个电机进行发送
}
void DJI_Motor_2006::setPolar(int polar)
{
    // 没找到：（
}
void DJI_Motor_2006::update(Motor &motor)
{
    // dji_motor->update(id, data);
    switch (status)
    {
    case MotorStatus::error:
        break;
    case MotorStatus::ready:
        motor.current_angle = dji_motor->getAngle();
        motor.current_speed = dji_motor->getSpeed();
        motor.current_torque = dji_motor->getTorqueCurrent() / 1000.0f * 0.18f; // 2006的力矩常数是0.18Nm/A，电流单位是mA，所以要除以1000
        break;
    default:
        for (;;)
            ;
        break;
    }
}
void DJI_Motor_2006::setTorque(float target_torque)
{
    dji_motor->Out = target_torque; // 包装can信息和发送队列放在外面

    // motor_dji::MotorMsgPack(this->motor_msg, *dji_motor);
    //  xQueueSend(this->CAN_TxPort, &this->motor_msg.Id2ff, 0);
}

void DJI_Motor_3508::bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort)
{
    this->motor_msg = _motor_msg;
    this->CAN_TxPort = _CAN_TxPort;
    // 对于DJI电机，不需要绑定发送队列，会在执行任务中放到统一一个包中对多个电机进行发送
}
void DJI_Motor_3508::setPolar(int polar)
{
    
}
void DJI_Motor_3508::update(Motor &motor)
{
    // dji_motor->update(id, data);
    switch (status)
    {
    case MotorStatus::error:
        break;
    case MotorStatus::ready:
        motor.current_angle = dji_motor->getAngle();
        motor.current_speed = dji_motor->getSpeed();
        motor.current_torque = dji_motor->getTorqueCurrent();
        break;
    default:
        for (;;)
            ;
        break;
    }
}
void DJI_Motor_3508::setTorque(float target_torque)
{
    dji_motor->Out = target_torque; // 包装can信息和发送队列放在外面
}
