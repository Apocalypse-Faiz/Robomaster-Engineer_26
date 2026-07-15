#include "DM_motor.h"

void DM_Motor::bindTxQueueHandle(QueueHandle_t _txQueue)
{
    this->dm_motor->bindCanQueueHandle(_txQueue);
}
void DM_Motor::setPolar(int polar)
{
    // 达妙电机库貌似没有这个功能
}
void DM_Motor::clearError()
{
    dm_motor->ClearError();
}

void DM_Motor::startMotor()
{
    dm_motor->startMotor();
}

void DM_Motor::update(Motor &motor)
{
    // dm_motor->update(id, data);
    switch (status)
    {
    case MotorStatus::error:
        // if (dm_motor->getRecData().angle != 0)
        if (dm_motor->GetMotorState() == 1)
        {
            status = MotorStatus::ready;
            motor.current_angle = dm_motor->getRecData().angle;
            motor.current_speed = dm_motor->getRecData().velocity;
            motor.current_torque = dm_motor->getRecData().torque;
        }
        break;
    case MotorStatus::ready:
        if(dm_motor->GetMotorState() != 1)
        {
            motor.forceSetMode(MotorMode::disabled);
            status = MotorStatus::error;
            break;
        }
        motor.current_angle = dm_motor->getRecData().angle;
        motor.current_speed = dm_motor->getRecData().velocity;
        motor.current_torque = dm_motor->getRecData().torque;
        break;
    default:
        for (;;)
            ;
        break;
    }
}
void DM_Motor::setTorque(float target_torque)
{
    dm_motor->setTorque(target_torque);
}
