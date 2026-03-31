#ifndef __DM_MOTOR_H__
#define __DM_MOTOR_H__

#include "SRML.h"
#include "Motor.h"

class DM_Motor : public Motor_
{
private:
    Motor_DM_classdef *dm_motor = nullptr;

public:
    explicit DM_Motor(Motor_DM_classdef *dm_motor)
    {
        if (dm_motor)
            this->dm_motor = dm_motor;
        status = MotorStatus::error;
    }
    ~DM_Motor() override
    {
    }
    void bindTxQueueHandle(QueueHandle_t _txQueue);
    void setPolar(int polar);
    void enableMotor();
    void update(Motor &motor) override;
    void setTorque(float target_torque) override;
};

#endif
