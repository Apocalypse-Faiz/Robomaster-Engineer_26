#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include "SRML.h"
#include "Motor.h"

class DJI_Motor_2006 : public Motor_
{
private:
    Motor_C610 *dji_motor = nullptr;

public:
    explicit DJI_Motor_2006(Motor_C610 *dji_motor)
    {
        if (dji_motor)
        {
            this->dji_motor = dji_motor;
        }
        status = MotorStatus::ready; // 默认是连接正常的
    }

    void bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort);
    void setPolar(int polar);
    void update(Motor &motor) override;
    void setTorque(float target_torque) override;
    Motor_CAN_COB motor_msg;
    QueueHandle_t CAN_TxPort;
};

class DJI_Motor_3508 : public Motor_
{
private:
    Motor_C620 *dji_motor = nullptr;
public:
    explicit DJI_Motor_3508(Motor_C620 *dji_motor)
    {
        if (dji_motor)
        {
            this->dji_motor = dji_motor;
        }
        status = MotorStatus::ready; // 默认是连接正常的
    }
    void bindRxTxQueueHandle(Motor_CAN_COB &_motor_msg, QueueHandle_t &_CAN_TxPort);
    void setPolar(int polar);
    void update(Motor &motor) override;
    void setTorque(float target_torque) override;
    Motor_CAN_COB motor_msg;
    QueueHandle_t CAN_TxPort;
};

#endif
