#include "DM_motor.h"
#include "task.h"

namespace
{
constexpr uint32_t DM_FEEDBACK_TIMEOUT_MS = 100U;
}

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
    const TickType_t now_tick = xTaskGetTickCount();
    const uint32_t feedback_sequence = dm_motor->GetFeedbackSequence();

    if (feedback_sequence != last_feedback_sequence)
    {
        last_feedback_sequence = feedback_sequence;
        last_feedback_tick = now_tick;
        feedback_received = true;
    }

    const bool feedback_timed_out =
        !feedback_received ||
        ((now_tick - last_feedback_tick) > pdMS_TO_TICKS(DM_FEEDBACK_TIMEOUT_MS));
    const bool motor_ready =
        !feedback_timed_out && (dm_motor->GetMotorState() == 1);

    if (!motor_ready)
    {
        // 仅隔离当前掉线或报错的电机，其他电机的模式和目标不受影响。
        motor.forceSetMode(MotorMode::disabled);
        status = MotorStatus::error;
        return;
    }

    status = MotorStatus::ready;
    motor.current_angle = dm_motor->getRecData().angle;
    motor.current_speed = dm_motor->getRecData().velocity;
    motor.current_torque = dm_motor->getRecData().torque;
}
void DM_Motor::setTorque(float target_torque)
{
    dm_motor->setTorque(target_torque);
}
