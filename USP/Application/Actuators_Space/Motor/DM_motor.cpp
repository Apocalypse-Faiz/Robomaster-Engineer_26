#include "DM_motor.h"
#include "task.h"

namespace
{
constexpr uint32_t DM_FEEDBACK_TIMEOUT_MS = 100U;
constexpr float DM_EQUIVALENT_TURN_DEG = 360.0f * PI / 3.125f;

float AlignDmAngleToReference(float raw_angle, float reference_angle)
{
    const float branch_f =
        (reference_angle - raw_angle) / DM_EQUIVALENT_TURN_DEG;
    const int branch = (branch_f >= 0.0f)
        ? static_cast<int>(branch_f + 0.5f)
        : static_cast<int>(branch_f - 0.5f);

    return raw_angle + branch * DM_EQUIVALENT_TURN_DEG;
}
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
    const bool has_new_feedback =
        (feedback_sequence != last_feedback_sequence);

    if (has_new_feedback)
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
    if (has_new_feedback)
    {
        float current_angle = dm_motor->getRecData().angle;
        if (angle_initialized)
        {
            // DM 重新上电后可能切换到相差约 361.91° 的等价位置分支。
            // 始终对齐到上一帧可信角度附近，避免运行期零点跳圈。
            current_angle = AlignDmAngleToReference(current_angle, last_valid_angle);
        }

        motor.current_angle = current_angle;
        motor.current_speed = dm_motor->getRecData().velocity;
        motor.current_torque = dm_motor->getRecData().torque;
        last_valid_angle = current_angle;
        angle_initialized = true;
    }
}
void DM_Motor::setTorque(float target_torque)
{
    dm_motor->setTorque(target_torque);
}
