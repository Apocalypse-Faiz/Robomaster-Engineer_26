#ifndef __LK_MOTOR_H__
#define __LK_MOTOR_H__

#include "SRML.h"
#include "Motor.h"

/**
 * @brief 自定义多圈计数器类 区别于SRML中的多圈计数
 */
class origin_MultiTurnRecorderClassdef
{
public:
    inline void ClearRoundCnt() { round_cnt = 0; }
    bool angle_is_init = 0; // 由私有变为公共

    void setAngleOffset(float _angle_offset)
    {
        angle_is_init = true;
        last_singleTurnAngle = angle_offset = _angle_offset;
        ClearRoundCnt(); // 清除圈数
    }

    float MultiTurnRecord(float SingleTurnAngle)
    {
        if (angle_is_init)
        {
            if (SingleTurnAngle - last_singleTurnAngle > 180)
                this->round_cnt--;
            else if (SingleTurnAngle - last_singleTurnAngle < -180)
                this->round_cnt++;
        }
        else
        {
            angle_offset = SingleTurnAngle;
            angle_is_init = true;
        }
        this->last_singleTurnAngle = SingleTurnAngle;
        return round_cnt * 360 + SingleTurnAngle - angle_offset;
    }

private:
    int32_t round_cnt = 0;
    float last_singleTurnAngle = 0, angle_offset = 0;
};

/**
 * @brief 自定义多圈计数器类
 */
class My_MultiTurnRecorderClassdef
{
public:
    inline void ClearRoundCnt() { round_cnt = 0; }
    bool angle_is_init = 0; // 由私有变为公共

protected:
    void setAngleOffset(float _angle_offset)
    {
        angle_is_init = true;
        last_singleTurnAngle = angle_offset = _angle_offset;
    }

    float MultiTurnRecord(float SingleTurnAngle)
    {
        if (angle_is_init)
        {
            if (SingleTurnAngle - last_singleTurnAngle > 180)
                this->round_cnt--;
            else if (SingleTurnAngle - last_singleTurnAngle < -180)
                this->round_cnt++;
        }
        else
        {
            for (;;)
                ;
        }
        this->last_singleTurnAngle = SingleTurnAngle;
        return round_cnt * 360 + SingleTurnAngle - angle_offset;
    }

private:
    int32_t round_cnt = 0;
    float last_singleTurnAngle = 0, angle_offset = 0;
};

/**
 * @brief LK电机类
 *
 */
class LK_Motor : public Motor_, My_MultiTurnRecorderClassdef
{
private:
    MotorMF9025v2Classdef *lk_motor = nullptr; // 或许使用MF9025_v2的电机类并不好，因为有减速比的错误
    float Reduction_ratio = 0.33;              // LK 从9025到当前所用型号电机的线性变换专用减速比
    uint8_t motor_type = 0;                    // 瓴控的电机类型，0为MG，1为MS
    float RATIO_ENCODER_TO_ANGLE = 360 / 65536.f;
    uint16_t encoder_offset = 0;

public:
    explicit LK_Motor(MotorMF9025v2Classdef *_lk_motor, float _reduction_ratio, uint8_t _motor_type = 0)
    {
        if (_lk_motor)
        {
            this->lk_motor = _lk_motor;
        }
        status = MotorStatus::error; // 使用前需先使能电机
        if (_reduction_ratio >= 0)
        {
            Reduction_ratio = _reduction_ratio;
        }
        motor_type = _motor_type; // 0为MG，1为MS
    }

    void bindRxTxQueueHandle(QueueHandle_t &_CAN_TxPort);
    void setPolar(int polar);
    void setOffset(float _offset); // 为电机设置零点
    void update(Motor &motor) override;
    void setTorque(float target_torque) override;
    void enableMotor();
};

#endif