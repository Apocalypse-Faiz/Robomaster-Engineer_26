#ifndef __EXCHANGEENERGYUNIT_H__
#define __EXCHANGEENERGYUNIT_H__

#ifdef __cplusplus
#include "TopState.h"
#include "Claw.h"
#include "ActionGroup.h"
#include "CustomControl.h"
#include "VisualControl.h"

/* 控制模式枚举 */
enum class ExchangeMode {
    ACTION_GROUP,       // 动作组模式
    CUSTOM_CONTROL,     // 自定义控制器模式
    VISION_CONTROL      // 视觉模式
};

/* 兑换状态枚举 */
enum class ExchangeState {
    IDLE,               // 空闲状态
    INSERTING,          // 插入能量单元
    TRANSLATING,        // 向右平移
    ROTATING,           // 旋转能量单元
    VERIFYING,          // 验证兑换
    // COMPLETED,          // 兑换完成
    // FAILED              // 兑换失败
};

/* 兑换参数（由外部配置） */
typedef struct {
    float insert_distance;      // 插入距离（mm）
    float translation_distance; // 平移距离（默认100mm）
    float rotation_angle;       // 旋转角度（度）
    bool translation_required;  // 是否需要平移
    bool rotation_required;     // 是否需要旋转
    uint32_t timeout_ms;        // 超时时间
    uint8_t retry_count;        // 重试次数
} ExchangeParams;

extern "C"
{
    void ExchangeEnergyUnit_enterAction();
    void ExchangeEnergyUnit_action(Event_t **pEvent);
    void ExchangeEnergyUnit_exitAction();
    
    /* 配置接口 */
    void ExchangeEnergyUnit_setParams(ExchangeParams& params);
    void ExchangeEnergyUnit_startExchange();
    void ExchangeEnergyUnit_stopExchange();
    bool ExchangeEnergyUnit_isExchanging();
    ExchangeState ExchangeEnergyUnit_getCurrentState();
}

#endif
#endif