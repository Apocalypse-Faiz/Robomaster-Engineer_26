#ifndef _WS2812_H_
#define _WS2812_H_

#ifdef __cplusplus

#include "SRML.h"
#include "tim.h"

#define PIXEL_NUM 6

#define WS_NUM 24 * (PIXEL_NUM + 3) // Reset 80us / 1.25us = 64<100

#define WS1 52

#define WS0 26

/// @brief 灯珠颜色种类
enum WS_colorType
{
    WS_Red = 0,
    WS_Green = 1,
    WS_Blue = 2,
    WS_White = 3,
    WS_Purple = 4,
    WS_Black = 5,
    WS_Yellow = 6,
    WS_Cyan = 7,
};

extern WS_colorType WS_indicator[6];

class WS2812Classdef
{
public:
    WS2812Classdef() {};
    void WS_Load();                                                               // 加载数据到灯珠，更改PWM波形
    void WS_Write_Red(uint8_t LedId, uint8_t strength);                           // 转化成二级制后6位，对应到灯珠数量，1为红色，0为无色
    void WS_Write_Color(uint8_t LedId, WS_colorType colortype, uint8_t strength); // 设置对应LED灯珠颜色
    void WS_Write_AllColor(uint8_t strength);                                     // 显示所有灯珠颜色

private:
    uint32_t send_buff[PIXEL_NUM + 3][24];

    void WS_Write_SingleRed(uint8_t LedId, uint8_t strength);    // 第id+1个是红色, strength为亮度（1-8）
    void WS_Write_SingleBlack(uint8_t LedId, uint8_t strength);  // 第id+1个是无色, strength为亮度（1-8）
    void WS_Write_SingleBlue(uint8_t LedId, uint8_t strength);   // 第id+1个是蓝色, strength为亮度（1-8）
    void WS_Write_SingleYellow(uint8_t LedId, uint8_t strength); // 第id+1个是黄色, strength为亮度（1-8）
    void WS_Write_SingleWhite(uint8_t LedId, uint8_t strength);  // 第id+1个是白色 , strength为亮度（1-8）
    void WS_Write_SinglePurple(uint8_t LedId, uint8_t strength); // 第id+1个是紫色, strength为亮度（1-8）
    void WS_Write_SingleGreen(uint8_t LedId, uint8_t strength);  // 第id+1个是绿色, strength为亮度（1-8）
    void WS_Write_SingleCyan(uint8_t LedId, uint8_t strength);   // 第id+1个是青色, strength为亮度（1-8）
};

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

#endif

#endif