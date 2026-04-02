#pragma once
#include "SRML.h"

/*
 *@brief 接受底盘信息
 */
struct ReBottomDataStructdef
{
    uint8_t is_ctrl : 1; // 底盘控制状态 0：无控，1：在控
    uint8_t is_rst : 1;  // 底盘复位状态 0：非复位，1：复位
    uint8_t L_Store : 4; // 左储矿 0：未知；1：闭合；2：打开；3：正在关闭；4：正在打开
    uint8_t R_Store : 4; // 右储矿 0：未知；1：闭合；2：打开；3：正在关闭；4：正在打开
    uint8_t L_Ore : 1;     // 左边是否有矿石 0：无，1：有
    uint8_t R_Ore : 1;     // 右边是否有矿石 0：无，1：有
    uint8_t HP;          // 机器人血量
    uint16_t CRC_code;   // 16位CRC校验码
} __attribute__((packed));

struct ChassisControlStruct // struct of targets and flags for controlling chassis
{
    float x;
    float y;
    float z;
} __attribute__((packed));

/**
 * @brief 发送给底盘信息
 */
struct ToBottomDataStructdef
{
    uint8_t frame_head[2] = {0x12, 0x23};
    uint8_t is_ctrl : 1;
    uint8_t is_rst : 1; // 底盘控制状态 0：无控，1：在控；复位状态 0：非复位，1：复位
    uint8_t UI_mode : 7; // 暂时用做UI_mode
    ChassisControlStruct chassis_control_struct;
    uint8_t L_Store : 2;        // 左储矿 0：待命，1：开，2：合
    uint8_t R_Store : 2;        // 右储矿 0：待命，1：开，2：合
    uint16_t crc_code;
} __attribute__((packed));

struct Vec3
{
    float x, y, z;
};

void Bottomdata_Transmit(float _x, float _y, float _z);
void Bottomdata_Receive(USART_COB _Usart_RxCOB);
