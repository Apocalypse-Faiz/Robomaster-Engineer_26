#pragma once
#include "SRML.h"

/*
 *@brief 接受底盘信息
 */
struct ReBottomDataStructdef
{
    uint8_t is_ctrl : 1; // 底盘控制状态 0：无控，1：在控
    uint8_t L_Store : 1; // 左储矿 0：电磁铁关闭；1：电磁铁开启
    uint8_t R_Store : 1; // 右储矿 0：电磁铁关闭；1：电磁铁开启
    float rfid_height;
    float chassis_height;
    uint8_t HP;          // 机器人血量
    uint16_t CRC_code;   // 16位CRC校验码
} __attribute__((packed));

struct ChassisControlStruct // struct of targets and flags for controlling chassis
{
    float x;
    float y;
    float z;
    float rfid_height;
    float chassis_height;
} __attribute__((packed));

/**
 * @brief 发送给底盘信息
 */
struct ToBottomDataStructdef
{
    uint8_t frame_head[2] = {0x12, 0x23};
    ChassisControlStruct chassis_control_struct;
    uint8_t is_ctrl : 1;
    uint8_t UI_mode : 7; // 暂时用做UI_mode
    uint8_t L_Store : 1;        // 左储矿 0：关闭电磁铁，1：开启
    uint8_t R_Store : 1;        // 右储矿 0：关闭电磁铁，1：开启
    uint16_t crc_code;
} __attribute__((packed));

struct Vec3
{
    float x, y, z;
};

uint8_t Bottomdata_Transmit(float _x, float _y, float _z);
void Bottomdata_Receive(USART_COB _Usart_RxCOB);
