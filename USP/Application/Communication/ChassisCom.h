#pragma once
#include "SRML.h"

/*
 *@brief 接受底盘信息
 */
struct ReBottomDataStructdef
{
    uint8_t ChassisState : 4; // 失能0 正常1 兑矿2 前腿3 后腿4
    float rfid_height;   // 单位: m
    float chassis_height; // 单位: m
    float chassis_angle[2];    // 真实反馈 —— 底盘倾角，单位: rad（0：Pitch，1：Roll）
    float EndPosition[8];      // 真实反馈 —— 足末端位置（0-3：X坐标，4-7：Z坐标）（左前、右前、左后、右后）   
    uint8_t HighTempMotorNum : 4;   // 真实反馈 —— 高温电机数量（大于100度）
    uint8_t OverTempFlag : 1;      // 真实反馈 —— 过温标志位
    // uint8_t L_Store : 1; // 左储矿 0：电磁铁关闭；1：电磁铁开启
    // uint8_t R_Store : 1; // 右储矿 0：电磁铁关闭；1：电磁铁开启
    // uint8_t HP;          // 机器人血量
    uint16_t CRC_code;   // 16位CRC校验码
} __attribute__((packed));

struct ChassisControlStruct // struct of targets and flags for controlling chassis
{
    float x;
    float y;
    float z;
    float rfid_height;   // 单位: m
    float chassis_height; // 单位: m
} __attribute__((packed));

/**
 * @brief 发送给底盘信息
 */
struct ToBottomDataStructdef
{
    uint8_t frame_head[2] = {0x12, 0x23};
    ChassisControlStruct chassis_control_struct;
    uint8_t ChassisState : 4; // 失能0 正常1 兑矿2 前腿3 后腿4
    uint8_t UI_mode : 4; // 0：默认状态  1：刷新UI —— 因为动态UI需要UI绘制到第一次才可以刷新，该位置以供刷新使用
    // uint8_t L_Store : 1;        // 左储矿 0：关闭电磁铁，1：开启
    // uint8_t R_Store : 1;        // 右储矿 0：关闭电磁铁，1：开启
    uint16_t crc_code;
} __attribute__((packed));

struct Vec3
{
    float x, y, z;
};

struct BottomComDiagStats
{
    uint32_t rx_total_packets;
    uint32_t rx_ok_packets;
    uint32_t rx_crc_fail_packets;
    uint32_t rx_len_fail_packets;
};

extern volatile BottomComDiagStats g_bottom_com_diag_stats;

uint8_t Bottomdata_Transmit(float _x, float _y, float _z);
uint8_t Bottomdata_ReceiveBytes(const uint8_t *Recv_Data, uint16_t ReceiveLen);
void Bottomdata_Receive(USART_COB _Usart_RxCOB);
