#ifndef _VISUALCOM_H_
#define _VISUALCOM_H_

#include "SRML.h"

#ifdef __cplusplus
extern "C"
{
#endif

#pragma pack(push, 1)

struct ToVisionDataStruct
{
    uint8_t head1;
    uint8_t head2;
    uint16_t seq_num;
    uint8_t comm_flag;
    float joint_current[6];
    float torque_current[6];
    float chassis_height_current;
    uint16_t crc_code;
};

struct ReVisionDataStruct
{
    uint8_t head1;
    uint8_t head2;
    uint16_t seq_num;
    uint8_t comm_flag;
    float joint_target[6];
    float torque_target[6];
    float chassis_height_target;
    uint16_t crc_code;
};

// CRC错误统计结构体
struct CRCErrorStats {
    uint32_t total_received;      // 总共接收到的包数
    uint32_t crc_errors;          // CRC校验失败的包数
    float error_rate;             // 错误率
};

#pragma pack(pop)

// CRC16 校验算法 
// ===========================
inline uint16_t calculate_crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

extern CRCErrorStats vision_crc_stats;
uint8_t Visiondata_Transmist();
uint8_t Visiondata_Receive(const uint8_t *Recv_Data, uint16_t ReceiveLen);

#ifdef __cplusplus
}
#endif

#endif