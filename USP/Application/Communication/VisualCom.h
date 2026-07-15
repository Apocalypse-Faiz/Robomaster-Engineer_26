#ifndef _VISUALCOM_H_
#define _VISUALCOM_H_

#include "SRML.h"

#define VISION_FRAME_HEAD1             0xFEU
#define VISION_FRAME_HEAD2             0xEEU
#define VISION_FRAME_MAX_SIZE          128U
#define VISION_FRAME_HEADER_SIZE       5U
#define VISION_FRAME_CRC_SIZE          2U
#define VISION_FRAME_MIN_SIZE          (VISION_FRAME_HEADER_SIZE + VISION_FRAME_CRC_SIZE)
#define VISION_FRAME_MAX_PAYLOAD_SIZE  (VISION_FRAME_MAX_SIZE - VISION_FRAME_MIN_SIZE)
#define VISION_FRAME_TYPE_CONTROL      0x01U
#define VISION_FRAME_TYPE_FEEDBACK     0x02U

/*
 * Wire format (little-endian):
 * FE EE | total_length:u16 | frame_type:u8 | payload | CRC16:u16
 * total_length includes the header, payload and CRC16.
 */

#ifdef __cplusplus
extern "C"
{
#endif

#pragma pack(push, 1)

struct ToVisionDataStruct
{
    uint8_t head1;
    uint8_t head2;
    uint16_t frame_length;
    uint8_t frame_type;
    uint16_t seq_num;
    uint8_t comm_flag;
    float joint_current[7];
    float torque_current[7];
    float chassis_height_current;
    uint16_t crc_code;
};

struct ReVisionDataStruct
{
    uint8_t head1;
    uint8_t head2;
    uint16_t frame_length;
    uint8_t frame_type;
    uint16_t seq_num;
    uint8_t comm_flag;
    float joint_target[7];
    float torque_target[7];
    float chassis_height_target;
    uint16_t crc_code;
};

// CRC错误统计结构体
struct CRCErrorStats {
    uint32_t length_errors;
    uint32_t dropped_bytes;
    uint32_t total_received;      // 总共接收到的包数
    uint32_t crc_errors;          // CRC校验失败的包数
    float error_rate;             // 错误率
};

#pragma pack(pop)

struct VisionFrameBuffer
{
    uint16_t length;
    uint8_t data[VISION_FRAME_MAX_SIZE];
};

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
void Visiondata_Init(void);
uint8_t Visiondata_Transmist();
uint8_t Visiondata_TransmitPayload(uint8_t frame_type, const uint8_t *payload, uint16_t payload_length);
uint16_t Visiondata_ReceiveFromISR(const uint8_t *Recv_Data, uint16_t ReceiveLen);
uint8_t Visiondata_ProcessRx(void);
uint8_t Visiondata_TakeLatestFrame(VisionFrameBuffer *frame);
uint8_t Visiondata_DecodeControlFrame(const VisionFrameBuffer *frame, ReVisionDataStruct *control_data);
uint8_t Visiondata_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif
