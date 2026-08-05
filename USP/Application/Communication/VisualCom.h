#ifndef _VISUALCOM_H_
#define _VISUALCOM_H_

#include "SRML.h"

#define VISION_FRAME_HEAD1             0xFEU
#define VISION_FRAME_HEAD2             0xEEU
#define VISION_FRAME_MAX_SIZE          128U
#define VISION_FRAME_HEADER_SIZE       2U
#define VISION_FRAME_CRC_SIZE          2U

/* comm_flag values shared with the vision computer. */
enum VisionCommFlag
{
    VISION_COMM_CONTROL_ACTIVE = 0U,
    VISION_COMM_CONTROL_INACTIVE = 1U
};

enum VisionTxCommand
{
    VISION_TX_RECOGNIZE_AND_PREINSERT = 2U,
    VISION_TX_START_INSERTION = 3U,
    VISION_TX_CONFIRM = 4U,
    VISION_TX_PUSH_UP = 5U,
    VISION_TX_FLIP = 6U,
    VISION_TX_ROTATE_LEFT_30 = 7U,
    VISION_TX_ROTATE_RIGHT_30 = 8U,
    VISION_TX_ROTATE_LEFT_5 = 9U,
    VISION_TX_ROTATE_RIGHT_5 = 10U
};

enum VisionRxStatus
{
    VISION_RX_RECOGNITION_FAILED = 5U,
    VISION_RX_SOLUTION_FAILED = 6U,
    VISION_RX_RECOGNITION_SUCCEEDED = 10U,
    VISION_RX_INSERTION_COMPLETED = 11U,
    VISION_RX_CONFIRM_COMPLETED = 12U,
    VISION_RX_PUSH_UP_COMPLETED = 13U,
    VISION_RX_FLIP_COMPLETED = 14U,
    VISION_RX_ROTATE_LEFT_30_COMPLETED = 15U,
    VISION_RX_ROTATE_RIGHT_30_COMPLETED = 16U,
    VISION_RX_ROTATE_LEFT_5_COMPLETED = 17U,
    VISION_RX_ROTATE_RIGHT_5_COMPLETED = 18U
};

/*
 * Wire format (little-endian):
 * FE EE | agreed fixed payload | CRC16:u16
 * Frame boundaries are determined by the packed structure size agreed with
 * the host, not by USB packet boundaries.
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
    uint16_t seq_num;
    uint8_t comm_flag;
    float joint_target[7];
    float torque_target[7];
    float chassis_height_target;
    uint16_t crc_code;
};

// CRC错误统计结构体
struct CRCErrorStats {
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
uint16_t Visiondata_ReceiveFromISR(const uint8_t *Recv_Data, uint16_t ReceiveLen);
uint8_t Visiondata_ProcessRx(void);
uint8_t Visiondata_TakeLatestFrame(VisionFrameBuffer *frame);
uint8_t Visiondata_DecodeControlFrame(const VisionFrameBuffer *frame, ReVisionDataStruct *control_data);
uint8_t Visiondata_IsTxBusy(void);

#ifdef __cplusplus
}
#endif

#endif
