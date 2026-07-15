#include "internal.h"
#include "stream_buffer.h"

CRCErrorStats vision_crc_stats = {0};

static_assert(sizeof(ToVisionDataStruct) <= VISION_FRAME_MAX_SIZE, "Vision TX frame is too large");
static_assert(sizeof(ReVisionDataStruct) <= VISION_FRAME_MAX_SIZE, "Vision RX frame is too large");
static_assert(sizeof(ToVisionDataStruct) >= (VISION_FRAME_HEADER_SIZE + VISION_FRAME_CRC_SIZE),
              "Vision TX frame is too small");
static_assert(sizeof(ReVisionDataStruct) >= (VISION_FRAME_HEADER_SIZE + VISION_FRAME_CRC_SIZE),
              "Vision RX frame is too small");

static const size_t VISION_RX_STREAM_CAPACITY = VISION_FRAME_MAX_SIZE * 4U;
static const uint16_t VISION_RX_FRAME_SIZE = (uint16_t)sizeof(ReVisionDataStruct);

static StaticStreamBuffer_t vision_rx_stream_control;
static uint8_t vision_rx_stream_storage[VISION_RX_STREAM_CAPACITY + 1U];
static StreamBufferHandle_t vision_rx_stream = NULL;

static StaticQueue_t vision_frame_queue_control;
static uint8_t vision_frame_queue_storage[sizeof(VisionFrameBuffer)];
static QueueHandle_t vision_frame_queue = NULL;

static uint8_t vision_rx_frame_buffer[VISION_FRAME_MAX_SIZE];
static uint16_t vision_rx_frame_length = 0U;
static VisionFrameBuffer vision_publish_frame;
static uint8_t vision_task_receive_buffer[VIRTUALCOM_RX_BUFFER_SIZE];

static void Visiondata_UpdateErrorRate()
{
    if (vision_crc_stats.total_received > 0)
    {
        vision_crc_stats.error_rate = (float)vision_crc_stats.crc_errors / (float)vision_crc_stats.total_received;
    }
}

static void Visiondata_ResyncReceiveBuffer()
{
    const uint16_t buffered_length = vision_rx_frame_length;

    /* A damaged candidate may already contain the beginning of the next frame. */
    for (uint16_t i = 1U; (i + 1U) < buffered_length; i++)
    {
        if ((vision_rx_frame_buffer[i] == VISION_FRAME_HEAD1) &&
            (vision_rx_frame_buffer[i + 1U] == VISION_FRAME_HEAD2))
        {
            vision_rx_frame_length = buffered_length - i;
            memmove(vision_rx_frame_buffer,
                    &vision_rx_frame_buffer[i],
                    vision_rx_frame_length);
            return;
        }
    }

    if ((buffered_length > 0U) &&
        (vision_rx_frame_buffer[buffered_length - 1U] == VISION_FRAME_HEAD1))
    {
        vision_rx_frame_buffer[0] = VISION_FRAME_HEAD1;
        vision_rx_frame_length = 1U;
    }
    else
    {
        vision_rx_frame_length = 0U;
    }
}

static uint8_t Visiondata_PublishFrame()
{
    vision_publish_frame.length = VISION_RX_FRAME_SIZE;
    memcpy(vision_publish_frame.data,
           vision_rx_frame_buffer,
           vision_publish_frame.length);

    if (vision_frame_queue != NULL)
    {
        return (xQueueOverwrite(vision_frame_queue, &vision_publish_frame) == pdPASS) ? 1U : 0U;
    }

    return 0U;
}

static uint8_t Visiondata_ParseByte(uint8_t byte)
{
    if (vision_rx_frame_length == 0U)
    {
        if (byte == VISION_FRAME_HEAD1)
        {
            vision_rx_frame_buffer[0] = byte;
            vision_rx_frame_length = 1U;
        }
        return 0U;
    }

    if (vision_rx_frame_length == 1U)
    {
        if (byte == VISION_FRAME_HEAD2)
        {
            vision_rx_frame_buffer[1] = byte;
            vision_rx_frame_length = 2U;
        }
        else if (byte != VISION_FRAME_HEAD1)
        {
            vision_rx_frame_length = 0U;
        }
        return 0U;
    }

    vision_rx_frame_buffer[vision_rx_frame_length++] = byte;

    if (vision_rx_frame_length < VISION_RX_FRAME_SIZE)
    {
        return 0U;
    }

    vision_crc_stats.total_received++;

    const uint16_t crc_offset = VISION_RX_FRAME_SIZE - VISION_FRAME_CRC_SIZE;
    const uint16_t received_crc =
        (uint16_t)vision_rx_frame_buffer[crc_offset] |
        ((uint16_t)vision_rx_frame_buffer[crc_offset + 1U] << 8U);

    if (received_crc == calculate_crc16(vision_rx_frame_buffer, crc_offset))
    {
        const uint8_t published = Visiondata_PublishFrame();
        vision_rx_frame_length = 0U;
        Visiondata_UpdateErrorRate();
        return published;
    }

    vision_crc_stats.crc_errors++;
    Visiondata_ResyncReceiveBuffer();
    Visiondata_UpdateErrorRate();
    return 0U;
}

uint8_t Visiondata_Transmist()
{
    static ToVisionDataStruct tx_frame;

#if USE_SRML_VIRTUAL_COM
    if (VirtualComIsTxBusy() != 0U)
    {
        return USBD_BUSY;
    }

    memcpy(&tx_frame, &toVisiondata, sizeof(tx_frame));
    tx_frame.head1 = VISION_FRAME_HEAD1;
    tx_frame.head2 = VISION_FRAME_HEAD2;

    uint8_t *frame_bytes = (uint8_t *)&tx_frame;
    const uint16_t crc_offset = (uint16_t)sizeof(tx_frame) - VISION_FRAME_CRC_SIZE;
    const uint16_t crc = calculate_crc16(frame_bytes, crc_offset);
    frame_bytes[crc_offset] = (uint8_t)(crc & 0xFFU);
    frame_bytes[crc_offset + 1U] = (uint8_t)(crc >> 8U);

    return VirtualComTransmitData(frame_bytes, sizeof(tx_frame));
#else
    return USBD_FAIL;
#endif
}

uint8_t Visiondata_IsTxBusy()
{
#if USE_SRML_VIRTUAL_COM
    return VirtualComIsTxBusy();
#else
    return 0U;
#endif
}

void Visiondata_Init(void)
{
    if (vision_rx_stream == NULL)
    {
        vision_rx_stream = xStreamBufferCreateStatic(
            VISION_RX_STREAM_CAPACITY,
            1U,
            vision_rx_stream_storage,
            &vision_rx_stream_control);
    }

    if (vision_frame_queue == NULL)
    {
        vision_frame_queue = xQueueCreateStatic(
            1U,
            sizeof(VisionFrameBuffer),
            vision_frame_queue_storage,
            &vision_frame_queue_control);
    }
}

uint16_t Visiondata_ReceiveFromISR(const uint8_t *Recv_Data, uint16_t ReceiveLen)
{
    if ((Recv_Data == NULL) || (ReceiveLen == 0U) || (vision_rx_stream == NULL))
    {
        vision_crc_stats.dropped_bytes += ReceiveLen;
        return 0U;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    const size_t stored = xStreamBufferSendFromISR(
        vision_rx_stream,
        Recv_Data,
        ReceiveLen,
        &xHigherPriorityTaskWoken);

    vision_crc_stats.dropped_bytes += (uint32_t)ReceiveLen - (uint32_t)stored;
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    return (uint16_t)stored;
}

uint8_t Visiondata_ProcessRx(void)
{
    uint8_t valid_frame_count = 0U;

    if (vision_rx_stream == NULL)
    {
        return 0U;
    }

    for (;;)
    {
        const size_t received = xStreamBufferReceive(
            vision_rx_stream,
            vision_task_receive_buffer,
            sizeof(vision_task_receive_buffer),
            0U);

        if (received == 0U)
        {
            break;
        }

        for (size_t i = 0U; i < received; i++)
        {
            if (Visiondata_ParseByte(vision_task_receive_buffer[i]) != 0U)
            {
                if (valid_frame_count < UINT8_MAX)
                {
                    valid_frame_count++;
                }
            }
        }
    }

    return valid_frame_count;
}

uint8_t Visiondata_TakeLatestFrame(VisionFrameBuffer *frame)
{
    if ((frame == NULL) || (vision_frame_queue == NULL))
    {
        return 0U;
    }

    return (xQueueReceive(vision_frame_queue, frame, 0U) == pdPASS) ? 1U : 0U;
}

uint8_t Visiondata_DecodeControlFrame(const VisionFrameBuffer *frame,
                                      ReVisionDataStruct *control_data)
{
    if ((frame == NULL) || (control_data == NULL) ||
        (frame->length != sizeof(ReVisionDataStruct)))
    {
        return 0U;
    }

    ReVisionDataStruct temp;
    memcpy(&temp, frame->data, sizeof(temp));

    if ((temp.head1 != VISION_FRAME_HEAD1) ||
        (temp.head2 != VISION_FRAME_HEAD2))
    {
        return 0U;
    }

    memcpy(control_data, &temp, sizeof(temp));
    return 1U;
}
