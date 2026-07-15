#include "internal.h"
#include "stream_buffer.h"

CRCErrorStats vision_crc_stats = {0};

static_assert(sizeof(ToVisionDataStruct) <= VISION_FRAME_MAX_SIZE, "Vision TX frame is too large");
static_assert(sizeof(ReVisionDataStruct) <= VISION_FRAME_MAX_SIZE, "Vision RX frame is too large");

static const uint16_t VISION_FRAME_LENGTH_OFFSET = 2U;
static const size_t VISION_RX_STREAM_CAPACITY = VISION_FRAME_MAX_SIZE * 4U;

static StaticStreamBuffer_t vision_rx_stream_control;
static uint8_t vision_rx_stream_storage[VISION_RX_STREAM_CAPACITY + 1U];
static StreamBufferHandle_t vision_rx_stream = NULL;

static StaticQueue_t vision_frame_queue_control;
static uint8_t vision_frame_queue_storage[sizeof(VisionFrameBuffer)];
static QueueHandle_t vision_frame_queue = NULL;

static uint8_t vision_rx_frame_buffer[VISION_FRAME_MAX_SIZE];
static uint16_t vision_rx_frame_length = 0U;
static uint16_t vision_rx_expected_length = 0U;
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

    vision_rx_expected_length = 0U;

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

static uint16_t Visiondata_ReadFrameLength()
{
    return (uint16_t)vision_rx_frame_buffer[VISION_FRAME_LENGTH_OFFSET] |
           ((uint16_t)vision_rx_frame_buffer[VISION_FRAME_LENGTH_OFFSET + 1U] << 8U);
}

static uint8_t Visiondata_PublishFrame()
{
    vision_publish_frame.length = vision_rx_expected_length;
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

    for (;;)
    {
        if ((vision_rx_expected_length == 0U) &&
            (vision_rx_frame_length >= (VISION_FRAME_LENGTH_OFFSET + sizeof(uint16_t))))
        {
            vision_rx_expected_length = Visiondata_ReadFrameLength();

            if ((vision_rx_expected_length < VISION_FRAME_MIN_SIZE) ||
                (vision_rx_expected_length > VISION_FRAME_MAX_SIZE))
            {
                vision_crc_stats.length_errors++;
                Visiondata_ResyncReceiveBuffer();

                if (vision_rx_frame_length < (VISION_FRAME_LENGTH_OFFSET + sizeof(uint16_t)))
                {
                    return 0U;
                }
                continue;
            }
        }

        if ((vision_rx_expected_length == 0U) ||
            (vision_rx_frame_length < vision_rx_expected_length))
        {
            return 0U;
        }

        vision_crc_stats.total_received++;

        const uint16_t crc_offset = vision_rx_expected_length - VISION_FRAME_CRC_SIZE;
        const uint16_t received_crc =
            (uint16_t)vision_rx_frame_buffer[crc_offset] |
            ((uint16_t)vision_rx_frame_buffer[crc_offset + 1U] << 8U);

        if (received_crc == calculate_crc16(vision_rx_frame_buffer, crc_offset))
        {
            const uint8_t published = Visiondata_PublishFrame();
            vision_rx_frame_length = 0U;
            vision_rx_expected_length = 0U;
            Visiondata_UpdateErrorRate();
            return published;
        }

        vision_crc_stats.crc_errors++;
        Visiondata_ResyncReceiveBuffer();
        Visiondata_UpdateErrorRate();

        if (vision_rx_frame_length < (VISION_FRAME_LENGTH_OFFSET + sizeof(uint16_t)))
        {
            return 0U;
        }
    }
}

uint8_t Visiondata_TransmitPayload(uint8_t frame_type,
                                   const uint8_t *payload,
                                   uint16_t payload_length)
{
    static uint8_t tx_frame[VISION_FRAME_MAX_SIZE];

    if (((payload == NULL) && (payload_length != 0U)) ||
        (payload_length > VISION_FRAME_MAX_PAYLOAD_SIZE))
    {
        return USBD_FAIL;
    }

    const uint16_t frame_length =
        VISION_FRAME_HEADER_SIZE + payload_length + VISION_FRAME_CRC_SIZE;

#if USE_SRML_VIRTUAL_COM
    if (VirtualComIsTxBusy() != 0U)
    {
        return USBD_BUSY;
    }

    tx_frame[0] = VISION_FRAME_HEAD1;
    tx_frame[1] = VISION_FRAME_HEAD2;
    tx_frame[VISION_FRAME_LENGTH_OFFSET] = (uint8_t)(frame_length & 0xFFU);
    tx_frame[VISION_FRAME_LENGTH_OFFSET + 1U] = (uint8_t)(frame_length >> 8U);
    tx_frame[4] = frame_type;

    if (payload_length != 0U)
    {
        memcpy(&tx_frame[VISION_FRAME_HEADER_SIZE], payload, payload_length);
    }

    const uint16_t crc_offset = frame_length - VISION_FRAME_CRC_SIZE;
    const uint16_t crc = calculate_crc16(tx_frame, crc_offset);
    tx_frame[crc_offset] = (uint8_t)(crc & 0xFFU);
    tx_frame[crc_offset + 1U] = (uint8_t)(crc >> 8U);

    return VirtualComTransmitData(tx_frame, frame_length);
#else
    return USBD_FAIL;
#endif
}

uint8_t Visiondata_Transmist()
{
    static ToVisionDataStruct ToVisionData_temp;

    memcpy(&ToVisionData_temp, &toVisiondata, sizeof(ToVisionData_temp));

    const uint8_t *payload =
        ((const uint8_t *)&ToVisionData_temp) + VISION_FRAME_HEADER_SIZE;
    const uint16_t payload_length =
        (uint16_t)sizeof(ToVisionData_temp) - VISION_FRAME_HEADER_SIZE - VISION_FRAME_CRC_SIZE;

    return Visiondata_TransmitPayload(
        VISION_FRAME_TYPE_FEEDBACK,
        payload,
        payload_length);
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
        (temp.head2 != VISION_FRAME_HEAD2) ||
        (temp.frame_length != sizeof(temp)) ||
        (temp.frame_type != VISION_FRAME_TYPE_CONTROL))
    {
        return 0U;
    }

    memcpy(control_data, &temp, sizeof(temp));
    return 1U;
}
