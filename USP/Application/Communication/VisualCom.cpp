#include "internal.h"

CRCErrorStats vision_crc_stats = {0};

static void Visiondata_UpdateErrorRate()
{
    if (vision_crc_stats.total_received > 0)
    {
        vision_crc_stats.error_rate = (float)vision_crc_stats.crc_errors / (float)vision_crc_stats.total_received;
    }
}

uint8_t Visiondata_Transmist()
{
    static ToVisionDataStruct ToVisionData_temp;

    memcpy(&ToVisionData_temp, &toVisiondata, sizeof(ToVisionData_temp));

    ToVisionData_temp.head1 = 0xFE;
    ToVisionData_temp.head2 = 0xEE;

    ToVisionData_temp.crc_code = calculate_crc16((uint8_t *)&ToVisionData_temp, sizeof(ToVisionData_temp) - 2);

#if USE_SRML_VIRTUAL_COM
    return VirtualComTransmitData((uint8_t *)&ToVisionData_temp, sizeof(ToVisionData_temp));
#endif

    return USBD_FAIL;
}

uint8_t Visiondata_Receive(const uint8_t *Recv_Data, uint16_t ReceiveLen)
{
    vision_crc_stats.total_received++;

    if ((Recv_Data == NULL) || (ReceiveLen < sizeof(ReVisionDataStruct)))
    {
        vision_crc_stats.crc_errors++;
        Visiondata_UpdateErrorRate();
        return 0;
    }

    ReVisionDataStruct ReVisionData_temp;
    memcpy(&ReVisionData_temp, Recv_Data, sizeof(ReVisionData_temp));

    if (ReVisionData_temp.crc_code != calculate_crc16((uint8_t *)&ReVisionData_temp, sizeof(ReVisionData_temp) - 2))
    {
        vision_crc_stats.crc_errors++;
        Visiondata_UpdateErrorRate();
        return 0;
    }

    if (ReVisionData_temp.head1 != 0xFE || ReVisionData_temp.head2 != 0xEE)
    {
        vision_crc_stats.crc_errors++;
        Visiondata_UpdateErrorRate();
        return 0;
    }

    memcpy(&reVisiondata, &ReVisionData_temp, sizeof(ReVisionData_temp));
    Visiondata_UpdateErrorRate();
    return 1;
}
