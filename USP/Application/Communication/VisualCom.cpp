#include "internal.h"

CRCErrorStats vision_crc_stats = {0};

void Visiondata_Transmist()
{
    static ToVisionDataStruct ToVisionData_temp;
    static USART_COB Vision_COB;

    memcpy(&ToVisionData_temp, &toVisiondata, sizeof(ToVisionData_temp));

    ToVisionData_temp.head1 = 0xFE;
    ToVisionData_temp.head2 = 0xED;

    // for (int i = 0; i < 6; i++)
    // {
    //     ToVisionData_temp.joint_current[i] = toVisiondata.joint_current[i];
    // }

    // ToVisionData_temp.seq_num = toVisiondata.seq_num;
    // ToVisionData_temp.timestamp = toVisiondata.timestamp;
   
    // ToComData_temp.CRC_code = std_lib::CRC16(((uint8_t *)&ToComData_temp) + 1, sizeof(ToComData_temp) - 4, 0x8005);
    // ToVisionData_temp.crc_code = 0;
    ToVisionData_temp.crc_code = calculate_crc16((uint8_t *)&ToVisionData_temp, sizeof(ToVisionData_temp) - 2);
    // ToVisionData_temp.crc_code = std_lib::CRC16((uint8_t *)&ToVisionData_temp, sizeof(ToVisionData_temp) - 2, 0XFFFF);

    Vision_COB.port_num = 1;
    Vision_COB.len = sizeof(ToVisionData_temp);
    Vision_COB.address = (uint8_t *)&ToVisionData_temp;

    xQueueSend(USART_TxPort, &Vision_COB, 0);
}

void Visiondata_Receive(USART_COB _Usart_RxCOB)
{
    static ReVisionDataStruct ReVisionData_temp;
    memcpy(&ReVisionData_temp, _Usart_RxCOB.address, sizeof(ReVisionData_temp));
    vision_crc_stats.total_received++;
    if (ReVisionData_temp.crc_code == calculate_crc16((uint8_t *)&ReVisionData_temp, sizeof(ReVisionData_temp) - 2))
    {
        if (ReVisionData_temp.head1 == 0xFE && ReVisionData_temp.head2 == 0xED)
        {
            memcpy(&reVisiondata, &ReVisionData_temp, sizeof(ReVisionData_temp));
            // toVisiondata.timestamp = HAL_GetTick();
            // toVisiondata.seq_num = reVisiondata.seq_num;
            xTaskNotify(StateMachine_Handle, 55, eSetValueWithOverwrite);
        }
        else
        {
            vision_crc_stats.crc_errors++;
        }
    }
    else
    {
        vision_crc_stats.crc_errors++;
    }

    // 计算当前错误率
    if (vision_crc_stats.total_received > 0)
    {
        vision_crc_stats.error_rate = (float)vision_crc_stats.crc_errors / (float)vision_crc_stats.total_received;
    }
}
