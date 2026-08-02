#include "internal.h"
#include "ActionGroupController.h"
#include "TopState.h"
#include "GetEnergyUnit.h"

volatile BottomComDiagStats g_bottom_com_diag_stats = {0U, 0U, 0U, 0U};

/**
 * @brief 底盘信息发送执行
 */
uint8_t Bottomdata_Transmit(float _x, float _y, float _z)
{

    static ToBottomDataStructdef ToBottomData_temp; // 缓冲
    static USART_COB BottomBoard_COB;

    memcpy(&ToBottomData_temp, &toBottomdata, sizeof(toBottomdata));

    ToBottomData_temp.chassis_control_struct.x = _x;
    ToBottomData_temp.chassis_control_struct.y = _y;
    ToBottomData_temp.chassis_control_struct.z = _z;
    ToBottomData_temp.chassis_control_struct.rfid_height = toBottomdata.chassis_control_struct.rfid_height;
    ToBottomData_temp.chassis_control_struct.chassis_height = toBottomdata.chassis_control_struct.chassis_height;

    ToBottomData_temp.ChassisState = toBottomdata.ChassisState; // 控制状态

    // Final_toBottomdata.chassis_control_struct.x = _x; // 用于打印信息
    // Final_toBottomdata.chassis_control_struct.y = _y;
    // Final_toBottomdata.chassis_control_struct.z = _z;

    // ToBottomData_temp.L_Store = toBottomdata.L_Store;
    // ToBottomData_temp.R_Store = toBottomdata.R_Store;

    if ((g_top_state_id == TopStateId::GetEnergyUnit) && GetEnergyUnit_isActionGroupMode())
    {
        ToBottomData_temp.GetOreIndex =
            action_group_controller.isAutoFourOreRunning()
                ? 7U
                : (uint8_t)(action_group_controller.getSelectedOre() + 1U);
    }
    else
    {
        ToBottomData_temp.GetOreIndex = 0U;
    }

    ToBottomData_temp.OreBit[0] = OreBit[0];
    ToBottomData_temp.OreBit[1] = OreBit[1];
    ToBottomData_temp.OreBit[2] = OreBit[2];
    ToBottomData_temp.OreBit[3] = OreBit[3];

    ToBottomData_temp.UI_mode = (uint8_t)UI_Mode; // UI界面赋值

    ToBottomData_temp.crc_code = std_lib::CRC16(((uint8_t *)&ToBottomData_temp) + 2, sizeof(ToBottomData_temp) - 4, 0x8005); // 跨过两个头开始取，然后去掉头和尾的长度

    BottomBoard_COB.port_num = 4;
    BottomBoard_COB.len = sizeof(ToBottomData_temp);
    BottomBoard_COB.address = (uint8_t *)&ToBottomData_temp;

    return (xQueueSend(USART_TxPort, &BottomBoard_COB, 0) == pdPASS) ? 1U : 0U;
}

/**
 * * @brief 接受底盘信息
 */
uint8_t Bottomdata_ReceiveBytes(const uint8_t *Recv_Data, uint16_t ReceiveLen)
{
    static ReBottomDataStructdef _tempReBottomData;

    g_bottom_com_diag_stats.rx_total_packets++;

    if (Recv_Data == NULL || ReceiveLen != sizeof(_tempReBottomData))
    {
        g_bottom_com_diag_stats.rx_len_fail_packets++;
        return 0U;
    }

    memcpy(&_tempReBottomData, Recv_Data, sizeof(_tempReBottomData));
    if (std_lib::CRC16(&_tempReBottomData, sizeof(_tempReBottomData), 0x8005) == 0)
    {
        memcpy(&reBottomdata, &_tempReBottomData, sizeof(_tempReBottomData));
        g_bottom_com_diag_stats.rx_ok_packets++;
        return 1U;
    }
    else
    {
        g_bottom_com_diag_stats.rx_crc_fail_packets++;
        return 0U;
    }
}

void Bottomdata_Receive(USART_COB _Usart_RxCOB)
{
    (void)Bottomdata_ReceiveBytes((const uint8_t *)_Usart_RxCOB.address, _Usart_RxCOB.len);
}
