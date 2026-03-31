#include "internal.h"

/**
 * @brief 底盘信息发送执行
 */
void Bottomdata_Transmit(float _x, float _y, float _z)
{

    static ToBottomDataStructdef ToBottomData_temp; // 缓冲
    static USART_COB BottomBoard_COB;

    memcpy(&ToBottomData_temp, &toBottomdata, sizeof(toBottomdata));

    ToBottomData_temp.chassis_control_struct.x = _x;
    ToBottomData_temp.chassis_control_struct.y = _y;
    ToBottomData_temp.chassis_control_struct.z = _z;

    // Final_toBottomdata.chassis_control_struct.x = _x; // 用于打印信息
    // Final_toBottomdata.chassis_control_struct.y = _y;
    // Final_toBottomdata.chassis_control_struct.z = _z;

    ToBottomData_temp.L_gripper = toBottomdata.L_gripper;
    ToBottomData_temp.R_gripper = toBottomdata.R_gripper;

    ToBottomData_temp.UI_mode = (uint8_t)UI_Mode; // UI界面赋值

    ToBottomData_temp.crc_code = std_lib::CRC16(((uint8_t *)&ToBottomData_temp) + 2, sizeof(ToBottomData_temp) - 4, 0x8005); // 跨过两个头开始取，然后去掉头和尾的长度

    BottomBoard_COB.port_num = 4;
    BottomBoard_COB.len = sizeof(ToBottomData_temp);
    BottomBoard_COB.address = (uint8_t *)&ToBottomData_temp;

    xQueueSend(USART_TxPort, &BottomBoard_COB, 0);
}

/**
 * * @brief 接受底盘信息
 */
void Bottomdata_Receive(USART_COB _Usart_RxCOB)
{
    static ReBottomDataStructdef _tempReBottomData;
    memcpy(&_tempReBottomData, _Usart_RxCOB.address, sizeof(_tempReBottomData));
    if (std_lib::CRC16(&_tempReBottomData, sizeof(_tempReBottomData), 0x8005) == 0 && _Usart_RxCOB.len == sizeof(_tempReBottomData))
    {
        memcpy(&reBottomdata, &_tempReBottomData, sizeof(_tempReBottomData));
    }
}