#include "RGB_led.h"
/**@brief 亮灯
*@param[in] data 颜色数组
*@return 0 success
         1 fail
*/

uint8_t LED::ON(uint32_t data[6][RGB_NUM])
{
    uint16_t i, j;
    uint8_t res;

    /* 由于定时器2的寄存器为32位,所以分开讨论 */
    if (ledPwmTimer->Instance == TIM2)
    {
        for (i = 0; i < 6; i++)
        {
            for (j = 0; j < RGB_NUM; j++)
                Temp_Buffer[i][j] = data[i][j];
        }
        res = Set((uint32_t *)Temp_Buffer, 6 * RGB_NUM);
    }
    else
    {
        res = Set((uint32_t *)data, 6 * RGB_NUM);
    }
    /* 按照灯带时序，每次写入数据后需发送复位信号 */
    Reset();
    return res;
}

/**@brief 发送颜色数据
    *@param void
    *@return 0 success
             1 fail
*/
uint8_t LED::Set(uint32_t *data, uint32_t num)
{
    if (!Data_Transmit(data, num))
        return 0;
    else
        return 1;
}

/**@brief 发送重置数据
    *@param void
    *@return 0 success
             1 fail
*/
uint8_t LED::Reset(void)
{
    if (!Data_Transmit(Reset_RGB, RESET_NUM))
        return 0;
    else
        return 1;
}

/**@brief 数据传输
    *@param[in]	buffer 	数组缓存
    *@param[in] num	 传输数目
    *@return 0 success
             1 fail
*/
uint8_t LED::Data_Transmit(uint32_t *buffer, uint32_t num)
{
    /* 开始传输数据 */
    while (HAL_TIM_PWM_Start_DMA(ledPwmTimer, Channel, buffer, num) != HAL_OK)
        ;

    /* 等待传输完成 */
    while (HAL_DMA_GetState(hdma) != HAL_DMA_STATE_READY)
        ;

    /* 多通道的DMA要关闭 */
    while (HAL_TIM_PWM_Stop_DMA(ledPwmTimer, Channel) != HAL_OK)
        ;

    return 0;
}
