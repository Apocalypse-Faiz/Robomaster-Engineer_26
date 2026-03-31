#include "WS2812.h"

uint8_t DMATIM2Flag = 0;
WS_colorType WS_indicator[6] = {WS_Blue, WS_Blue, WS_Blue, WS_Blue, WS_Blue, WS_Blue};

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		// HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
		DMATIM2Flag = 1;
	}
}

void WS2812Classdef::WS_Load()
{
	if (DMATIM2Flag == 0)
	{
		// HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)send_buff, WS_NUM);
	}
	if (DMATIM2Flag == 1)
	{
		DMATIM2Flag = 0;
	}
}

void WS2812Classdef::WS_Write_SingleRed(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
	}
}

void WS2812Classdef::WS_Write_SingleBlack(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;

	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
	}
}
void WS2812Classdef::WS_Write_SingleGreen(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
	}
}

void WS2812Classdef::WS_Write_SingleYellow(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
	}
}

void WS2812Classdef::WS_Write_SingleWhite(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
	}
}

void WS2812Classdef::WS_Write_SinglePurple(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
	}
}

void WS2812Classdef::WS_Write_SingleCyan(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS1;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
	}
}

void WS2812Classdef::WS_Write_SingleBlue(uint8_t LedId, uint8_t strength)
{
	if (LedId > 5)
		return;
	for (uint8_t i = 0; i < 24; i++)
	{
		send_buff[LedId][i] = WS0;
		if (i < strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 8 && i < 8 + strength)
		{
			send_buff[LedId][i] = WS0;
		}
		if (i >= 16 && i < 16 + strength)
		{
			send_buff[LedId][i] = WS1;
		}
	}
}

void WS2812Classdef::WS_Write_Red(uint8_t LedId, uint8_t strength)
{

	if (LedId > 5)
		return;
	bool Red;
	if (LedId > 63)
		return;
	for (uint8_t i = 0; i <= 5; i++)
	{
		Red = LedId & (1 << i);
		if (Red)
		{
			WS_Write_SingleRed(i, strength);
		}
		else
		{
			WS_Write_SingleBlack(i, strength);
		}
	}
	for (uint8_t i = 6; i <= 8; i++)
	{
		for (uint8_t j = 0; j < 24; j++)
		{
			send_buff[i][j] = 0; // 复位帧
		}
	}

	WS_Load();
}

void WS2812Classdef::WS_Write_Color(uint8_t LedId, WS_colorType WS_color, uint8_t strength)
{
	if (LedId > 5)
		return;

	switch (WS_color)
	{
	case WS_Red:
		WS_Write_SingleRed(LedId, strength);
		break;
	case WS_Black:
		WS_Write_SingleBlack(LedId, strength);
		break;
	case WS_Blue:
		WS_Write_SingleBlue(LedId, strength);
		break;
	case WS_Yellow:
		WS_Write_SingleYellow(LedId, strength);
		break;
	case WS_White:
		WS_Write_SingleWhite(LedId, strength);
		break;
	case WS_Purple:
		WS_Write_SinglePurple(LedId, strength);
		break;
	case WS_Green:
		WS_Write_SingleGreen(LedId, strength);
		break;
	case WS_Cyan:
		WS_Write_SingleCyan(LedId, strength);
		break;
	default:
		// Handle invalid color
		break;
	}

	for (uint8_t i = 6; i <= 8; i++)
	{
		for (uint8_t j = 0; j < 24; j++)
		{
			send_buff[i][j] = 0; // Reset frame
		}
	}

	WS_Load();
}

void WS2812Classdef::WS_Write_AllColor(uint8_t strength)
{
	for (uint8_t i = 0; i < 6; i++)
	{
		WS_Write_Color(i, WS_indicator[i], strength);
	}
}