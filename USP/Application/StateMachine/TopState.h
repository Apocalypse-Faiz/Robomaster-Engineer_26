#ifndef __TOPSTATE_H__
#define __TOPSTATE_H__

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "hfsm.h"
#include "main.h"
/* Private macros ------------------------------------------------------------*/
using namespace hfsm;
/* Private type --------------------------------------------------------------*/

enum class TopStateId : uint8_t
{
	None = 0,
	Mobile,
	Lost,
	Reset,
	GetEnergyUnit,
	ExchangeEnergyUnit,
	Emergency
};

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
extern State_t TopStateMachine;
extern State_t MobileState;
extern State_t LostState;
extern State_t ResetState;

extern TopStateId g_top_state_id;

extern Event_t ToMobile;
extern Event_t ToReset;
extern Event_t ToLost;

/* Exported function declarations --------------------------------------------*/
void Total_init();
void Top_Action(Event_t **pEvent);

#endif

/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

#endif