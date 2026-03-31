#ifndef __LOSTSTATE_H__
#define __LOSTSTATE_H__

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "TopState.h"
/* Private macros ------------------------------------------------------------*/
/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

#endif

extern "C"
{
    /* Exported macros -----------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
    /* Exported function declarations --------------------------------------------*/
    void Lost_enterAction();
    void Lost_action(Event_t **pEvent);
    void Lost_exitAction();
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/

#endif