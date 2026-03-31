#pragma once

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
    void Mobile_enterAction();
    void Mobile_action(Event_t **pEvent);
    void Mobile_exitAction();
}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/