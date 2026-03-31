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
    void Reset_enterAction();
    void Reset_action(Event_t **pEvent);
    bool Manipulator_TorqueReset();
    void Manipulator_TorqueReset_Params_Reset();
    void Reset_exitAction();

}
/************************ COPYRIGHT(C) SCUT-ROBOTLAB **************************/
