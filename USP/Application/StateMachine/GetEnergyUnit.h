#ifndef __GETENERGYUNIT_H__
#define __GETENERGYUNIT_H__

#ifdef __cplusplus
#include "TopState.h"

extern "C"
{
    void GetEnergyUnit_enterAction();
    void GetEnergyUnit_action(Event_t **pEvent);
    void GetEnergyUnit_exitAction();
}

#endif
#endif