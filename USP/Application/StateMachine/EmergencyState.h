#ifndef __EMERGENCYSTATE_H__
#define __EMERGENCYSTATE_H__

#ifdef __cplusplus
#include "TopState.h"

extern "C"
{
    void Emergency_enterAction();
    void Emergency_action(Event_t **pEvent);
    void Emergency_exitAction();
}

#endif
#endif