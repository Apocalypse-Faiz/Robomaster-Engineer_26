/* Includes ------------------------------------------------------------------*/
#include "Keyboard.h"
#include "TopState.h"
#include "MobileState.h"
#include "LostState.h"
#include "GetEnergyUnit.h"
#include "ExchangeEnergyUnit.h"
#include "EmergencyState.h"

/* Private define ------------------------------------------------------------*/

/*Variables ---------------------------------------------------------*/
using namespace hfsm;

State_t TopStateMachine;
State_t MobileState;
State_t LostState;
State_t GetEnergyUintState;
State_t ExchangeEnergyUnitState;       
State_t EmergencyState;      

Event_t ToMobile;
Event_t ToLost;
Event_t ToGetEnergyUnit;
Event_t ToExchangeEnergyUnit;
Event_t ToEmergency;

TopStateId g_top_state_id = TopStateId::None;

/* Private type --------------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

void Top_Action(Event_t **pEvent)
{

    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
    {
        // DM电机无需力矩复位，Robot_Prepared默认为1，所有状态切换直接可用
        if ((DR16.IsKeyPress(DR16_KEY_B) && DR16.IsKeyPress(DR16_KEY_CTRL)) || (VT13.IsKeyPress(VT13_KEY_B) && VT13.IsKeyPress(VT13_KEY_CTRL)))
        {
            *pEvent = &ToMobile;
        }
        else if((DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL)) || (VT13.IsKeyPress(VT13_KEY_Z) && VT13.IsKeyPress(VT13_KEY_CTRL)))
        {
            *pEvent = &ToGetEnergyUnit;
        }
        else if((DR16.IsKeyPress(DR16_KEY_V) && DR16.IsKeyPress(DR16_KEY_CTRL)) || (VT13.IsKeyPress(VT13_KEY_V) && VT13.IsKeyPress(VT13_KEY_CTRL)))
        {
            *pEvent = &ToExchangeEnergyUnit;
        }
        else if((DR16.IsKeyPress(DR16_KEY_X) && DR16.IsKeyPress(DR16_KEY_CTRL)) || (VT13.IsKeyPress(VT13_KEY_X) && VT13.IsKeyPress(VT13_KEY_CTRL)))
        {
            *pEvent = &ToEmergency;
        }
    }
    else
    {
        *pEvent = &ToLost;
    }
}

void Total_init()
{
    TopStateMachine.regisAction(Top_Action);

    TopStateMachine.addSon(&MobileState);
    TopStateMachine.addSon(&LostState);
    TopStateMachine.addSon(&GetEnergyUintState);
    TopStateMachine.addSon(&ExchangeEnergyUnitState);
    TopStateMachine.addSon(&EmergencyState);

    TopStateMachine.addSonTrans(nullptr, &ToMobile, &MobileState);
    TopStateMachine.addSonTrans(nullptr, &ToLost, &LostState);
    TopStateMachine.addSonTrans(nullptr, &ToGetEnergyUnit, &GetEnergyUintState);
    TopStateMachine.addSonTrans(nullptr, &ToExchangeEnergyUnit, &ExchangeEnergyUnitState);
    TopStateMachine.addSonTrans(nullptr, &ToEmergency, &EmergencyState);

    TopStateMachine.setInitialState(&LostState);

    MobileState.regisEnterAction(Mobile_enterAction);
    MobileState.regisAction(Mobile_action);
    MobileState.regisExitAction(Mobile_exitAction);

    LostState.regisEnterAction(Lost_enterAction);
    LostState.regisAction(Lost_action);
    LostState.regisExitAction(Lost_exitAction);

    GetEnergyUintState.regisEnterAction(GetEnergyUnit_enterAction);
    GetEnergyUintState.regisAction(GetEnergyUnit_action);
    GetEnergyUintState.regisExitAction(GetEnergyUnit_exitAction);

    ExchangeEnergyUnitState.regisEnterAction(ExchangeEnergyUnit_enterAction);
    ExchangeEnergyUnitState.regisAction(ExchangeEnergyUnit_action);
    ExchangeEnergyUnitState.regisExitAction(ExchangeEnergyUnit_exitAction);

    EmergencyState.regisEnterAction(Emergency_enterAction);
    EmergencyState.regisAction(Emergency_action);
    EmergencyState.regisExitAction(Emergency_exitAction);
}
