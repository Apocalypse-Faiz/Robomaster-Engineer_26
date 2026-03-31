#ifndef _KEYBOARDCONTROL_H
#define _KEYBOARDCONTROL_H

#ifdef __cplusplus
/* Includes ------------------------------------------------------------------*/
#include "internal.h"

/* Private macros ------------------------------------------------------------*/
enum class ButtonType {
    FN1,    // 左上
    FN2,    // 右上
    TRIGGER // 扳机
};

/* Private type --------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/
extern bool VT13_ButtonStates[3];
/* Exported types ------------------------------------------------------------*/
extern "C"
{
    /* Exported macros -----------------------------------------------------------*/
    /* Exported types ------------------------------------------------------------*/
    /* Exported function declarations --------------------------------------------*/
}

float max(float a, float b);
float min(float a, float b);
float constrain(float val, float lb, float rb);
float map(float x, float in_lb, float in_rb, float out_lb, float out_rb);
float sgn(float val);
float deadband(float val, float center, float radius);

/**
 * @brief 判断是否只有传入的按键被按下
 *
 * @tparam KeyType
 * @param _keys
 * @return true
 * @return false
 */
template <class... KeyType>
bool IsKeyOnlyPress(KeyType... _keys)
{
    bool pressedKeys[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int array[] = {(pressedKeys[(int)_keys] = true, 0)...};
    for (uint8_t i = 0; i < 18; i++)
    {
        if (pressedKeys[i] == DR16.IsKeyPress(i))
            continue;
        else
            return false;
    }
    return true;
}

bool IsKeyPressOnce(int _key);
void KeyBoard_MoveCtrl(ToBottomDataStructdef &_ToBottomData);
void KeyBoard_MoveCtrl_Custom(ToBottomDataStructdef &_ToBottomData);
void KeyBoard_MoveCtrl_Ore(ToBottomDataStructdef &_ToBottomData);
void DR16_Manipulater_JointCtrl();
void DR16_Debug_Ctrl();
void DR16_MoveCtrl(ToBottomDataStructdef &_ToBottomData);
void DR16_MoveCtrl_Ore(ToBottomDataStructdef &_ToBottomData);
void DR16_Grip();
void Keyboard_Grippers();
void DR16_StoreGrippers(ToBottomDataStructdef &_ToBottomData);
void Keyboard_StoreGrippers(ToBottomDataStructdef &_ToBottomData);

void VT13_ProcessButtons();
bool VT13_IsButtonActive(ButtonType buttonType); 
bool handleButtonToggle(ButtonType buttonType, bool isPressed);
void VT13_ProcessButtons();
bool VT13_IsButtonActive(ButtonType buttonType);   
void VT13_MoveCtrl(ToBottomDataStructdef &_ToBottomData);
void VT13_KeyBoard_MoveCtrl(ToBottomDataStructdef &_ToBottomData);
void VT13_KeyBoard_MoveCtrl(ToBottomDataStructdef &_ToBottomData);
void VT13_KeyBoard_MoveCtrl_Custom(ToBottomDataStructdef &_ToBottomData);
void VT13_Grip();
void VT13_KeyBoard_Grip();
void VT13_StoreGrippers(ToBottomDataStructdef &_ToBottomData);
void VT13_KeyboardStoreGrippers(ToBottomDataStructdef &_ToBottomData);
void VT13_Manipulater_JointCtrl();

#endif
#endif