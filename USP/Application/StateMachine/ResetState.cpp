#include "ResetState.h"
#include "internal.h"
#include "KeyBoard.h"
#include "ActionGroup.h"

 /*以下为力矩复位做准备*/
static float motor1_torque, motor2_torque, motor3_torque, motor4_torque;
static uint8_t J4_reset_prepared, J6_reset_prepared = 0;
static uint8_t reset_ok = 0; // 电机复位是否成功标志位
static uint8_t J2_reset, J3_reset, J5_reset, J6_reset = 0;

void Reset_enterAction()
{
    // 进入复位状态后先执行一次力矩复位参数重置，确保每次进入复位状态时相关参数都被重置
    Manipulator_TorqueReset_Params_Reset();
    j5_reset_target = joint[4].getCurrentAngle();
    j6_reset_target = joint[5].getCurrentAngle();
}

void Manipulator_TorqueReset_Params_Reset()
{
    motor1_torque = 0.0f;
    motor2_torque = 0.0f;
    motor3_torque = 0.0f;
    motor4_torque = 0.0f;

    J4_reset_prepared = 0;
    J6_reset_prepared = 0;

    J2_reset = 0;
    J3_reset = 0;
    J5_reset = 0;
    J6_reset = 0;
}

/**
 * @brief 机械臂利用力矩复位
 * @note 主要是给J2、J3、J5、J6电机复位，并执行复位动作
 */
bool Manipulator_TorqueReset()
{

    if (reset_ok == 1)
    {
        return true;
    }

    /*复位过程*/
    if (motor[0].getStatus() == MotorStatus::ready && motor[1].getStatus() == MotorStatus::ready && motor[2].getStatus() == MotorStatus::ready && motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready)
    {

        //完成力矩复位
        motor1_torque = mfsL1.f(abs(motor[4].getTorque()));
        motor2_torque = mfsL2.f(abs(motor[5].getTorque()));
        motor3_torque = mfsL3.f(abs(motor[1].getTorque()));
        motor4_torque = mfsL4.f(abs(motor[2].getTorque())); 

        // gripper.torqueReset(); // 夹爪内部相关标志位复位

        float Reduction_ratio1 = 0.0667 * 0.0526; // 1/15 DJI3508的减速比
	    float Reduction_ratio2 = 0.0278; // 1/36 DJI2006的减速比

        // 由于一代车uwb支架干涉的问题，暂未找到完全不会遇到干涉的自动复位方案，故需要一定程度上手动辅助

        if (reset_ok == 0)
        {
            // 复位方案二：
            if (J4_reset_prepared == 0)
            {
                float j4_current_angle = joint[3].getCurrentAngle();
                float angle_diff = j4_current_angle - 20.0f;    

                if (abs(angle_diff) > 1.0f)
                {
                    if (angle_diff > 0)
                    {
                        joint[3].ajustJointTarget(0.1f);
                    }
                    else
                    {
                        joint[3].ajustJointTarget(-0.1f);
                    }
                }
                else
                {
                    J4_reset_prepared = 1;
                }
            }

            if (J6_reset_prepared == 0 && J4_reset_prepared == 1)
            {
                if (motor1_torque < 5700 || motor2_torque < 5700)
                 {
                     joint[5].ajustJointTarget(-15.0f);
                    // joint[5].setTargetAngle(j6_reset_target += 0.1f);
                 }
                 else
                 {
                     joint[4].setMode(JointMode::disabled);
                     joint[5].setMode(JointMode::disabled);
 
                     joint[5].calibrate();
 
                     joint[5].setJointMapping(-Reduction_ratio2, 152.1f);
                     joint[5].setJointParam(-190, 148, -10, 10);
 
                     joint[4].setMode(JointMode::angle);
                     joint[5].setMode(JointMode::angle);
                     J6_reset_prepared = 1;
                 }
            }


            if (J6_reset == 0 && J6_reset_prepared == 1)
            {
                float j6_current_angle = joint[5].getCurrentAngle();
                float j6_angle_diff = j6_current_angle - 0.0f;    

                if (abs(j6_angle_diff) > 0.1f)
                {
                    if (j6_angle_diff > 0)
                    {
                        joint[5].ajustJointTarget(3.6f);
                    }
                    else
                    {
                        joint[5].ajustJointTarget(-3.6f);
                    }
                }
                else                
                {
                    J6_reset = 1;
                }
            }

            if (J5_reset == 0 && J4_reset_prepared == 1 && J6_reset == 1 && J6_reset_prepared == 1)
            {
                 if (motor1_torque < 5700 || motor2_torque < 5700)
                 {
                    //  joint[4].ajustJointTarget(-15.0f);
                    joint[4].setTargetAngle(j5_reset_target -= 0.1f);
                 }
                 else
                 {
                     joint[4].setMode(JointMode::disabled);
                     joint[5].setMode(JointMode::disabled);
                    
                     // J5与J6只要一个进行了标定，那么motor5与motor6都会进行标定，之后相关的映射与限位J5、J6都要重新设
                     joint[4].calibrate();
 
                     joint[4].setJointMapping(Reduction_ratio2, -86.46f);
                     joint[4].setJointParam(-84, 84, -10, 10);
                     joint[5].setJointMapping(-Reduction_ratio2, 6.25f);
                     joint[5].setJointParam(-190, 149, -10, 10);
 
                     joint[4].setMode(JointMode::angle);
                     joint[5].setMode(JointMode::angle);
                     J5_reset = 1;
                 }
            }

            if (J6_reset == 0 && J6_reset_prepared == 1 && J5_reset == 1)
            {
                float j6_current_angle = joint[5].getCurrentAngle();
                float j6_angle_diff = j6_current_angle - 0.0f;    

                if (abs(j6_angle_diff) > 0.5f)
                {
                    if (j6_angle_diff > 0)
                    {
                        joint[5].ajustJointTarget(5.0f);
                    }
                    else
                    {
                        joint[5].ajustJointTarget(-5.0f);
                    }
                }
                else                
                {
                    J6_reset = 1;
                }
            }


            if (J3_reset == 0 && J5_reset == 1 && J4_reset_prepared == 1 && J6_reset == 1 && J6_reset_prepared == 1)
            {
                if (motor4_torque < 10000)
                {
                    joint[2].ajustJointTarget(-10.0f);
                }
                else
                {
                    joint[2].setMode(JointMode::disabled);

                    joint[2].calibrate();

                    joint[2].setJointMapping(-Reduction_ratio1, 79.15f);
                    joint[2].setJointParam(-49.0f, 77.0f, -10, 10);

                    joint[2].setMode(JointMode::angle);
                    J3_reset = 1;
                }
            }

            if (J2_reset == 0 && J3_reset == 1 && J5_reset == 1 && J4_reset_prepared == 1 && J6_reset == 1 && J6_reset_prepared == 1)
            {
                if (motor3_torque < 10000)
                {
                    joint[1].ajustJointTarget(-5.0f);
                }
                else
                {
                    joint[1].setMode(JointMode::disabled);

                    joint[1].calibrate();

                    joint[1].setJointMapping(-Reduction_ratio1, 164.08f);
                    joint[1].setJointParam(24.0f, 160.0f, -10, 10);

                    joint[1].setMode(JointMode::angle);
                    J2_reset = 1;
                }
            }

            // if (J2_reset == 1 && J3_reset == 1 && J5_reset == 1 && J6_reset == 0  && J4_reset_prepared == 1)
            // {
            //     if (motor1_torque < 5000 || motor2_torque < 5000)
            //      {
            //          joint[5].ajustJointTarget(-12.5f);
            //      }
            //      else
            //      {
            //          joint[4].setMode(JointMode::disabled);
            //          joint[5].setMode(JointMode::disabled);
 
            //          joint[5].setJointMapping(Reduction_ratio2, 149.1f);
            //          joint[5].setJointParam(-190, 147, -10, 10);
 
            //          joint[4].setMode(JointMode::angle);
            //          joint[5].setMode(JointMode::angle);
            //          J6_reset = 1;
            //      }   
            // }

            // 复位方案一：
        //    if (J5_J6_reset == 0)
        //    {
        //         if (motor1_torque < 4000 || motor2_torque < 4000)
        //         {
        //             joint[4].ajustJointTarget(-12.5f);
        //         }
        //         else
        //         {
        //             joint[4].setMode(JointMode::disabled);
        //             joint[5].setMode(JointMode::disabled);

        //             joint[4].calibrate();

        //             joint[4].setJointMapping(Reduction_ratio2, -86.46f);
        //             joint[4].setJointParam(-86.46, 85.24, -10, 10);

        //             joint[4].setMode(JointMode::angle);
        //             joint[5].setMode(JointMode::angle);
        //             J5_J6_reset = 1;
        //         }
        //    }

        //    if (J4_reset_prepared == 0)
        //    {
        //         float j4_current_angle = joint[3].getCurrentAngle();
        //         float angle_diff = j4_current_angle - 30.0f;    

        //         if (abs(angle_diff) > 1.0f)
        //         {
        //             if (angle_diff > 0)
        //             {
        //                 joint[3].ajustJointTarget(-0.1f);
        //             }
        //             else
        //             {
        //                 joint[3].ajustJointTarget(-0.1f);
        //             }
        //         }
        //         else
        //         {
        //             J4_reset_prepared = 1;
        //         }
        //    }

        //    if (J3_reset == 0 && J5_J6_reset == 1 && J4_reset_prepared == 1)
        //    {
        //         if (motor4_torque < 8000)
        //         {
        //             joint[2].ajustJointTarget(5.0f);
        //         }
        //         else
        //         {
        //             joint[2].setMode(JointMode::disabled);

        //             joint[2].calibrate();

        //             joint[2].setJointMapping(-Reduction_ratio1, -50.88f);
        //             joint[2].setJointParam(-50.88f, 79.15f, -10, 10);

        //             joint[2].setMode(JointMode::angle);
        //             J3_reset = 1;
        //         }
        //    }

        //    if (J2_reset == 0 && J3_reset == 1 && J5_J6_reset == 1)
        //    {
        //         if (motor3_torque < 8000)
        //         {
        //             joint[1].ajustJointTarget(-5.0f);
        //         }
        //         else
        //         {
        //             joint[1].setMode(JointMode::disabled);

        //             joint[1].calibrate();

        //             joint[1].setJointMapping(-Reduction_ratio1, 164.08f);
        //             joint[1].setJointParam(20.91f, 164.08f, -10, 10);

        //             joint[1].setMode(JointMode::angle);
        //             J2_reset = 1;
        //         }
        //    }

           if (J2_reset == 1 && J3_reset == 1 && J5_reset == 1 && J6_reset == 1 && J4_reset_prepared == 1 /*&& gripper.isReseted()*/&& J6_reset_prepared == 1)
           {
                reset_ok = 1;

                // gripper.init(5000.0f, 4000.0f, 2000.0f, 0.0278, 70.9, 0, 68); 

                return true;
           }
        }

    }

    return false;
}

void Reset_action(Event_t **pEvent)
{
    static ActionGroup_enum action_enum = ActionGroup_enum::NuLLAction;
    static uint8_t Add_onlyOnce = 0;
    static bool DG_reset = false;

    WS_indicator[0] = WS_Purple;

    /*刚上电的时候电机的连接检查，还有PID更新*/
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
    {

        /*-----------回到一个初始折叠位姿action------------*/

        /*J2、3、5关节复位*/
        if (((DR16.GetS1() == 1 && DR16.GetS2() == 1) || DR16.IsKeyPress(DR16_KEY_F) || (VT13.GetModeSwitch() == MS_Status_Typedef::MS_N && VT13.GetTrigger())) && Robot_Prepared == 0)
        {
            DG_reset = Manipulator_TorqueReset();
        }
        // 复位动作组
        else if ((DR16.IsKeyPress(DR16_KEY_Z) || VT13.IsKeyPress(VT13_KEY_Z)) && Add_onlyOnce == 0 && joint[0].mapping.b != 0)
        {

            action_enum = ActionGroup_enum::ResetNoOre;
            if (motor[0].getStatus() == MotorStatus::ready)
            {
                joint[0].updateJointData();
                joint[3].updateJointData();

                ResetActionNoOre_Joint[0][0] = joint[0].getCurrentAngle(); // 保持当前位置不变
                ResetActionNoOre_Joint[0][3] = joint[3].getCurrentAngle(); // 保持当前位置不变
            }
            Add_onlyOnce = 1;
        }
        else
        {
            action_enum = ActionGroup_enum::NuLLAction;
        }

        /*选择动作组*/
        ActionGroup_Choose(joint_trajectory_point_array, action_enum, Add_onlyOnce);

        /*动作组执行*/
        ActionGroup_Execute(joint, joint_trajectory_point_array, 6, Add_onlyOnce);

        // toBottomdata.L_gripper = 0; // 夹爪待命
        // toBottomdata.R_gripper = 0; // 夹爪待命


        // 单片机重新复位操作
        if ((DR16.IsKeyPress(DR16_KEY_Z) && DR16.IsKeyPress(DR16_KEY_CTRL) && DR16.IsKeyPress(DR16_KEY_SHIFT)) || (VT13.IsKeyPress(VT13_KEY_Z) && VT13.IsKeyPress(VT13_KEY_CTRL) && VT13.IsKeyPress(VT13_KEY_SHIFT)))
        {
            // 整车进入复位状态，所有机构停止运动

            vTaskDelay(1000);
            __set_FAULTMASK(1); // 关闭所有中断
            NVIC_SystemReset(); // 复位
        }
        
    }
    else
    {
        TopStateMachine.pSonEvent = &ToLost;
    }

    /*底盘移动操作*/
    if(VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if(VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_MoveCtrl(toBottomdata);
        }
        else if(!VT13_IsButtonActive(ButtonType::FN1))
        {
            VT13_KeyBoard_MoveCtrl(toBottomdata);
        }

        VT13_Manipulater_JointCtrl();

        VT13_Grip();
    }
    if(DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED)
    {
        if (DR16.GetS1() == 2)
        {
            DR16_MoveCtrl(toBottomdata);
        }
        else
        {
            KeyBoard_MoveCtrl(toBottomdata);
        }
        DR16_Manipulater_JointCtrl();

        DR16_Grip();

        DR16_StoreGrippers(toBottomdata);
    }


    if (DG_reset == true)
    {
        static uint8_t joints_reset_done = 0;
        static uint8_t j4_reset_done, j5_reset_done, j6_reset_done = 0;

        if (joints_reset_done)
        {
            Robot_Prepared = 1;
            Reset_exitAction();
        }

        if (motor[3].getStatus() == MotorStatus::ready && motor[4].getStatus() == MotorStatus::ready && motor[5].getStatus() == MotorStatus::ready)
        {
            float j5_current_angle = joint[4].getCurrentAngle();
            float j5_angle_diff = j5_current_angle - 0.0f;    

            float j6_current_angle = joint[5].getCurrentAngle();
            float j6_angle_diff = j6_current_angle - 0.0f;    

            float j4_current_angle = joint[3].getCurrentAngle();
            float j4_angle_diff = j4_current_angle - 0.0f;    

            if (j4_reset_done == 0)
            {
                float j4_current_angle = joint[3].getCurrentAngle();
                float angle_diff = j4_current_angle - 0.0f;    

                if (abs(angle_diff) > 1.0f)
                {
                    if (angle_diff > 0)
                    {
                        joint[3].ajustJointTarget(0.1f);
                    }
                    else
                    {
                        joint[3].ajustJointTarget(-0.1f);
                    }
                }
                else
                {
                    j4_reset_done = 1;
                }
            }

            if (j5_reset_done == 0 && j4_reset_done == 1)
            {
                if (abs(j5_angle_diff) > 1.0f)
                {
                    if (j5_angle_diff > 0)
                    {
                        joint[4].ajustJointTarget(-5.0f);
                    }
                    else
                    {
                        joint[4].ajustJointTarget(5.0f);
                    }
                }
                else                
                {
                    j5_reset_done = 1;
                }
            }

            // if (j6_reset_done == 0 && j5_reset_done == 1 && j4_reset_done == 1)
            // {
            //     if (abs(j6_angle_diff) > 1.0f)
            //     {
            //         if (j6_angle_diff > 0)
            //         {
            //             joint[5].ajustJointTarget(-5.0f);
            //         }
            //         else
            //         {
            //             joint[5].ajustJointTarget(5.0f);
            //         }
            //     }
            //     else                
            //     {
            //         j6_reset_done = 1;
            //     }
            // }

            if (j4_reset_done == 1 && j5_reset_done == 1 /*&& j6_reset_done == 1*/)
            {
                joints_reset_done = 1;
            }
        }
    }
}

void Reset_exitAction()
{
    if (DR16.GetStatus() == LinkageStatus_Typedef::ESTABLISHED || (VT13.GetModeSwitch() != MS_Status_Typedef::MS_C && VT13.GetStatus() == LinkageStatus_Typedef::ESTABLISHED))
        TopStateMachine.pSonEvent = &ToMobile;
    else
    {
        TopStateMachine.pSonEvent = &ToLost;
    }
}