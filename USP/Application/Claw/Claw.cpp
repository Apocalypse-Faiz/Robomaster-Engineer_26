#include "Claw.h"

#include <math.h>

void Gripper::update() 
{
    if(!servo) 
    {
        state = GripperState::ERROR;
        return;
    }

    updateCurrentPosition();
}

void Gripper::updateControl() 
{
    if(state == GripperState::ERROR) 
    {
        return;
    }

    if(checkMechanicalLimit()) 
    {
        emergencyStop();
        return;
    }

    switch(state) 
    {
        case GripperState::OPENING:
            target_position = min_position;
            moveTowardsTarget();
            break;

        case GripperState::CLOSING:
            target_position = max_position;
            moveTowardsTarget();
            break;
            
        case GripperState::IDLE:
        default:
            break;
    }
}

void Gripper::executeOutput() 
{
    if(!servo || state == GripperState::ERROR)
    {
        return;
    }

    if (!pwm_started)
    {
        servo->PWM_Start();
        pwm_started = true;
    }

    servo->setTarget(current_position);
}

void Gripper::open() 
{
    if(state == GripperState::ERROR) return;

    state = GripperState::OPENING;
}

void Gripper::close() 
{
    if(state == GripperState::ERROR) return;

    state = GripperState::CLOSING;
}

void Gripper::emergencyStop() 
{
    state = GripperState::IDLE;
    target_position = current_position;
}

GripperState Gripper::getState() const 
{
    return state;
}

float Gripper::getCurrentPosition() const 
{
    return current_position;
}

void Gripper::clearError() 
{
    if(state == GripperState::ERROR) 
    {
        state = GripperState::IDLE;
    }
}

bool Gripper::checkMechanicalLimit() 
{
    return (current_position < min_position) || (current_position > max_position);
}

void Gripper::updateCurrentPosition() 
{
    if (current_position < min_position)
    {
        current_position = min_position;
    }
    else if (current_position > max_position)
    {
        current_position = max_position;
    }
}

void Gripper::setState(GripperState state)
{
    this->state = state;
}

void Gripper::moveTowardsTarget()
{
    if (target_position < min_position)
    {
        target_position = min_position;
    }
    else if (target_position > max_position)
    {
        target_position = max_position;
    }

    if (current_position < target_position)
    {
        current_position += angle_step;
        if (current_position > target_position)
        {
            current_position = target_position;
        }
    }
    else if (current_position > target_position)
    {
        current_position -= angle_step;
        if (current_position < target_position)
        {
            current_position = target_position;
        }
    }
}
