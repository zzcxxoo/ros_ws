#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : JtcxFiniteStateMachine.cpp
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/16
Last Modified :
Description   : FSM
Function List :
History       : 20221013_v1.0.0
******************************************************************************/


#include "_JtcxFiniteStateMachine.hpp"
#include "JtcxRobotCleaningState.hpp"
#include "JtcxRobotFaultingState.hpp"
#include "JtcxRobotHomingState.hpp"
#include "JtcxRobotIdlingState.hpp"
#include "JtcxRobotMappingState.hpp"
#include "JtcxRobotSleepingState.hpp"
#include "JtcxExternalEventsFactory.hpp"
#include "JtcxStateTransitionTable.hpp"

using namespace JTCX;

namespace JTCX
{


    JtcxFSMBase* instantiateJtcxFSMBase()
    {
        return new JtcxFSM();
    }


    void destroyJtcxFSMBase(JtcxFSMBase* ptr)
    {
        delete ptr;
    }

    JtcxFSM::JtcxFSM()
    {
        _robot_state_handler.reset(new RobotIdlingState());
    }

    JtcxFSM::~JtcxFSM()
    {

        if(_external_event_handler)
        {
            delete _external_event_handler;
            _external_event_handler=nullptr;
        }

    }

 
    ControlMessage JtcxFSM::getRobotCurrentInfor()
    {
        return InternalEventQueueManager::getEventManager().getRobotCurrentInfor();
    }

    /// @brief start  FSM
    void JtcxFSM::startFSM()
    {
        while(1)
        {
            ControlMessage current_msg = InternalEventQueueManager::getEventManager().popRobotCurrentInfor();
            
            //handle high level envent
            if(current_msg.highLevelEvent != InvalidEvent_high)
            {
                int index=current_msg.highLevelEvent;
                
                std::string class_name=ExternalEventsFactory::getInstance().G_enum_class_reflector_high_events[index];
                
                _external_event_handler=ExternalEventsFactory::getInstance().getEventHandler(class_name);
                
                _external_event_handler->handleEvent();

                RobotStateType next_type =JtcxStateTransitionTable::getTable().getNextStateType(current_msg.currentState,current_msg.highLevelEvent);

                switchState(next_type);

            }
        }
    }
 
    /// @brief switch state 
    void JtcxFSM::switchState(RobotStateType next_type)
    {
        //TODO table
        _robot_state_handler.reset(new RobotIdlingState());
    }

    ///handle event
    void JtcxFSM::handleEvent()
    {


    }

    std::unique_ptr<RobotStateBase> _robot_state_handler=nullptr;  
}