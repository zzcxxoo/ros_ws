#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : JtcxExternalEventQueueManager.cpp
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/16
Last Modified :
Description   : FSM
Function List :
History       : 20221013_v1.0.0
******************************************************************************/


#include "JtcxExternalEventQueueManager.hpp"

namespace JTCX{
ExternalEventQueueManager::ExternalEventQueueManager()
{

    G_events_level_queue =new ConcurrenceQueue<JTCX::ExternalEventsLevel>(1);

    G_lowlevel_event_queue = new ConcurrenceQueue<JTCX::LowLevelExternalEvents>(1);
    
    G_midlevel_event_queue = new ConcurrenceQueue<JTCX::MidLevelExternalEvents>(1);
    
    G_highlevel_event_queue = new ConcurrenceQueue<JTCX::HighLevelExternalEvents>(1);

    _events_listener_thread = new std::thread(&ExternalEventQueueManager::eventsListener,this);

}

ExternalEventQueueManager::~ExternalEventQueueManager()
{

    if(G_lowlevel_event_queue)
    {
        delete G_lowlevel_event_queue;

        G_lowlevel_event_queue=nullptr;
    }
    if(G_midlevel_event_queue)
    {
        delete G_midlevel_event_queue;

        G_midlevel_event_queue=nullptr;
    }
    if(G_highlevel_event_queue)
    {
        delete G_highlevel_event_queue;

        G_highlevel_event_queue=nullptr;
    }

    if(_events_listener_thread)
    {
        _events_listener_thread->join();
        delete _events_listener_thread;
        _events_listener_thread = nullptr;
    }

}

JTCX::RobotAbnormalType ExternalEventQueueManager::popAbnormalState()
{
    return G_abnormal_console_queue->pop();
}

JTCX::RobotAbnormalType ExternalEventQueueManager::getAbnormalState()
{
    return G_abnormal_console_queue->check_up();
}

JTCX::RobotStateType ExternalEventQueueManager::getRobotState()
{

    JTCX::ControlMessage temp =G_state_console_queue->check_up();

    return temp.currentState;
}

void ExternalEventQueueManager::pushAbnormalState(JTCX::RobotAbnormalType abnormalType)
{

    G_abnormal_console_queue->push(abnormalType);

}

void ExternalEventQueueManager::pushPenddingLowEvent(JTCX::LowLevelExternalEvents low_event)
{
    G_events_level_queue->push(JTCX::lowLevel);
    G_lowlevel_event_queue->push(low_event);
}

void ExternalEventQueueManager::pushPenddingMidEvent(JTCX::MidLevelExternalEvents mid_event)
{
    G_events_level_queue->push(JTCX::midLevel);
    G_midlevel_event_queue->push(mid_event);
}
 
void ExternalEventQueueManager::pushPenddingHighEvent(JTCX::HighLevelExternalEvents high_event)
{
    G_events_level_queue->push(JTCX::highLevel);
    G_highlevel_event_queue->push(high_event);
}

JTCX::ExternalEventsLevel ExternalEventQueueManager::popPenddingEventLevel()
{
   return  G_events_level_queue->pop();
}

JTCX::LowLevelExternalEvents ExternalEventQueueManager::popPenddingLowEvent()
{
    return G_lowlevel_event_queue->pop();
}

JTCX::MidLevelExternalEvents ExternalEventQueueManager::popPenddingMidEvent()
{
    return G_midlevel_event_queue->pop();

}
            
JTCX::HighLevelExternalEvents ExternalEventQueueManager::popPenddingHighEvent()
{
    return G_highlevel_event_queue->pop();

}

void ExternalEventQueueManager::setEventFinishedFlag(bool finished)
{
    std::unique_lock<std::mutex>lock(_event_handler_finished_mutex);

    _event_handler_finished=finished;

    _event_handler_finished_cv.notify_one();
}

bool ExternalEventQueueManager::getEventFinishedFlag()
{
    std::unique_lock<std::mutex>lock(_event_handler_finished_mutex);
    return _event_handler_finished;
}

void ExternalEventQueueManager::eventsListener(ExternalEventQueueManager *ptr)
{
    while(1)
    {

        JTCX::ExternalEventsLevel level= ptr->popPenddingEventLevel();
        //将当前监听到的消息，制作执行队列消息结构
        JTCX::ControlMessage current_swith_info;
        current_swith_info.abnormalType  = ptr->getAbnormalState();
        current_swith_info.currentState  = ptr->getRobotState();
        switch (level)
	    {
            case lowLevel:
            current_swith_info.lowLevelEvent = ptr->popPenddingLowEvent(); 
            current_swith_info.midLevelEvent = JTCX::InvalidEvent_mid; 
            current_swith_info.highLevelEvent= JTCX::InvalidEvent_high; 
            break;
            
            case midLevel:
            current_swith_info.lowLevelEvent = JTCX::InvalidEvent_low; 
            current_swith_info.midLevelEvent = ptr->popPenddingMidEvent();
            current_swith_info.highLevelEvent= JTCX::InvalidEvent_high; 
            break;
            
            case highLevel:
            current_swith_info.lowLevelEvent = JTCX::InvalidEvent_low; 
            current_swith_info.midLevelEvent = JTCX::InvalidEvent_mid; 
            current_swith_info.highLevelEvent= ptr->popPenddingHighEvent(); 
            break;
        }
        ptr->G_state_console_queue->push(current_swith_info);
    }
}

ExternalEventQueueManager& ExternalEventQueueManager::getEventManager()
{
    static ExternalEventQueueManager E_Manager;
    return E_Manager;
}
}