#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : JtcxInternalEventQueueManager.cpp
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/10
Last Modified :
Description   : FSM
Function List :
History       : 20221013_v1.0.0
******************************************************************************/

#include "JtcxInternalEventQueueManager.hpp"

namespace JTCX{
InternalEventQueueManager::InternalEventQueueManager()
{
    //Abstract Class Initialization
    //Only store abnormal state, do not put healthy state into
    G_abnormal_console_queue = new  ConcurrenceQueue<JTCX::RobotAbnormalType>(1); 
        
    G_state_console_queue  = new  ConcurrenceQueue<JTCX::ControlMessage>(100); 

    // Inheritance Class Initialization
    _abnormal_listener_thread = new std::thread(&InternalEventQueueManager::abnormalListener,this);

}

InternalEventQueueManager::~InternalEventQueueManager()
{
    
    if(G_abnormal_console_queue)
    {
        delete G_abnormal_console_queue;

        G_abnormal_console_queue=nullptr;
    }
    if(G_state_console_queue)
    {
        delete G_state_console_queue;

        G_state_console_queue=nullptr;
    }
    if(_abnormal_listener_thread)
    {
        _abnormal_listener_thread->join();
        delete _abnormal_listener_thread;
        _abnormal_listener_thread = nullptr;
    }
}
       
JTCX::RobotAbnormalType InternalEventQueueManager::popAbnormalState()
{
    return G_abnormal_console_queue->pop();
}

JTCX::RobotAbnormalType InternalEventQueueManager::getAbnormalState()
{
    return G_abnormal_console_queue->check_up();
}

JTCX::RobotStateType InternalEventQueueManager::getRobotState()
{

    JTCX::ControlMessage temp =G_state_console_queue->check_up();

    return temp.currentState;
}

void InternalEventQueueManager::pushAbnormalState(JTCX::RobotAbnormalType abnormalType)
{

    G_abnormal_console_queue->push(abnormalType);
    
}

void InternalEventQueueManager::abnormalListener(InternalEventQueueManager *ptr)
{
    while(1)
    {
        JTCX::ControlMessage current_swith_info;

        current_swith_info.abnormalType   = ptr->popAbnormalState();//pop contains _cv

        current_swith_info.currentState = ptr->getRobotState();
        
        ptr->G_state_console_queue->push(current_swith_info);

    }
}

JTCX::ControlMessage InternalEventQueueManager::getRobotCurrentInfor()
{
    return G_state_console_queue->check_up();
}

JTCX::ControlMessage InternalEventQueueManager::popRobotCurrentInfor()
{
    return G_state_console_queue->pop();
}

void InternalEventQueueManager::launchEmergentFlag()
{

    std::unique_lock<std::mutex>lock(_emergent_flag_mutex);

    _emergent_flag=true;

    _emergent_flag_cv.notify_one();


}

bool InternalEventQueueManager::getEmergentFlag()
{
    std::unique_lock<std::mutex>lock(_emergent_flag_mutex);
    return _emergent_flag;

}

InternalEventQueueManager& InternalEventQueueManager::getEventManager()
{
    static InternalEventQueueManager I_Manager;
    return I_Manager;
}
}