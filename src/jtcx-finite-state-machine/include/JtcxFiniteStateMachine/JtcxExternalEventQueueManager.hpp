#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : JtcxExternalEventQueueManager.hpp
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/10
Last Modified :
Description   : FSM
Function List :
History       : 20221012_v1.0.0
******************************************************************************/

#include "JtcxCommon.hpp"
#include "JtcxMsgQueueManagerBase.hpp"
#include "JtcxExternalEventsBase.hpp"
#include "JtcxExternalEventsFactory.hpp"

namespace JTCX{
    class ExternalEventQueueManager:private MsgQueueManagerBase
    {
        public:

            ~ExternalEventQueueManager(){};
            
            /// @brief Query current Abnormal type
            /// @return Abnormal type
            virtual JTCX::RobotAbnormalType getAbnormalState() override;

            /// @brief Query current state
            /// @return Robot state 
            virtual JTCX::RobotStateType getRobotState() override;

            /// @brief Query current Abnormal type and pop back from queue
            /// @return Abnormal type 
            virtual JTCX::RobotAbnormalType popAbnormalState() override;

            /// @brief pish abnormal state into queue
            virtual void pushAbnormalState(JTCX::RobotAbnormalType abnormalType) override;

            /// @brief Called by external event class
            /// @param low_event 
            void pushPenddingLowEvent(JTCX::LowLevelExternalEvents low_event);

            /// @brief Called by external event class
            /// @param mid_event 
            void pushPenddingMidEvent(JTCX::MidLevelExternalEvents mid_event);

            /// @brief Called by external event class
            /// @param high_event 
            void pushPenddingHighEvent(JTCX::HighLevelExternalEvents high_event);
            
            /// @brief 
            /// @param low_event 
            JTCX::ExternalEventsLevel popPenddingEventLevel();

            /// @brief used by listener thread to process
            /// @param low_event 
            JTCX::LowLevelExternalEvents popPenddingLowEvent();

            /// @brief used by listener thread to process
            /// @param mid_event 
            JTCX::MidLevelExternalEvents popPenddingMidEvent();
            
            /// @brief used by listener thread to process
            /// @param high_event 
            JTCX::HighLevelExternalEvents popPenddingHighEvent();
            
            /// @brief set event handler finished or not;
            void setEventFinishedFlag(bool finished);

            /// @brief get event handler finished flag state;
            /// @return true or false
            bool getEventFinishedFlag();

            /// @brief external event listening is placed in the sub thread
            static void eventsListener(ExternalEventQueueManager*ptr);

            /// singleton constructor 
            static ExternalEventQueueManager& getEventManager();



        private:

            /// @brief External event handling flag
            bool _event_handler_finished= false;

            /// @brief External event handling flag mutex
            std::mutex _event_handler_finished_mutex;

            /// @brief External event handling flag cv
            std::condition_variable _event_handler_finished_cv;

            static ConcurrenceQueue<JTCX::ExternalEventsLevel>* G_events_level_queue;

            /// @brief lowlevel_event_queue
            static ConcurrenceQueue<JTCX::LowLevelExternalEvents>* G_lowlevel_event_queue;
            
            /// @brief midlevel_event_queue
            static ConcurrenceQueue<JTCX::MidLevelExternalEvents>* G_midlevel_event_queue;
            
            /// @brief highlevel_event_queue
            static ConcurrenceQueue<JTCX::HighLevelExternalEvents>* G_highlevel_event_queue;

            //singleton mode;
            ExternalEventQueueManager();

            std::thread *_events_listener_thread  = nullptr;  

    };
}