#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : JtcxMsgQueueManagerBase.hpp
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/10
Last Modified :
Description   : FSM
Function List :
History       : 20221012_v1.0.0
******************************************************************************/

#include "JtcxMsgQueueManagerBase.hpp"
#include "JtcxFiniteStateMachine.hpp"

class RobotStateBase;

namespace JTCX
{

    class JtcxFSM: public JtcxFSMBase
    {
        public:

            JtcxFSM();

            virtual ~JtcxFSM() override;

            /// @brief Query current internal and external event information
            /// @return Robot state 
            virtual JTCX::ControlMessage getRobotCurrentInfor() override;

            /// @brief start FSM
            virtual void startFSM() override;
        
        private:

            /// @brief switch state 
            void switchState(RobotStateType next_type);

            /// @brief handle event
            void handleEvent();

            /// @brief current state
            std::unique_ptr<RobotStateBase> _robot_state_handler=nullptr; 

            /// @brief event handler
            ExternalEventsBase* _external_event_handler=nullptr; 

            /// @brief Take out the queue elements at the control end one by one and sort them by priority
            static ConcurrenceQueue<JTCX::ControlMessage>* G_state_waiting_queue; 

    };

}