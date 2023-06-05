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

#include "JtcxCommon.hpp"

namespace JTCX
{

    struct ControlMessage
    {
        JTCX::RobotStateType currentState;

        JTCX::RobotAbnormalType abnormalType;

        JTCX::LowLevelExternalEvents lowLevelEvent;

        JTCX::MidLevelExternalEvents midLevelEvent;

        JTCX::HighLevelExternalEvents highLevelEvent;
    };


    class MsgQueueManagerBase
    {
        public:
            virtual ~MsgQueueManagerBase(){};
            
            /// @brief Query current Abnormal type
            /// @return Abnormal type
            virtual JTCX::RobotAbnormalType getAbnormalState()=0;

            /// @brief Query current state
            /// @return Robot state 
            virtual JTCX::RobotStateType getRobotState()=0;

            /// @brief Query current Abnormal type and pop back from queue
            /// @return Abnormal type 
            virtual JTCX::RobotAbnormalType popAbnormalState()=0;

            /// @brief pish abnormal state into queue
            virtual void pushAbnormalState(JTCX::RobotAbnormalType abnormalType)=0;

        protected:

            /// @brief Only store abnormal state, do not put healthy state into
            static ConcurrenceQueue<JTCX::RobotAbnormalType>* G_abnormal_console_queue; 
            
            /// @brief 
            static ConcurrenceQueue<JTCX::ControlMessage>* G_state_console_queue; 
            
    };

}