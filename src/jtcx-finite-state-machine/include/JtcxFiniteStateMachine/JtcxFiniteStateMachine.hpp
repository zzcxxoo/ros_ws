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

namespace JTCX
{
    class JtcxFSMBase
    {
        public:

            virtual ~JtcxFSMBase()=0;

            /// @brief Query current internal and external event information
            /// @return Robot state 
            virtual JTCX::ControlMessage getRobotCurrentInfor()=0;

            /// @brief start FSM
            virtual void startFSM()=0;
    };
    
    JtcxFSMBase* instantiateJtcxFSMBase();
    void destroyJtcxFSMBase(JtcxFSMBase* ptr);
}

