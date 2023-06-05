#pragma once
/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : JtcxInternalEventQueueManager.hpp
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/10
Last Modified :
Description   : FSM
Function List :
History       : 20221012_v1.0.0
*******************************************************************************/

#include "JtcxMsgQueueManagerBase.hpp"

namespace JTCX{
    class InternalEventQueueManager:private MsgQueueManagerBase
    {
        public:
            
            ~InternalEventQueueManager();
            
            /// @brief Query current Abnormal type
            /// @return Abnormal type
            virtual JTCX::RobotAbnormalType getAbnormalState() override;

            /// @brief Query current state
            /// @return Robot state 
            virtual JTCX::RobotStateType getRobotState() override;

            /// @brief Query current Abnormal type and pop back from queue
            /// @return Abnormal type 
            virtual JTCX::RobotAbnormalType popAbnormalState() override;

            /// @brief Set abnormal state
            virtual void pushAbnormalState(JTCX::RobotAbnormalType abnormalType) override;
            
            /// @brief The state Abnormal listener is placed on the child thread to start
            /// @param ptr 
            static void abnormalListener(InternalEventQueueManager *ptr);

            /// singleton constructor 
            static InternalEventQueueManager& getEventManager();
            
            /// @brief called by upper state machine 
            /// @return ControlMessage
            JTCX::ControlMessage getRobotCurrentInfor();
            
            /// @brief called by upper state machine 
            /// @return ControlMessage
            JTCX::ControlMessage popRobotCurrentInfor();

            /// @brief set emergent flag;
            void launchEmergentFlag();

            /// @brief get emergent flag;
            /// @return true or false
            bool getEmergentFlag();
            
        private:

            /// @brief emergent flag
            bool _emergent_flag= false;

            /// @brief emergent flag mutex
            std::mutex _emergent_flag_mutex;

            /// @brief emergent flag cv
            std::condition_variable _emergent_flag_cv;

            //Singleton mode
            InternalEventQueueManager();

            std::thread *_abnormal_listener_thread = nullptr;

            InternalEventQueueManager(const InternalEventQueueManager&);

            InternalEventQueueManager& operator=(const InternalEventQueueManager& )
            {return *this;}
    };
}