#pragma once

/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : ExternalEventsFactory
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/17
Last Modified :
Description   : JtcxStateTransitionTable
Function List :
History       : 20221017_v1.0.0
******************************************************************************/

#include "JtcxCommon.hpp"

using namespace JTCX;
namespace JTCX
{

    class JtcxStateTransitionTable
    {
        public:

            ~JtcxStateTransitionTable();

            /// @brief 
            /// @param current_state 
            /// @param event_code 
            /// @return 
            RobotStateType getNextStateType(RobotStateType current_state,LowLevelExternalEvents event_code);

            /// @brief 
            /// @param current_state 
            /// @param event_code 
            /// @return 
            RobotStateType getNextStateType(RobotStateType current_state,MidLevelExternalEvents event_code);

            /// @brief 
            /// @param current_state 
            /// @param event_code 
            /// @return 
            RobotStateType getNextStateType(RobotStateType current_state,HighLevelExternalEvents event_code);

            /// @brief 
            /// @param current_state 
            /// @param abnormal_type 
            /// @return 
            RobotStateType getNextStateType(RobotStateType current_state,RobotAbnormalType abnormal_type);

            static JtcxStateTransitionTable& getTable();

        private:
            JtcxStateTransitionTable();

            std::map<RobotStateType,std::map<LowLevelExternalEvents,RobotStateType>> JtcxStateTransitionTable_lowEvent;

            std::map<RobotStateType,std::map<MidLevelExternalEvents,RobotStateType>> JtcxStateTransitionTable_midEvent;

            std::map<RobotStateType,std::map<HighLevelExternalEvents,RobotStateType>>JtcxStateTransitionTable_highEvent;

            std::map<RobotAbnormalType,RobotStateType> JtcxStateTransitionTable_abnormalType;

    };


    RobotStateType JtcxStateTransitionTable::getNextStateType(RobotStateType current_state,LowLevelExternalEvents event_code)
    {
        return JtcxStateTransitionTable_lowEvent[current_state][event_code];
    }

    RobotStateType JtcxStateTransitionTable::getNextStateType(RobotStateType current_state,MidLevelExternalEvents event_code)
    {
        return JtcxStateTransitionTable_midEvent[current_state][event_code];
    }

    RobotStateType JtcxStateTransitionTable::getNextStateType(RobotStateType current_state,HighLevelExternalEvents event_code)
    {
        return JtcxStateTransitionTable_highEvent[current_state][event_code];
    }

    RobotStateType JtcxStateTransitionTable::getNextStateType(RobotStateType current_state,RobotAbnormalType abnormal_type)
    {
        return JtcxStateTransitionTable_abnormalType[abnormal_type];
    }

    JtcxStateTransitionTable& JtcxStateTransitionTable::getTable()
    {

        static JtcxStateTransitionTable Table;
            
        return Table;
    }

}