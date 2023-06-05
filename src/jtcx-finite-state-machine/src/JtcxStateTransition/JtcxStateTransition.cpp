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

#include "JtcxStateTransitionTable.hpp"
#include "JtcxExternalEventsFactory.hpp"

using namespace JTCX;
namespace JTCX
{

    ExternalEventsFactory::ExternalEventsFactory()
    {
        G_enum_class_reflector_low_events[0]  ="null";
        G_enum_class_reflector_low_events[1]  ="L0001";
        G_enum_class_reflector_low_events[2]  ="L0002";
        G_enum_class_reflector_low_events[3]  ="L0003";
        G_enum_class_reflector_low_events[4]  ="L0004";
        G_enum_class_reflector_low_events[5]  ="L0005";
        G_enum_class_reflector_low_events[6]  ="L0006";
        G_enum_class_reflector_low_events[7]  ="L0007";
        G_enum_class_reflector_low_events[8]  ="L0008";
        G_enum_class_reflector_low_events[9]  ="L0009";
        G_enum_class_reflector_low_events[10] ="L0010";
        G_enum_class_reflector_low_events[11] ="L0011";
        G_enum_class_reflector_low_events[12] ="L0012";
        G_enum_class_reflector_low_events[13] ="L0013";
        G_enum_class_reflector_low_events[14] ="L0014";
        G_enum_class_reflector_low_events[15] ="L0015";
        G_enum_class_reflector_low_events[14] ="L0016";
        G_enum_class_reflector_low_events[15] ="L0017";
        G_enum_class_reflector_low_events[14] ="L0018";
        G_enum_class_reflector_low_events[15] ="L0019";
        G_enum_class_reflector_low_events[14] ="L0020";
        G_enum_class_reflector_low_events[15] ="L0021";
        G_enum_class_reflector_low_events[14] ="L0022";
        G_enum_class_reflector_low_events[15] ="L0023";

        G_enum_class_reflector_mid_events[0]  ="null";
        G_enum_class_reflector_mid_events[1]  ="M0001";
        G_enum_class_reflector_mid_events[2]  ="M0002";
        G_enum_class_reflector_mid_events[3]  ="M0003";
        G_enum_class_reflector_mid_events[4]  ="M0004";
        G_enum_class_reflector_mid_events[5]  ="M0005";
        G_enum_class_reflector_mid_events[6]  ="M0006";
        G_enum_class_reflector_mid_events[7]  ="M0007";
        G_enum_class_reflector_mid_events[8]  ="M0008";
        G_enum_class_reflector_mid_events[9]  ="M0009";
        G_enum_class_reflector_mid_events[10] ="M0010";
        G_enum_class_reflector_mid_events[11] ="M0011";
        G_enum_class_reflector_mid_events[12] ="M0012";
        G_enum_class_reflector_mid_events[13] ="M0013";
        G_enum_class_reflector_mid_events[14] ="M0014";
        G_enum_class_reflector_mid_events[15] ="M0015";
        G_enum_class_reflector_mid_events[16] ="M0016";
        G_enum_class_reflector_mid_events[17] ="M0017";
        G_enum_class_reflector_mid_events[18] ="M0018";
        G_enum_class_reflector_mid_events[19] ="M0019";
        G_enum_class_reflector_mid_events[20] ="M0020";
        G_enum_class_reflector_mid_events[21] ="M0021";
        G_enum_class_reflector_mid_events[22] ="M0022";
        G_enum_class_reflector_mid_events[23] ="M0023";

        G_enum_class_reflector_high_events[0]  ="null";
        G_enum_class_reflector_high_events[1]  ="H0001";
        G_enum_class_reflector_high_events[2]  ="H0002";
        G_enum_class_reflector_high_events[3]  ="H0003";
        G_enum_class_reflector_high_events[4]  ="H0004";
        G_enum_class_reflector_high_events[5]  ="H0005";
        G_enum_class_reflector_high_events[6]  ="H0006";
        G_enum_class_reflector_high_events[7]  ="H0007";
        G_enum_class_reflector_high_events[8]  ="H0008";
        G_enum_class_reflector_high_events[9]  ="H0009";
        G_enum_class_reflector_high_events[10] ="H0010";
        G_enum_class_reflector_high_events[11] ="H0011";
        G_enum_class_reflector_high_events[12] ="H0012";
        G_enum_class_reflector_high_events[13] ="H0013";
        G_enum_class_reflector_high_events[14] ="H0014";
        G_enum_class_reflector_high_events[15] ="H0015";
        G_enum_class_reflector_high_events[16] ="L0016";
        G_enum_class_reflector_high_events[17] ="L0017";
        G_enum_class_reflector_high_events[18] ="L0018";
        G_enum_class_reflector_high_events[19] ="L0019";
        G_enum_class_reflector_high_events[20] ="L0020";
        G_enum_class_reflector_high_events[21] ="L0021";
        G_enum_class_reflector_high_events[22] ="L0022";
        G_enum_class_reflector_high_events[23] ="L0023";

    }


    JtcxStateTransitionTable::JtcxStateTransitionTable()
    {
        /// @brief low event determined transition relationship 
        std::map<LowLevelExternalEvents,RobotStateType> l_1 ;
        std::map<LowLevelExternalEvents,RobotStateType> l_2 ;
        std::map<LowLevelExternalEvents,RobotStateType> l_3 ;

        /// @brief transition relationship
        l_1[L0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_lowEvent[RobotStateType_cleaning]=l_1;

        l_2[L0006]=RobotStateType_avoidance;
        JtcxStateTransitionTable_lowEvent[RobotStateType_cleaning]=l_2;

        l_3[L0005]=RobotStateType_faulting;
        JtcxStateTransitionTable_lowEvent[RobotStateType_cleaning]=l_3;

 
        /// @brief mid event determined transition relationship 
        std::map<MidLevelExternalEvents,RobotStateType> m_1;
        std::map<MidLevelExternalEvents,RobotStateType> m_2;
        std::map<MidLevelExternalEvents,RobotStateType> m_3;
        std::map<MidLevelExternalEvents,RobotStateType> m_4;
        std::map<MidLevelExternalEvents,RobotStateType> m_5;        
        
        m_1[M0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_midEvent[RobotStateType_cleaning]=m_1;

        m_2[M0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_midEvent[RobotStateType_cleaning]=m_2;

        m_3[M0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_midEvent[RobotStateType_cleaning]=m_3;


        /// @brief high event determined transition relationship 
        std::map<HighLevelExternalEvents,RobotStateType>h_1;
        std::map<HighLevelExternalEvents,RobotStateType>h_2;
        std::map<HighLevelExternalEvents,RobotStateType>h_3;
        std::map<HighLevelExternalEvents,RobotStateType>h_4;
        std::map<HighLevelExternalEvents,RobotStateType>h_5;

        h_1[H0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_highEvent[RobotStateType_cleaning]=h_1;

        h_2[H0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_highEvent[RobotStateType_cleaning]=h_2;

        h_3[H0007]=RobotStateType_cleaning;
        JtcxStateTransitionTable_highEvent[RobotStateType_cleaning]=h_3;


        JtcxStateTransitionTable_abnormalType[low_battery_level]=RobotStateType_idling;
    }

}