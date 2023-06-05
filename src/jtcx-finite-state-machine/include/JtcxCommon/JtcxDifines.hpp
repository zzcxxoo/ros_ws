#pragma once
#include "JtcxCommon.hpp"

namespace JTCX{

        enum RobotAbnormalType
        {
            /// @brief FSM controllable state
            low_battery_level           = 0,

            /// @brief 
            low_water_level             = 1,
            
            /// @brief 
            high_temperature_warning    = 2,

            /// @brief Must enter faulting state
            emergency                   = 3,

            /// @brief finish state
            state_finish                = 4
        };

        enum RobotStateType
        {
            /// @brief 
            RobotStateType_sleeping    = 0,
            
            /// @brief 
            RobotStateType_cleaning    = 1,
            
            /// @brief 
            RobotStateType_homing      = 2,
            
            /// @brief 
            RobotStateType_idling      = 3,
            
            /// @brief 
            RobotStateType_mapping     = 4,

            /// @brief 
            RobotStateType_avoidance   = 5,

            /// @brief 
            RobotStateType_faulting    = 6,
        };

        enum ExternalEventsLevel
        {
            /// @brief 
            lowLevel    = 0,
            
            /// @brief 
            midLevel    = 1,
            
            /// @brief 
            highLevel   = 2,
        };


        enum LowLevelExternalEvents
        {
            /// @brief 
            InvalidEvent_low    = 0,
            
            /// @brief 
            L0001               = 1,

            /// @brief 
            L0002               = 2,
            
            /// @brief 
            L0003               = 3,
            
            /// @brief 
            L0004               = 4,
            
            /// @brief 
            L0005               = 5,

            /// @brief 
            L0006               = 6,

            /// @brief 
            L0007               = 7,
            
            /// @brief 
            L0008               = 8,
            
            /// @brief 
            L0009               = 9,
            
            /// @brief 
            L0010               = 10,

            /// @brief 
            L0011               = 11,

            /// @brief 
            L0012               = 12,
            
            /// @brief 
            L0013               = 13,
            
            /// @brief 
            L0014               = 14,
            
            /// @brief 
            L0015               = 15,

            /// @brief 
            L0016               = 16,

            /// @brief 
            L0017               = 17,
            
            /// @brief 
            L0018               = 18,
            
            /// @brief 
            L0019               = 19,
            
            /// @brief 
            L0020               = 20,

            /// @brief 
            L0021               = 21,
            
            /// @brief 
            L0022               = 22,
            
            /// @brief 
            L0023               = 23,
        };
        

        enum MidLevelExternalEvents
        {
            /// @brief 
            InvalidEvent_mid    = 0,
            
            /// @brief 
            M0001               = 1,

            /// @brief 
            M0002               = 2,
            
            /// @brief 
            M0003               = 3,
            
            /// @brief 
            M0004               = 4,
            
            /// @brief 
            M0005               = 5,

            /// @brief 
            M0006               = 6,

            /// @brief 
            M0007               = 7,
            
            /// @brief 
            M0008               = 8,
            
            /// @brief 
            M0009               = 9,
            
            /// @brief 
            M0010               = 10,

            /// @brief 
            M0011               = 11,

            /// @brief 
            M0012               = 12,
            
            /// @brief 
            M0013               = 13,
            
            /// @brief 
            M0014               = 14,
            
            /// @brief 
            M0015               = 15,

            /// @brief 
            M0016               = 16,

            /// @brief 
            M0017               = 17,
            
            /// @brief 
            M0018               = 18,
            
            /// @brief 
            M0019               = 19,
            
            /// @brief 
            M0020               = 20,

            /// @brief 
            M0021               = 21,
            
            /// @brief 
            M0022               = 22,
            
            /// @brief 
            M0023               = 23,
        };

        enum HighLevelExternalEvents
        {
            /// @brief 
            InvalidEvent_high   = 0,
            
            /// @brief 
            H0001               = 1,

            /// @brief 
            H0002               = 2,
            
            /// @brief 
            H0003               = 3,
            
            /// @brief 
            H0004               = 4,
            
            /// @brief 
            H0005               = 5,

            /// @brief 
            H0006               = 6,

            /// @brief 
            H0007               = 7,
            
            /// @brief 
            H0008               = 8,
            
            /// @brief 
            H0009               = 9,
            
            /// @brief 
            H0010               = 10,

            /// @brief 
            H0011               = 11,

            /// @brief 
            H0012               = 12,
            
            /// @brief 
            H0013               = 13,
            
            /// @brief 
            H0014               = 14,
            
            /// @brief 
            H0015               = 15,

            /// @brief 
            H0016               = 16,

            /// @brief 
            H0017               = 17,
            
            /// @brief 
            H0018               = 18,
            
            /// @brief 
            H0019               = 19,
            
            /// @brief 
            H0020               = 20,

            /// @brief 
            H0021               = 21,
            
            /// @brief 
            H0022               = 22,
            
            /// @brief 
            H0023               = 23,
        };
}