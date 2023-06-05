/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxEnumsDefinitions.hpp
Version       : 固件v1.1.1rc
Author        : JianQiang 
Created       : 2022/11/25
Last Modified :
Description   : Necessary enums definitions
Function List :
History       : first edition ----20221125
******************************************************************************/
#pragma once

namespace JTCX
{
    //Opening and closing degree of front baffle
    enum RubType
    { 
        SMALL_RUB      = 0,
        MEDIAN_RUB     = 1,
        BIG_RUB        = 3,
        HUGE_RUB       = 4,
    };

    //Cleaning method: edge, covering, both
    enum CleaningMode
    {
        EDGE_CLEANING     =0,
        COVERAGE_CLEANING =1,
        BOTH_CLEANING     =2,
    };

    //Intermittent scanning
    enum CleaningIntermittent
    {
        FROM_SCRATCH      =0,
        FROM_BREAKPOINT   =1,
    };

    // msg coming from tracking result
    enum CleaningException
    {
        COMPLETE_SWEEPING    =0,
        ///-------------- pursuit error ---------------------
        MANUAL_INTERRUPT     =1,
        PURSUIT_FAIL         =2,
        // --------------- unusally reasons
        NODE_EXIT            =3,
        PURE_PURSUIT_EXIST   =4,
        UNKNOWN_ERROR        =5,
    };
    
    enum LOG_LEVEL
    {
        TRACE = 0,
        DEBUG = 1,
        INFO = 2,
        WAIN = 3,
        ERROR = 4,
        CRITIAL = 5,
        OFF = 6,
    };
}