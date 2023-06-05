/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxErrorCodes.hpp
Version       : 固件v1.1.1rc
Author        : JianQiang 
Created       : 2022/11/25
Last Modified :
Description   : error code
Function List :
History       : first edition ----20221125
******************************************************************************/
#pragma once

namespace JTCX
{

    enum JTCXErrorCode
    {
        NO_ERROR             = 0,

        BATTERY_ERROR        = 1,

        PURSUIT_ERROR        = 2,

        CHASSIS_ERROR        = 3,

        ERROR3               = 4,

        ERROR4               = 5,
    };

}