/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxUtilTools.hpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/12/03
Last Modified :
Description   : util tools
Function List : 
History       : first edition ----20221203
******************************************************************************/
#pragma once
#include <ros/ros.h>
#include <uuid/uuid.h>
#include "JtcxMacroDefinition.hpp"

namespace JTCX
{
    class util
    {
        public:
            ros::WallTime _start_time;
            ros::WallTime _stamp_time;

        public:
            util(){};
            ~util(){};

            static double getCurrentDoubleTime()
            {
                return ros::WallTime::now().toSec();
            }
            inline void setStartTime()
            {
                _start_time = ros::WallTime::now();
            }
            inline void setStampTime()
            {
                _stamp_time = ros::WallTime::now();
            }
            inline double getStartTime()
            {
                return _start_time.toSec();
            }
            inline double getStampTime()
            {
                return _stamp_time.toSec();
            }
            inline double getDurationTime()
            {
                return getStampTime() - getStartTime();
            }
            static bool getCurrentMapName(std::string& m)
            {
                return ros::param::get("/selected_map", m);
            }
            static std::string genUUID() noexcept
            {
                uuid_t uuid;
                char str[36];
                uuid_generate(uuid);
                uuid_unparse(uuid, str);
                return std::string(str);
            }
            /**
             * @brief format: "xx:xx"
             * 
             * @param str 
             * @return std::pair<int, int> hour min
             */
            static std::pair<int, int> fmtHourAndMin(std::string str){
                auto res = std::make_pair(-1, -1); 
                int pos = str.find(':');
                int sz = str.size();
                if(pos != str.npos){
                    res.first = std::stoi(str.substr(0, pos));
                    res.second = std::stoi(str.substr(pos+1, sz-pos-1));
                }
                return res;   
            }
            static int convertHourMinSecsToSecs(int h, int m, int s){
                return 3600 * h + 60 * m + s;
            }
            
    };
}