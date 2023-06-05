/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxFileHandler.cpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/11/26
Last Modified :
Description   : File and folder processing
Function List : 
History       : first edition ----20221125
******************************************************************************/
#include "JtcxAppHandler.hpp"
#include "mobile_platform_msgs/Agent.h"

namespace JTCX
{
	JtcxLogger* CleaningAppHandler::_logger = JtcxSingleton<JtcxLogger>::GetInstance();

    bool CleaningAppHandler::callAgent(std::string topic, std::string srv_id, const json& msg, std::string& res)
    {
        Agent srv;
        srv.request.trace_id = JTCX::util::genUUID();
        srv.request.service_id = srv_id;
        srv.request.data = msg.dump();
        _logger->TRACE("req: {}", srv.request.data);
        if(ros::service::call(topic, srv)){
            if(srv.response.code == 0){
                res = srv.response.result;
                return true;
            }
            _logger->ERROR("agent call fail for: {}", srv.response.msg);
        }else
            _logger->ERROR("agent offline!!");
        
        return false;
    }
    void CleaningAppHandler::uploadStatus(const CleaningContextInfo& useful_info)
    {
        static double last = ros::WallTime::now().toSec();
        if(ros::WallTime::now().toSec() - last < 2.0)   return;
        last = ros::WallTime::now().toSec();

        json jv;
        // jv["task_id"] = g_timing_task_id;
        jv["map_id"] 	= useful_info.map_id;
        jv["plan_id"] 	= useful_info.plan_id;  //_context.plan_name;
        jv["progress"]  = useful_info.process;  //_context.process;
        jv["area"]      = useful_info.area;  //_context.area;
        jv["odom"]  	= useful_info.odometry;  //_context.odometry;
        jv["duration"]  = useful_info.cleaning_time;  //_context.cleaning_time;
        
        std::string tmp;
        callAgent("/agent/props", "cleaning_progress", jv, tmp);
    }
    std::string CleaningAppHandler::genReportId() noexcept
    {
        auto now = std::time(0);
        tm *ltm = localtime(&now);
        char cr[64];
        std::sprintf(cr, "task%02d%02d%02dx%02d%02d%02d", -100 + ltm->tm_year, 1 + ltm->tm_mon,
                    ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
        return std::string(cr);
    }
    void CleaningAppHandler::uploadReport(const CleaningContextInfo& cont)
    {
        json j;
        j["report_id"] = genReportId();
        j["area"] = cont.area;
        j["average_velocity"] = cont.average_velocity;
        j["cleaning_time"] = cont.cleaning_time;
        j["mode"] = "";
        j["exception"] = cont.exception;
        j["map_id"] = cont.map_id;
        j["plan_id"] = cont.plan_id;
        j["task_id"] = "";
        j["progress"] = cont.process;
        j["start_time"] = cont.start_time;

        std::string tmp;
        callAgent("/agent/report", "cleaning_record", j, tmp);
    }

}                                                                                             