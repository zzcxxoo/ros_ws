/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxStructDefinitions.hpp
Version       : 固件v1.1.1rc
Author        : JianQiang 
Created       : 2022/11/26
Last Modified :
Description   : Necessary struct definitions
Function List :
History       : first edition ----20221125
******************************************************************************/
#pragma once
#include "JtcxMacroDefinition.hpp"
#include "nlohmann/json.hpp"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include "mobile_platform_msgs/Pursuit.h"

using namespace mobile_platform_msgs;
using json = nlohmann::json;

namespace JTCX
{

    // DEFINE_ENUM_WITH_TOSTRING(CleaningException, (Normal)(New_Task)(Unknown_Error))

    struct CleaningException
    {
        const std::string NORMAL = "Normal";
        const std::string LOW_POWER = "Low_Power";        
        const std::string RECALL = "Recall";
        const std::string TIMING_TASK = "Timing_Task";

        const std::string NEW = "New_Task";
        const std::string UNKNOWN = "Unknown_Error";

        const std::string INTERRUPT = "Manual_Stop";
        const std::string SUCCESS = "Success";
        const std::string FAIL = "Fail";
    };
    
    struct CleaningMode
    {

        const std::string EDGE = "egde";
        const std::string COVERAGE = "coverage";
        const std::string BOTH = "both";
        const std::string JOIN = "join";
        const std::string FROM_CHARGING = "from_charging";
        const std::string TO_CHARGING = "to_charging";


        const std::string RECALL = "recall";
        const std::string CLEANING = "cleaning";
        const std::string TIMING = "timing";
        const std::string RESUME = "resume";

    };
    
    // enum CleaningException
    // {
    //     COMPLETE_SWEEPING         =0,
    //     ///-------------- pursuit error -----------------
    //     MANUAL_INTERRUPT          =2,
    //     PURSUIT_FAIL              =3,
    //     CENTER_CONTROL_INTERRUPT  =4,
    //     // --------------- unusally reasons
    //     NODE_EXIT                 =5,
    //     PURE_PURSUIT_EXIT         =6,
    //     UNKNOWN_ERROR             =7,
    // };

    /// @brief 最小清洁计划原子动作
    struct CleaningTaskInfo 
    {
        int ID;
        //沿边、覆盖模式或者兼有
        std::string cleaning_mode;
        //挡板开合的大小
        std::string rub_type;
        //此粒度的task对应的path点的数量
        int path_size;
        //路径文件位置
        std::string path_file_location;
        //所属地图名字
        std::string map_id;
        //所属分区名字
        std::string zone_id;
        //所属分区文件位置
        std::string zone_dir;
        // 下面格式需要和txt文件一样 x y x y z w
        //起点
        std::string start;
        //终点
        std::string end;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(CleaningTaskInfo, ID, cleaning_mode,rub_type,path_size,path_file_location,map_id, zone_id, zone_dir, start, end);
    };
    
    // 用于构建任务，解析的命令存到这里
    struct CleaningAppConfInfo 
    {
        // 地图名
        std::string map_id; 
        //清洁计划名称   
        std::string plan_id;
        // 原始清洁任务信息，来自前端配置
        // time次数已默认重复构造在vector中
        // std::vector<CleaningTaskInfo> tasks; task.dump();
        std::string tasks;
        //新建或断点续扫
        bool continuous_mode;
        //定时清扫专用 
        std::string task_id;
        //开始时间
        std::string start_time;
        //重复日期
        std::vector<int> repeat_dates;
        //使能
        bool enable;

        NLOHMANN_DEFINE_TYPE_INTRUSIVE(CleaningAppConfInfo, map_id, plan_id, tasks,continuous_mode,task_id,start_time, repeat_dates, enable);
    };

    // 上下文，每次清洁会更新
    struct CleaningContextInfo 
    {
        // 地图id
        std::string map_id;
        //计划名称(默认字符不能去除)
        std::string plan_id;
        // 清洁模式 cleaningmode
        std::string cleaning_mode;
        //进度
        double process;
        //已清扫区域面积
        double area;
        //已行驶里程数
        double odometry;
        //清洁时间
        double cleaning_time;
        //开始时间
        double start_time;
        //结束时间
        double stop_time;
        //平均速度
        double average_velocity;
        //断点续扫专用
        int completed_task_id;
        //断点原因
        std::string exception; //null/low_power/stop_cmd/interrupt/ERROR*//
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(CleaningContextInfo, cleaning_mode, map_id, plan_id,process,area,odometry,cleaning_time,start_time,stop_time, average_velocity, completed_task_id, exception);
    };
}
// namespace nlohmann {
// 	template <>
// 	struct adl_serializer<JTCX::CleaningTaskInfo> {
// 		static JTCX::CleaningTaskInfo from_json(const json& j)
// 		{
//             JTCX::CleaningTaskInfo temp;
//             temp.ID                 =j["ID"].get<int>();
//             temp.cleaning_mode      =j["cleaning_mode"].get<std::string>();
//             temp.rub_type           =j["rub_type"].get<std::string>();
//             temp.path_size          =j["path_size"].get<int>();
//             temp.path_file_location =j["path_file_location"].get<std::string>();
//             temp.map_name           =j["map_name"].get<std::string>();
//             temp.zone_name          =j["zone_name"].get<std::string>();
//             temp.zone_dir           =j["zone_dir"].get<std::string>();
// 			return temp;
// 		}
// 		static void to_json(json& j, const JTCX::CleaningTaskInfo &p) {
//             j = json{{"ID", p.ID}, {"cleaning_mode", p.cleaning_mode}, {"rub_type", p.rub_type},{"path_size",p.path_size},
//             {"path_file_location", p.path_file_location},{"map_name", p.map_name},{"zone_name", p.zone_name},
//             {"zone_dir",p.zone_dir}};
// 		}
// 	};
//     template <>
// 	struct adl_serializer<JTCX::CleaningContextInfo> {
// 		static JTCX::CleaningContextInfo from_json(const json& j)
// 		{
//             JTCX::CleaningContextInfo temp;
//             temp.plan_name        =j["plan_name"].get<std::string>();;
//             temp.process          =j["process"].get<double>();
//             temp.area             =j["area"].get<double>();
//             temp.odometry         =j["odometry"].get<double>();
//             temp.cleaning_time    =j["cleaning_time"].get<std::string>();
//             temp.start_time       =j["start_time"].get<std::string>();
//             temp.stop_time        =j["stop_time"].get<std::string>();
//             temp.average_velocity =j["average_velocity"].get<double>();
//             temp.completed_task_id=j["completed_task_id"].get<int>();
//             temp.exception        =j["exception"].get<std::string>();
// 			return temp;
// 		}
// 		static void to_json(json& j, const JTCX::CleaningContextInfo &p) {
//             j = json{{"plan_name", p.plan_name}, {"process", p.process}, {"area", p.area},{"odometry",p.odometry},
//             {"cleaning_time", p.cleaning_time},{"start_time", p.start_time},{"stop_time", p.stop_time},{"average_velocity", p.average_velocity},
//             {"completed_task_id", p.completed_task_id},{"exception",p.exception}};
// 		}
// 	};
// }