
// ros msg header
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

// mobile_platform_msgs header
#include "mobile_platform_msgs/Pursuit.h"
#include "mobile_platform_msgs/PurePursuitStatus.h"
#include "mobile_platform_msgs/PurePursuitResult.h"
#include "mobile_platform_msgs/Chassis.h"
#include "mobile_platform_msgs/MappingSave.h"
#include "mobile_platform_msgs/Agent.h"
using namespace mobile_platform_msgs;

// std
#include <vector>
#include <queue>
#include <ctime>
#include <future>

// io
#include <fstream>
#include <boost/filesystem.hpp>
#include "nlohmann/json.hpp"
#define bfs boost::filesystem
using json = nlohmann::json;

// log
#include "JtcxLogWrapper.hpp"
#include "spdlog/fmt/fmt.h"

static JtcxLogWrapper logger("CleaningTaskScheduler", LOG_LEVEL::DEBUG);
#define LG logger.logger
static const std::string _save_log_dir = "/var/log/jtcx/robot-manager";

class CleaningTaskScheduler
{
private:

    std::unordered_map<std::string, json> _task_map;
    std::vector<std::string> _dump_task_ids;

    ros::NodeHandle _nh;
    ros::ServiceServer _task_server;
    ros::ServiceServer _reset_tasks_server;

    std::mutex _task_lock;
    std::unique_ptr<std::thread> _timer;    ///< check when to execute cleaning task
    std::atomic_bool _timer_start{true};
    
    const std::string _task_file = std::string(std::getenv("HOME")) + "/.robot/data/cleaning/timing_task.json";

public:
    CleaningTaskScheduler()
    {
        LG->trace("constructing obj!!");
        // create dir
        auto task_file = bfs::path(_task_file);
        if(!bfs::exists(task_file.parent_path()))   bfs::create_directories(task_file.parent_path());  
    
        _task_server = _nh.advertiseService("/ui/cleaning/timing_task", &CleaningTaskScheduler::cleaningTaskHandler, this);
        _reset_tasks_server = _nh.advertiseService("/timing_task/reset", &CleaningTaskScheduler::resetTimingTask, this);
    
        loadTaskFromFile();
        _timer.reset(new std::thread(&CleaningTaskScheduler::timerHandler, this)); 
    }

    CleaningTaskScheduler(const CleaningTaskScheduler&) = delete;
    CleaningTaskScheduler& operator=(const CleaningTaskScheduler&) = delete;

    bool resetTimingTask(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
        std::lock_guard<std::mutex> _lg(_task_lock);
        _task_map.clear();
        // clear all task
        json j;
        std::ofstream out(_task_file);
        out << j;
        out.close();
        return true;
    }

    /**
     * @brief external event
     * 
     * @return true 
     * @return false 
     */
    bool cleaningTaskHandler(AgentRequest& req, AgentResponse& res)
    {
        std::lock_guard<std::mutex> _lg(_task_lock);

        res.trace_id = req.trace_id;
        json j = json::parse(req.data);
        LG->info("task push in: {}", j.dump());

        std::string task_id = j["task_id"];
        std::string type = j["type"];

        // if not create, task must exist
        if(type != "create" && _task_map.find(task_id) == _task_map.end()){
            LG->error("type is {}, but task id({}) is not exist!!");
            return true;
        }

        if(type == "delete"){
            _task_map.erase(task_id);
        }else if(type == "create"){
            // create set a default enable
            j["enable"] = true;     
            _task_map[task_id] = j;
        }else if(type == "modify"){
            j["enable"] = _task_map[task_id]["enable"];
            _task_map[task_id] = j;
        }
        else{
            _task_map[task_id]["enable"] = (type == "enable");
        }

        return true;
    }

    /**
     * @brief format: "xx:xx"
     * 
     * @param str 
     * @return std::pair<int, int> hour min
     */
    std::pair<int, int> fmtHourAndMin(std::string str){
        auto res = std::make_pair(-1, -1); 

        int pos = str.find(':');
        int sz = str.size();

        if(pos != str.npos){
            res.first = std::stoi(str.substr(0, pos));
            res.second = std::stoi(str.substr(pos+1, sz-pos-1));
        }

        return res;   
    }

    AgentRequest mkReq(const json& j){
        json r;
        r["map_id"] = j["map_id"];
        r["plan_id"] = j["plan_id"];
        r["mode"] = "new";
        json jj;

        jj["zone_id"] = j["zone_id"];
        jj["times"] = j["times"];
        jj["mode"] = j["mode"];
        jj["rubtype"] = j["rubtype"];

        r["zones"].push_back(jj);

        json res;
        res["type"] = "start";
        res["content"] = r;
        
        LG->trace("make req: {}", res.dump());
        AgentRequest req;
        req.data = res.dump();
        return req;
    }

    int toSecs(int h, int m, int s){
        return 3600 * h + 60 * m + s;
    }

    void exec(std::string task_id, const json& task)
    {
        auto now = std::time(nullptr);
        auto fmt_now = localtime(&now);
        auto st = task["start_time"].get<std::string>();
        LG->trace("get start time: {}", st);
        auto hour_and_min = fmtHourAndMin(st);
        if(hour_and_min.first < 0 || hour_and_min.second < 0){
            LG->warn("invalid task and will be dumped!1");
            _dump_task_ids.push_back(task_id);
            return;
        }
        
        auto task_secs = toSecs(hour_and_min.first, hour_and_min.second, 0);
        auto now_secs = toSecs(fmt_now->tm_hour, fmt_now->tm_min, fmt_now->tm_sec); 
        int diff_secs = now_secs - task_secs;
        LG->trace("task {} diff mins: {}", task_id, diff_secs);
        if(diff_secs > 59){
            LG->warn("task({}) is out of date, dump!!", task_id);
            _dump_task_ids.push_back(task_id);
        }else if(0 <= diff_secs && diff_secs <= 59 && task["enable"].get<bool>()){
            // execute
            Agent ag;
            ag.request = mkReq(task["plan"]);
            if(ros::service::call("/ui/cleaning/control", ag))
            {
                if(ag.response.code == 0){
                    LG->info("task({}) is successfully exec!!", task_id);
                    _dump_task_ids.push_back(task_id);
                }else{
                    LG->error("task({}) fail to exec for: {}!!", task_id, ag.response.msg);
                }
            }else{
                LG->error("cleaning task node is offline!!");
            }
        }
    }

    /**
     * @brief check time when it is time to clean 
     * 
     */
    void timerHandler(){
        while(_timer_start)
        {
            {
                std::lock_guard<std::mutex> lock(_task_lock);

                decltype(_task_map)::const_iterator it;

                for(it = _task_map.begin(); it != _task_map.end(); it++){
                    auto task_id = it->first;
                    auto task = it->second;
                    
                    // do once
                    if(task["repeat_dates"].empty()){
                        exec(task_id, task);
                        continue;
                    }

                    auto now = std::time(nullptr);
                    auto fmt_now = localtime(&now);
                    // done repeatedly
                    int week_day = (fmt_now->tm_wday == 0 ? 7 : fmt_now->tm_wday);
                    auto repeated_dates = task["repeat_dates"].get<std::vector<int>>();
                    
                    auto fid = std::find(repeated_dates.begin(), repeated_dates.end(), week_day);
                    if(fid != repeated_dates.end()){
                        exec(task_id, task);
                    }
                }
                // dump all ids
                for(const auto& e : _dump_task_ids){
                    if(_task_map.find(e) != _task_map.end()){
                        LG->info("dumping task {}", e);
                        _task_map.erase(e);
                    }
                }

                std::vector<std::string> tmp;
                _dump_task_ids.swap(tmp);
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        LG->trace("timer thread exit!");
    }

    void saveTaskToFile()
    {
        if(!_task_map.empty()){
            json j = _task_map;
            std::ofstream out(_task_file);
            out << j;
            out.close();
            LG->trace("save task to file done!!");
        }
    }

    void loadTaskFromFile()
    {
        // only if file exist then load
        if(!bfs::exists(bfs::path(_task_file)))
        {
            LG->error("task file is not exist!!");
            return;
        }
        
        std::ifstream inf(_task_file);
        json j = json::parse(inf);
        if(!j.is_null())    _task_map = j;
    }

    ~CleaningTaskScheduler(){
        _timer_start = false;
        LG->trace("exit obj!!");
        if(_timer->joinable())  _timer->join();
        saveTaskToFile(); 
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test-timing");
    logger.initLogFile(_save_log_dir, "cleaning_timing.log");

    CleaningTaskScheduler cts;
    ros::spin();

    return 0;
}
