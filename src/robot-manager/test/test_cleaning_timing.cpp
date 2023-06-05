
// ros msg header
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

// mobile_platform_msgs header
#include "mobile_platform_msgs/Pursuit.h"
#include "mobile_platform_msgs/PurePursuitStatus.h"
#include "mobile_platform_msgs/PurePursuitResult.h"
#include "mobile_platform_msgs/Chassis.h"
#include "mobile_platform_msgs/MappingSave.h"

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

class CleaningTaskScheduler
{
private:

    struct Task{
        double ts;  ///< time stamp
        std::string map_name;
        std::string plan_name;
        int times;  

        bool operator<(const Task& r) const{
            return ts > r.ts;
        }
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Task, ts, map_name, plan_name, times);
    };

    ros::NodeHandle _nh;
    ros::ServiceServer _task_server;

    const float _valid_time_interval = 2.0; ///< when set time, it should be 5s later than now

    std::mutex _task_queue_lock;
    std::priority_queue<Task> _task_queue;  ///< queue containing task in lastest
    std::unique_ptr<std::thread> _timer;    ///< check when to execute cleaning task

    std::atomic_bool _timer_start{true};
    
public:
    CleaningTaskScheduler()
    {
        logger.logger->trace("constructing obj!!");
        _task_server = _nh.advertiseService("/cleaning/timing_task", &CleaningTaskScheduler::pushBackTask, this);
    
        // create dir
        auto task_file = bfs::path(_task_file);
        if(!bfs::exists(task_file.parent_path()))   bfs::create_directories(task_file.parent_path());  
    
        loadTaskFromFile();
        _timer.reset(new std::thread(&CleaningTaskScheduler::timerHandler, this)); 
    }

    CleaningTaskScheduler(const CleaningTaskScheduler&) = delete;
    CleaningTaskScheduler& operator=(const CleaningTaskScheduler&) = delete;

    const std::string _task_file = std::string(std::getenv("HOME")) + "/.robot/config/cleaning/task.json";

    /**
     * @brief external event
     * 
     * @return true 
     * @return false 
     */
    bool pushBackTask(mobile_platform_msgs::MappingSaveRequest& req, mobile_platform_msgs::MappingSaveResponse& res)
    {
        // only push valid task: time is after current timestamp
        json j = json::parse(req.filename);
        logger.logger->info("task push in: {}", j.dump());
        Task task = j;

        double now = ros::WallTime::now().toSec();

        if(task.ts - now < _valid_time_interval){
            logger.logger->error("time set not valid!!");
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(_task_queue_lock);
            _task_queue.push(task);    
        }

        logger.logger->trace("push task success!!");
        return true;
    }

    /**
     * @brief check time when it is time to clean 
     * 
     */
    void timerHandler(){
        while(_timer_start)
        {
            std::string selected_map;
            if(!ros::param::get("/selected_map", selected_map)){
                logger.logger->error("selected map does not exist!!");
                return;
            }

            double now = ros::WallTime::now().toSec();
            {
                std::lock_guard<std::mutex> lock(_task_queue_lock);
                // firstly pop all out of date task
                while (!_task_queue.empty())
                {
                    auto tmp = _task_queue.top();
                    // ts + 1 >= now >= ts
                    double diff = tmp.ts - now;
                    if(diff < 0){
                        logger.logger->info("the task is out of date!!");
                        _task_queue.pop();
                        continue;
                    }
                    break;
                }
                // get the first closest task
                if(!_task_queue.empty()){
                    auto closest_task = _task_queue.top();
                    double diff = closest_task.ts - now;
                    logger.logger->trace("task diff {}", diff);
                    if(diff >= 0 && diff < 1.0){
                        // 计划名是否存在可能也要判断??
                        if(closest_task.map_name == selected_map){
                            // just write here but not correct!!
                            logger.logger->trace("the task name {} is valid and should be executed now!", closest_task.plan_name);
                            mobile_platform_msgs::Pursuit srv;
                            srv.request.map = closest_task.map_name;
                            srv.request.path_name = closest_task.plan_name;
                            srv.request.task_index = closest_task.times;
                            ros::service::call("/ui/cleaning_task", srv);
                            _task_queue.pop();

                            if(srv.response.status == 0)    logger.logger->info("task execute success!!");
                            else    logger.logger->info("task execute fail for {}", srv.response.message);

                        }else{
                            logger.logger->warn("the task name {} got wrong map name!", closest_task.map_name);
                        }
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        logger.logger->trace("timer thread exit!");
    }

    void saveTaskToFile()
    {
        std::lock_guard<std::mutex> lock(_task_queue_lock);
        std::vector<Task> vt;

        double now = ros::WallTime::now().toSec();
        // 自动pop掉过时的任务
        while(!_task_queue.empty()){
            auto tmp = _task_queue.top();
            if(tmp.ts - now > _valid_time_interval) vt.emplace_back(tmp);
            _task_queue.pop();
        }
        json j = vt;
        std::ofstream out(_task_file);
        out << j;
        out.close();
        logger.logger->trace("save task to file done!!");
    }

    void loadTaskFromFile()
    {
        // only if file exist then load
        if(bfs::exists(bfs::path(_task_file))){
            std::ifstream inf(_task_file);
            json j = json::parse(inf);
            std::vector<Task> v = j.get<std::vector<Task>>();
            inf.close();
            for(auto const& e : v)  _task_queue.push(std::move(e));

            return;
        }

        logger.logger->trace("task file is not exist!!");
    }

    ~CleaningTaskScheduler(){
        _timer_start = false;
        logger.logger->trace("exit obj!!");
        _timer->join();
        saveTaskToFile(); 
    }
};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test-timing");

    CleaningTaskScheduler cts;
    ros::spin();

    return 0;
}
