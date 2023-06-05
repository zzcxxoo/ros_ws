#include "JtcxDifines.hpp"
#include "JtcxRobotCleaningState.hpp"
#include "JtcxLogWrapper.hpp"

// std
#include <fstream>

// ------------ static ---------------
static JtcxLogWrapper logger{"cleaning"};
const std::string CleaningTaskScheduler::_task_file = std::string(std::getenv("HOME")) + "./robot/config/cleaning/task.json";
const std::string RobotCleaningState::_save_context_file = std::string(std::getenv("HOME")) + "./robot/config/cleaning/context.json";


// ------------------ CleaningTaskScheduler ---------------------
auto& _ = CleaningTaskScheduler::getInstance();

CleaningTaskScheduler::CleaningTaskScheduler()
{
    loadTaskFromFile();
    _timer.reset(new std::thread(&CleaningTaskScheduler::timerHandler, this));    
}

bool CleaningTaskScheduler::pushBackTask(const Task& task)
{
    // only push valid task: time is after current timestamp
    double now = ros::WallTime::now().toSec();

    if(now - task.ts < _valid_time_interval){
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

void CleaningTaskScheduler::timerHandler(){
    while(true)
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
                double diff = now - tmp.ts;
                if(diff >= 1.0){
                    logger.logger->trace("the task is out of date!!");
                    _task_queue.pop();
                }
            }
            // get the first closest task
            if(!_task_queue.empty()){
                auto closest_task = _task_queue.top();
                double diff = now - closest_task.ts;
                if(diff >= 0 && diff < 1.0){
                    // 计划名是否存在可能也要判断??
                    if(closest_task.map_name == selected_map){
                        // just write here but not correct!!
                        logger.logger->trace("the task name {} is valid and should be executed now!", closest_task.plan_name);
                        JTCX::ExternalEventQueueManager::getEventManager().pushPenddingLowEvent(JTCX::LowLevelExternalEvents::L0001);
                    }else{
                        logger.logger->warn("the task name {} got wrong map name!", closest_task.map_name);
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
}

void CleaningTaskScheduler::saveTaskToFile(){
    std::lock_guard<std::mutex> lock(_task_queue_lock);
    std::vector<Task> vt;
    while(!_task_queue.empty()){
        auto tmp = _task_queue.top();
        vt.emplace_back(tmp);
        _task_queue.pop();
    }
    json j = vt;
    std::ofstream out(_task_file);
    out << j;
    out.close();
}

void CleaningTaskScheduler::loadTaskFromFile(){
    // only if file exist then load
    if(bfs::exists(bfs::path(_task_file))){
        std::ifstream inf(_task_file);
        json j = json::parse(inf);
        std::vector<Task> v = j.get<std::vector<Task>>();
        inf.close();
        for(auto const& e : v)  _task_queue.push(std::move(e));
    }
}

CleaningTaskScheduler::~CleaningTaskScheduler(){
    logger.logger->trace("save task to file!");
    saveTaskToFile();    
}

// ------------------ ExternalCleaningInterface ---------------------

void ExternalCleaningInterface::controlCleaningHandler(mobile_platform_msgs::Pursuit::Request& req,
    mobile_platform_msgs::Pursuit::Response& resp, std::promise<bool>& pro)
{
    std::string selected_map;
	ros::param::get("/selected_map", selected_map);

    if(selected_map.empty()){
        logger.logger->error("selected map does not exist!!");
        resp.message = "selected map does not exist!!";
        resp.status = -1;
        return;
    }

    req.mode = "path_tracking_with_path_name";
	
    logger.logger->trace("path tracking command: {}", req.command);
    logger.logger->trace("path tracking mode: {}", req.mode);
    logger.logger->trace("path tracking map: {}", req.map);
    logger.logger->trace("path tracking path_name: {}", req.path_name);
    logger.logger->trace("path tracking task_index: {}", req.task_index);

    // if start tracking, record info
    if(req.command == 1){
        _map_name = selected_map;
        _task_name = req.path_name;
        _times = req.task_index;
    }

	mobile_platform_msgs::Pursuit srv;
	srv.request = req;
	
	if(ros::service::call("/pursuit", srv))
	{
        if(srv.response.status == 0){
            logger.logger->info("path_tracking_client_call success!! ");
            pro.set_value(true);
        }else{
            // pursuit not success, set emergency
            logger.logger->error("path_tracking_client_call fail: {}", srv.response.message);
            pro.set_value(false);
        }
	}
	else
	{
        srv.response.status = -2;
		logger.logger->error("path_tracking_client_call defeat!! ");
        pro.set_value(false);
    }
}

// ------------------ RobotCleaningState ---------------------------

RobotCleaningState::RobotCleaningState()
{
    // init ros related
    _chassis_sub = _nh.subscribe("/chassis", 1, &RobotCleaningState::chassisHandler, this);
	_start_sub = _nh.subscribe("/pure_pursuit/tracking_path", 1, &RobotCleaningState::cleaningStartHandler, this);
	_status_sub = _nh.subscribe("/pure_pursuit/status", 1, &RobotCleaningState::cleaningStatusHandler, this);
	_result_sub = _nh.subscribe("/pure_pursuit/tracking_result", 1, &RobotCleaningState::cleaningResultHandler, this);
	_twist_sub = _nh.subscribe("/nav/cmd_vel", 1, &RobotCleaningState::twistHandler, this);
	_cleaning_logger_client = _nh.serviceClient<mobile_platform_msgs::MappingSave>("/ui/logger/cleaning_logger");

    loadContextFromFile();
    _check_health_state_thread_ptr.reset(new std::thread(&RobotCleaningState::checkHealthState, this));
    _loc_helper.reset(new JtcxLocalizationHelper(&_nh));
}

RobotCleaningState::~RobotCleaningState(){
    _start_check_health_flag = false;
    if(_check_health_state_thread_ptr->joinable())  _check_health_state_thread_ptr->join();
    if(_save_context_flag)  saveContextToFile();
}

void RobotCleaningState::saveContextToFile()
{
    std::ofstream out(_save_context_file);
    if(!out.is_open()){
        auto msg = fmt::format("{} cant open!!", _save_context_file);
        logger.logger->error(msg);
        throw std::runtime_error(msg);
    }
    json j = _context;
    out << j;
    out.close();
}

void RobotCleaningState::loadContextFromFile()
{
    // only if file exist then load
    if(bfs::exists(bfs::path(_save_context_file))){
        std::ifstream inf(_save_context_file);
        json j = json::parse(inf);
        inf.close();
        _context = j.get<cleaningContext>();
        logger.logger->info("context is loaded with path name {}", _context.path);
    }
}


void RobotCleaningState::checkHealthState()
{
    // check state:
    // 1. localization node
    // 2. localization performance
    // 3. tracking controller node
    // 4. tracking controller lost

}

// 这个handler一般只要启动就会收到
void RobotCleaningState::chassisHandler(const mobile_platform_msgs::ChassisConstPtr &msg)
{
    _context.mode = _mode_list[msg->driving_mode];
    if (_context.mode == "VCU_EMERGENCY_STOP" || _context.mode == "IPC_EMERGENCY")
        _context.exception = "Error";
}


void RobotCleaningState::cleaningResultHandler(const mobile_platform_msgs::PurePursuitResultConstPtr& msg)
{
    // 清扫正常结束
    if (msg->tracking_result == "Success")
    {
        logger.logger->trace("cleaning success!");
        // 删除上下文
        _save_context_flag = false;
        if(bfs::exists(bfs::path(_save_context_file)))
            bfs::remove(bfs::path(_save_context_file));
        
        pushAbnormalState(JTCX::RobotAbnormalType::state_finish);
    }


}


