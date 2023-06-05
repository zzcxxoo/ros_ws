// ros msg header
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

// mobile_platform_msgs header
#include "mobile_platform_msgs/LocalizationLost.h"
#include "mobile_platform_msgs/Pursuit.h"
#include "mobile_platform_msgs/PurePursuitStatus.h"
#include "mobile_platform_msgs/PurePursuitResult.h"
#include "mobile_platform_msgs/Chassis.h"
#include "mobile_platform_msgs/MappingSave.h"
#include "mobile_platform_msgs/TargetPoint.h"
#include "mobile_platform_msgs/HomeToDock.h"
#include "mobile_platform_msgs/Agent.h"
#include "mobile_platform_msgs/BcmCommand.h"

#include "JtcxCleaningDefinitions.hpp"
#include "JtcxMacroDefinition.hpp"
#include "JtcxErrorCodes.hpp"
#include "JtcxCleaningTaskHandler.hpp"
#include "JtcxAppHandler.hpp"
#include "JtcxExceptionHandler.hpp"
#include "JtcxFileHandler.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxPathHandler.hpp"
#include "JtcxSecurityQueue.hpp"
#include "JtcxUtilTools.hpp"
#include "stdio.h"

using namespace JTCX;
using namespace mobile_platform_msgs;
using namespace std;

namespace JTCX{

    // 一些单例都要在这里先创建实例
    auto _0 = JtcxSingleton<JtcxLogger>::CreateInstance("cleaning", "cleaning.log");
    auto _1 = JtcxSingleton<JtcxExceptionHandler>::CreateInstance();
    auto _2 = JtcxSingleton<CleaningTaskHandler>::CreateInstance();
    auto _3 = JtcxSingleton<CleaningPathHandler>::CreateInstance();
    auto _4 = JtcxSingleton<CleaningException>::CreateInstance();
    auto _5 = JtcxSingleton<CleaningMode>::CreateInstance();
    auto _6 = JtcxSingleton<CleaningAppHandler>::CreateInstance();

    class RobotCleaningState
    {
    private:
        // ros related
        ros::NodeHandle _nh;
        ros::Publisher _bcm_pub;

        ros::ServiceServer _rcs_srv;
        ros::ServiceServer _timing_srv;
        ros::ServiceServer _reset_tasks_server;
        ros::Subscriber _status_sub;
        ros::Subscriber _result_sub;
        ros::Subscriber _twist_sub;
        ros::Subscriber _chassis_battery_sub;

        // ------------cleaning context -------------
        CleaningContextInfo _context;
        std::vector<CleaningTaskInfo> _total_tasks;

        // ------------ need to be reset ------------
        std::atomic<int>_current_task_id{-1};
        double _total_odometry=0;
        int _way_point =0;
        std::deque<float> _velocity;

        //--------------common tools---------------
        std::thread* _cleaning_thread_ptr =nullptr;
        JtcxLogger* _logger = nullptr;
        CleaningTaskHandler * _task_handler =nullptr;
        CleaningPathHandler * _path_handler =nullptr;
        CleaningAppHandler * _app_handler =nullptr;

        //--------------timing cleaning--------------
        std::mutex _task_lock;
        std::unique_ptr<std::thread> _timer; 
        std::vector<CleaningAppConfInfo> _timing_tasks;
        std::atomic_bool _running_timer{true};

        //--------------battery------------
        std::atomic<bool> _battery_supporter{true};
        int _battery_threshold{0};

    public:

        RobotCleaningState(){
            _logger = JtcxSingleton<JtcxLogger>::GetInstance();
            _task_handler =JtcxSingleton<CleaningTaskHandler>::GetInstance();
            _path_handler =JtcxSingleton<CleaningPathHandler>::GetInstance();
            _app_handler  =JtcxSingleton<CleaningAppHandler>::GetInstance();

            _logger->INFO("Construct Cleaning State!!");
            // init ros related
            _rcs_srv = _nh.advertiseService("/ui/cleaning/control", &RobotCleaningState::cleaningHandler, this);
            _timing_srv = _nh.advertiseService("/ui/cleaning/timing_task", &RobotCleaningState::cleaningTimingHandler, this);
            _reset_tasks_server = _nh.advertiseService("/timing_task/reset", &RobotCleaningState::resetTimingTask, this);          

            _bcm_pub = _nh.advertise<BcmCommand>("/auto/clean_cmd", 5);
            _chassis_battery_sub =_nh.subscribe("/chassis", 1, &RobotCleaningState::checkBattery, this);
            _status_sub = _nh.subscribe("/pure_pursuit/status", 1, &RobotCleaningState::cleaningStatusHandler, this);
            _result_sub = _nh.subscribe("/pure_pursuit/tracking_result", 10, &RobotCleaningState::cleaningResultHandler, this, ros::TransportHints().tcpNoDelay());
            _twist_sub = _nh.subscribe("/nav/cmd_vel", 1, &RobotCleaningState::twistHandler, this);

            _timer.reset(new std::thread(&RobotCleaningState::cleaningTimerThread, this));

            // signal(SIGINT, &RobotCleaningState::stopTrackingAndExit);
            // signal(SIGTERM, &RobotCleaningState::stopTrackingAndExit);
            // create some necessary files
            if(!JtcxDir::existDir(CLEANING_PATH))                     system("mkdir -p " CLEANING_PATH);
            if(!JtcxDir::existDir(CLEANING_CONTEXT_JSON_PATH))        system("touch " CLEANING_CONTEXT_JSON_PATH);
            if(!JtcxDir::existDir(CLEANING_TASKS_JSON_PATH))          system("touch " CLEANING_TASKS_JSON_PATH);
            if(!JtcxDir::existDir(CLEANING_TIMING_CONTEXT_JSON_PATH)) system("touch " CLEANING_TIMING_CONTEXT_JSON_PATH);
            //初始化 timing_tasks
            _task_handler->getConfsFromFile(CLEANING_TIMING_CONTEXT_JSON_PATH,_timing_tasks);
        }
        bool resetTimingTask(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
            std::lock_guard<std::mutex> _lg(_task_lock);
            _timing_tasks.clear();
            // clear all task
            json j;
            std::ofstream out(CLEANING_TIMING_CONTEXT_JSON_PATH);
            out << j;
            out.close();
            return true;
        }
        ~RobotCleaningState(){
            _task_handler->saveConfs2File(CLEANING_TIMING_CONTEXT_JSON_PATH,_timing_tasks);
            if(_cleaning_thread_ptr!=nullptr)
            {  
                _cleaning_thread_ptr->join();
                delete _cleaning_thread_ptr;
                _cleaning_thread_ptr =nullptr;
            }
            _running_timer = false;
            if(_timer->joinable())  _timer->join();
        }
        void resetCleaningThread(){
            //关掉现有执行线程；
            if(_cleaning_thread_ptr!=nullptr) {
                _current_task_id.store(-1);
                if(_cleaning_thread_ptr->joinable())    _cleaning_thread_ptr->join();
                delete _cleaning_thread_ptr;
                _cleaning_thread_ptr = nullptr;
            }
        }
        void resetCleaningParams(){
            _total_odometry=0;
            _velocity.clear();
        }
        bool isWorkingNow(){return (_current_task_id.load() >= 0);}
        // 清洁控制回调
        bool cleaningHandler(AgentRequest& req, AgentResponse& res)
        {
            // 0. parse req
            res.trace_id = req.trace_id;
            json j = json::parse(req.data);
            _logger->TRACE("get req: {}", req.data);

            std::string type = j["type"];
            std::string data = j["content"].dump();
            //低电量只执行召回
            if(!_battery_supporter.load() && type != "recall")
            {  
                res.msg = "low battery!";res.code = -1;
                return true;
            }

            if(type == "pause" || type == "resume"){
                callPursuit(type == "pause" ? 3 : 4);
                res.msg = "ok";res.code = 0;
                return true;
            }
            else if(type == "stop"){
                callPursuit(2);
                res.msg = "ok";res.code = 0;
                return true;
            }
            if(type == "recall"){
                // pause tracking, result callback wont be dealt
                callPursuit(3);
                // if working, then update the context already written
                // else, the new context will be create at constructRecallFromTask
                if(isWorkingNow())
                    _task_handler->updateContext2File(_context, JtcxSingleton<JTCX::CleaningException>::GetInstance()->RECALL);
            
                resetCleaningThread();
                if(!constructRecallFromTask()){
                    res.msg = "can not recall";res.code = -1;
                    return true;
                }
            }
            else{ 
                if(isWorkingNow()){
                    callPursuit(3);
                    _task_handler->updateContext2File(_context, JtcxSingleton<JTCX::CleaningException>::GetInstance()->NEW);
                    resetCleaningThread();
                    // parse req to conf
                    CleaningAppConfInfo conf;
                    _task_handler->parseCleaningConfigFromAPP(data, conf);
                    if(!constructTasksFromCleaning(conf)){
                        res.msg = "can not init cleaning context!";res.code = -1;
                        return true;
                    }
                }
                //idle 构建
                else{
                    CleaningAppConfInfo conf;
                    _task_handler->parseCleaningConfigFromAPP(data, conf);
                    if(!constructTasksFromCharging(conf)){
                        res.msg = "can not init cleaning context!";res.code = -1;
                        return true;
                    } 
                }
                _context.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->CLEANING;
                _task_handler->saveContext2File(_context, JtcxSingleton<CleaningException>::GetInstance()->NORMAL);
            }           
            //激活线程
            _current_task_id.store(0);
            _cleaning_thread_ptr = new std::thread(&RobotCleaningState::carryOutTasks, this);
            return true;
        }
        // 相应app下达的定时任务，解析字段
        bool cleaningTimingHandler(AgentRequest& req, AgentResponse& res)
        {
            std::lock_guard<std::mutex> lg(_task_lock);
            res.trace_id = req.trace_id;
            if(!_battery_supporter.load())
            {  
                res.msg = "low battery!";res.code = -1;
                return true;
            }

            json j = json::parse(req.data);
            _logger->INFO("task push in: {}", j.dump());
            std::string type = j["type"];

            // create 
            if(type == "create"){
                CleaningAppConfInfo conf;
                _task_handler->parseCleaningConfigFromAPPTiming(j, conf);
                _timing_tasks.emplace_back(std::move(conf));
                return true;
            }
            // find
            auto task_id = j["task_id"].get<std::string>();
            auto itr = find_if(_timing_tasks.begin(),_timing_tasks.end(),[=](CleaningAppConfInfo temp){
                            return temp.task_id == task_id;});

            if(itr == _timing_tasks.end()){
                _logger->ERROR("type is {}, but task id({}) is not exist!!");
                return true;
            }
            //modify
            if(type == "modify"){
                CleaningAppConfInfo conf;
                _task_handler->parseCleaningConfigFromAPPTiming(j, conf);
                _timing_tasks[itr-_timing_tasks.begin()] = conf;
                return true;
            }
            //delete
            if(type == "delete")  _timing_tasks.erase(itr);
            else{
                _timing_tasks[itr-_timing_tasks.begin()].enable = (type == "enable");
            }
            return true;
        }
        // 定时任务线程
        void cleaningTimerThread(){
            while(_running_timer)
            {
                {
                    if(!_battery_supporter.load())
                    {  
                        LOG_WITH_GAP(_logger, WARN, 5.0, "low battery!");
                        continue;
                    }
                    std::lock_guard<std::mutex> lock(_task_lock);
                    decltype(_timing_tasks)::const_iterator it = _timing_tasks.begin();
                    while(it!=_timing_tasks.end()){
                        if(it->enable){
                            //解析配置事件，剔除不合理定时任务
                            string start_time=it->start_time;
                            // _logger->TRACE("get start time: {}", start_time);
                            int hours, mins;
                            std::tie(hours, mins) = JTCX::util::fmtHourAndMin(start_time);
                            if(hours < 0 || mins < 0){
                                _logger->WARN("invalid task {}",it->task_id);
                                _timing_tasks.erase(it);continue;
                            }
                            //解析当前时间
                            auto now = std::time(nullptr);
                            auto fmt_now = localtime(&now);
                            auto task_secs = JTCX::util::convertHourMinSecsToSecs(hours, mins, 0);
                            auto now_secs = JTCX::util::convertHourMinSecsToSecs(fmt_now->tm_hour, fmt_now->tm_min, fmt_now->tm_sec); 
                            int diff_secs = now_secs - task_secs;
                            // _logger->TRACE("task {} diff mins: {}", it->task_id, diff_secs);
                            //超时 阈值5s
                            if(diff_secs > 5){
                                _logger->WARN("task({}) is out of date,", it->task_id);
                                // 且未设置重复日期，将被剔除
                                if(it->repeat_dates.empty()){
                                    _timing_tasks.erase(it);
                                    continue;
                                }
                            }   //符合时间条件，进一步检查日期条件
                            else if(0 <= diff_secs && diff_secs <= 5){
                                int week_day = (fmt_now->tm_wday == 0 ? 7 : fmt_now->tm_wday);
                                auto fid = std::find(it->repeat_dates.begin(), it->repeat_dates.end(), week_day);
                                //当前符合重复日期或仅执行一次，将被执行任务
                                if(it->repeat_dates.empty() || fid !=it->repeat_dates.end()){
                                    //如果现在有任务在执行，保存当下清洁记录，并执行新的清洁计划需求；
                                    if(isWorkingNow()){
                                        callPursuit(3);
                                        _task_handler->updateContext2File(_context, JtcxSingleton<JTCX::CleaningException>::GetInstance()->TIMING_TASK);
                                        //重置清洁子线程
                                        resetCleaningThread();
                                        // 构建新任务
                                        if(!constructTasksFromCleaning(*it)){
                                            _logger->ERROR("timing task({}) construct fail!!", it->task_id);
                                            it++;
                                            continue;
                                        }
                                    }
                                    //idle 构建
                                    else{
                                        if(!constructTasksFromCharging(*it)){
                                            _logger->ERROR("timing task({}) construct fail!!", it->task_id);
                                            it++;
                                            continue;
                                        }
                                    }

                                    _context.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->TIMING;
                                    _task_handler->saveContext2File(_context, JtcxSingleton<CleaningException>::GetInstance()->NORMAL);
                                    //激活线程
                                    _current_task_id.store(0);
                                    _cleaning_thread_ptr = new std::thread(&RobotCleaningState::carryOutTasks, this);
                                }
                            }
                        }
                        it++;
                    }
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            _logger->TRACE("timer thread exit!");
        }
        // 检查电量回调
        void checkBattery(const mobile_platform_msgs::ChassisConstPtr &msg) {
            int battery = msg->bms.battery_soc_percentage;
            ros::param::get("/param/battery_threshold", _battery_threshold);
            // _logger->TRACE("battery_threshold: {}", _battery_threshold);
            //局部静态，避免覆盖
            static CleaningContextInfo s_context;
            static std::vector<CleaningTaskInfo> s_total_tasks;
            //低电量后，携带_battery_supporter阈值的功能均被关闭
            if (_battery_supporter.load() && battery < _battery_threshold){
                _battery_supporter.store(false);
                // 构建好返航路线和清洁计划
                s_total_tasks.clear();
                constructBreakPointResumingTasks(s_context,s_total_tasks);
                //3、召回
                json res;
                res["type"] = "recall";
                res["content"] = json::object();
                Agent ag;
                ag.request.data = res.dump();
                cleaningHandler(ag.request, ag.response);
                if(ag.response.code == 0){
                    _logger->INFO("recall is successfully exec!!");
                }else{
                    _logger->ERROR("fail to recall for: {}!!",ag.response.msg);
                }
                return;
            }
            //充满电后，携带_battery_supporter阈值的功能均开启，且首先执行回到断点继续执行任务的清洁计划
            if(!_battery_supporter.load() && battery >= 40){
                _battery_supporter.store(true);
                //导入已构建清洁计划
                if(s_total_tasks.size()>0){
                    resetCleaningThread();
                    _context = s_context;
                    _total_tasks = s_total_tasks;
                    _context.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->RESUME;
                    _task_handler->saveContext2File(_context, JtcxSingleton<CleaningException>::GetInstance()->NORMAL);
                    _task_handler->saveTasks2File(CLEANING_TASKS_JSON_PATH, _total_tasks);
                    //激活线程
                    _current_task_id.store(0);
                    _cleaning_thread_ptr = new std::thread(&RobotCleaningState::carryOutTasks, this);
                }
            }
        }
        // 构建召回任务
        bool constructRecallFromTask(){
            std::string to_charging_path, cur_map_name;
            if(!JTCX::util::getCurrentMapName(cur_map_name)){
                _logger->ERROR("selected map cant be got!!");
                return false;
            }
            // 这个recall.csv命名非常重要，关系到能否手机显示召回
            to_charging_path = fmt::format("{}/{}/path/recall.csv", ROBOT_MAP_DIR, cur_map_name);
            _logger->TRACE("to_charging_path: {}", to_charging_path);

            CleaningTaskInfo pre_start_task, to_charging_task;
            if(!_path_handler->getCurrentPose(pre_start_task.end)){
                _logger->ERROR("can not get current pose!!");
                return false;
            }

            if(_path_handler->createToChargingPathAndSave(pre_start_task,to_charging_path)){}
            else{
                _logger->ERROR("can not create {} path",to_charging_path);
                return false;
            }
            if(_task_handler->genTaskFromJoinPath(to_charging_path,to_charging_task)){}
            else{ 
                _logger->ERROR("to charging task construct failed!");
                return false;
            }
            //重置 _total_tasks；
            _total_tasks.clear();
            to_charging_task.ID=0;
            _total_tasks.emplace_back(to_charging_task);

            //配置清洁报告里的相关信息
            _context.completed_task_id  = -1;
            _context.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->RECALL;
            _task_handler->saveContext2File(_context, JtcxSingleton<CleaningException>::GetInstance()->NORMAL);
            _task_handler->saveTasks2File(CLEANING_TASKS_JSON_PATH,_total_tasks);
            return true;
        }
        bool constructTasksFromCleaning(const CleaningAppConfInfo& conf)
        {
            //解析上一段清洁计划中断前信息。
            int current_task_id = _context.completed_task_id +1;
            CleaningTaskInfo current_task = _total_tasks[current_task_id];
            // prepare pre start task
            CleaningTaskInfo pre_start_task = current_task;
            // _task_handler->genClipTask(current_task,_way_point,pre_start_task);
            std::string clip_path_location =JtcxDir::getFatherDir(current_task.path_file_location)+"/"+"cur.csv";
			pre_start_task.ID                   = 0;
			pre_start_task.cleaning_mode        = current_task.cleaning_mode;
			pre_start_task.rub_type             = current_task.rub_type;
			pre_start_task.path_file_location   = clip_path_location;
            if(!_path_handler->getCurrentPose(pre_start_task.end)){
                _logger->ERROR("can't get current pose!!");
                return false;
            }

            //重置 _total_tasks；
            _total_tasks.clear();

            //解析app配置信息
            _task_handler->initCurrentContext(conf, _context);
            if(conf.continuous_mode)
            {   
                //已清扫里程除以已清扫进度
                _total_odometry = _context.odometry*1.0/_context.process;
                _task_handler->getTasksFromFile(CLEANING_TASKS_JSON_PATH,_total_tasks);
            }
            else
            {
                //获取app设置的主要区域的tasks list
                std::vector<CleaningTaskInfo> app_config_tasks;
                _task_handler->genTasksFromAPPconfig(conf, app_config_tasks);
                
                //1、generate start task
                std::vector<CleaningTaskInfo> task_pair=constructTaskPair(pre_start_task, app_config_tasks[0]);
                if(task_pair.size()>1) _total_tasks.emplace_back(task_pair[1]);

                //2、生成拼接路径与task。
                for(int i = 0 ;i< app_config_tasks.size()-1; i++)
                {
                    auto from = app_config_tasks[i];
                    auto to = app_config_tasks[i+1];
                    std::vector<CleaningTaskInfo> task_pair=constructTaskPair(from,to);
                    for(const auto& e : task_pair)  _total_tasks.emplace_back(e);
                }
                _total_tasks.emplace_back(app_config_tasks.back());

                //3、返航路径task
                std::string to_charging_path = app_config_tasks.back().zone_dir + "/"+"to_charging.csv";
                _logger->TRACE("to_charging_path: {}", to_charging_path);

                CleaningTaskInfo to_charging_task;
                if(_path_handler->createToChargingPathAndSave(app_config_tasks.back(),to_charging_path)){}
                else{
                    _logger->ERROR("can not create {} path",to_charging_path);
                    return false;
                }
                if(_task_handler->genTaskFromJoinPath(to_charging_path,to_charging_task)){}
                else{ 
                    _logger->ERROR("to charging task construct failed!");
                    return false;
                }
                _total_tasks.emplace_back(to_charging_task);

                //重置排序ID
                int index=0;
                for(auto &task:_total_tasks)
                {
                    _total_odometry +=task.path_size;
                    task.ID=index;
                    index+=1;   
                }
                //配置清洁报告里的相关信息
                _context.completed_task_id  = -1;
                _task_handler->saveTasks2File(CLEANING_TASKS_JSON_PATH,_total_tasks);
            }
            return true;
        }
        bool constructTasksFromCharging(const CleaningAppConfInfo& conf)
        {
            //重置task vector
            _total_tasks.clear();
            _logger->TRACE("construct cleaning tasks from charging");

            /// @brief 是否设置了断点续扫
            /// @brief 如果设置了断点，清扫日志将载入历史最新一版本记录。否则重新初始化
            _task_handler->initCurrentContext(conf, _context);

            if(conf.continuous_mode)
            {   
                //已清扫里程除以已清扫进度
                _total_odometry = _context.odometry*1.0/_context.process;
                _task_handler->getTasksFromFile(CLEANING_TASKS_JSON_PATH,_total_tasks);
            }
            else
            {
                //获取app设置的主要区域的tasks list
                std::vector<CleaningTaskInfo> app_config_tasks;
                _task_handler->genTasksFromAPPconfig(conf, app_config_tasks);

                //出航task路径名字
                std::string from_charging_path = app_config_tasks.front().zone_dir + "/"+"from_charging.csv";
                _logger->TRACE("from_charging_path: {}", from_charging_path);

                //返航task路径名字
                std::string to_charging_path = app_config_tasks.back().zone_dir + "/"+"to_charging.csv";
                _logger->TRACE("to_charging_path: {}", to_charging_path);

                //1、from charging task
                CleaningTaskInfo from_charging_task;
                if(_task_handler->genTaskFromCharging(from_charging_path,from_charging_task)){} 
                else{
                    _logger->ERROR("from charging task construct failed!");
                    return false;
                }
                app_config_tasks.insert(app_config_tasks.begin(),from_charging_task);

                //2、生成拼接路径，并两两拼接task。
                for(int i = 0 ;i< app_config_tasks.size()-1; i++)
                {
                    auto from = app_config_tasks[i];
                    auto to = app_config_tasks[i+1];
                    std::vector<CleaningTaskInfo> task_pair=constructTaskPair(from,to);
                    for(const auto& e : task_pair)  _total_tasks.emplace_back(e);
                }
                _total_tasks.emplace_back(app_config_tasks.back());
                
                //3、返航路径task
                CleaningTaskInfo to_charging_task;
                if(_path_handler->createToChargingPathAndSave(app_config_tasks.back(),to_charging_path)){}
                else{
                    _logger->ERROR("can not create {} path",to_charging_path);
                    return false;
                }
                if(_task_handler->genTaskFromJoinPath(to_charging_path,to_charging_task)){}
                else{ 
                    _logger->ERROR("to charging task construct failed!");
                    return false;
                }
                _total_tasks.emplace_back(to_charging_task);

                //重置排序ID
                int index=0;
                for(auto &task:_total_tasks)
                {
                    _total_odometry +=task.path_size;
                    task.ID=index;
                    index+=1;   
                }
                //配置清洁报告里的相关信息
                _context.completed_task_id  = -1;
                // json _j = _total_tasks;
                // _logger->TRACE("total tasks: {}", _j.dump());
                _task_handler->saveTasks2File(CLEANING_TASKS_JSON_PATH,_total_tasks);
            }
            return true;
        }  
        // 构建冲完电后中断任务的恢复任务，出航-回到上次位置-继续上次任务
        bool constructBreakPointResumingTasks(CleaningContextInfo &s_context,std::vector<CleaningTaskInfo> &s_total_tasks){
            
            //1、出航路径和task
            std::string from_charging_path = _total_tasks.front().zone_dir + "/"+"from_charging.csv";
            _logger->TRACE("from_charging_path: {}", from_charging_path);
            CleaningTaskInfo from_charging_task;
            if(_task_handler->genTaskFromCharging(from_charging_path,from_charging_task)){} 
            else{
                    _logger->ERROR("from charging task construct failed!");
                    return false;
            }
            //2、出航路径到断点的路径和task
            CleaningTaskInfo temp;
            temp.zone_dir = _total_tasks.front().zone_dir;
            temp.path_file_location =JtcxDir::getFatherDir(_total_tasks.front().path_file_location)+"/"+"cur.csv";
            if(!_path_handler->getCurrentPose(temp.start)){
                _logger->ERROR("can't get current pose!!");
                return false;
            }
            std::vector<CleaningTaskInfo> task_pair=constructTaskPair(from_charging_task,temp);
            //断点是否在上一次清洁计划中
            if(!isWorkingNow())  // 不在清洁计划中
            {
                //重置 _context
                _task_handler->initCurrentContext(_context.map_id,_context.plan_id,_context);
                //重置 _total_tasks
                _total_tasks.swap(task_pair);
            }
            else{ 
                //在清洁计划中
                //3、裁切断点task
                int current_task_id = _context.completed_task_id +1;
                CleaningTaskInfo current_task = _total_tasks[current_task_id];
                CleaningTaskInfo clip_task;
                _task_handler->genClipTask(current_task,_way_point,clip_task);
                //去掉断点前的task
                _total_tasks.erase(_total_tasks.begin(),_total_tasks.begin()+current_task_id+1);

                //插入裁切task,拼接路径，出航路径
                _total_tasks.insert(_total_tasks.begin(),clip_task);
                _total_tasks.insert(_total_tasks.begin(), task_pair.begin(), task_pair.end());
                //重置 _context
                _task_handler->initCurrentContext(_context.map_id,_context.plan_id,_context);
            }

            //重置排序ID
            int index=0;
            for(auto &task:_total_tasks)
            {
                _total_odometry +=task.path_size;
                task.ID=index;
                index+=1;   
            }
            //记录
            _context.completed_task_id  = -1;
            // _task_handler->saveContext2File(_context, JtcxSingleton<CleaningException>::GetInstance()->NORMAL);
            // _task_handler->saveTasks2File(CLEANING_TASKS_JSON_PATH,_total_tasks);

            //返回
            s_context=_context;
            s_total_tasks=_total_tasks;
            return true;
        }
        std::vector<CleaningTaskInfo> constructTaskPair(const CleaningTaskInfo& from_task,const CleaningTaskInfo& to_task)
        {
            std::vector<CleaningTaskInfo> task_pair{from_task};
            std::string join_path_csv;
            if(_path_handler->createJoinPathAndSave(from_task,to_task,join_path_csv)){
                CleaningTaskInfo cti;
                if(_task_handler->genTaskFromJoinPath(join_path_csv, cti)){
                    task_pair.emplace_back(std::move(cti));
                }
            }
            _logger->TRACE("create task pair size: {}", task_pair.size());
            return task_pair;
        }
        // 主要执行任务线程
        void carryOutTasks()
        {
            int last_task_id = -1; 
            //如果还是上一个id，则说明没执行完，不能发送跟踪请求。
            while(true){
                if(_current_task_id.load() == -1 || _current_task_id.load() >= _total_tasks.size()){
                    _logger->TRACE("exit task thread!!");  
                    // ----- reset -----
                    _app_handler->uploadReport(_context);
                    resetCleaningParams();
                    return;
                }
                if(_current_task_id.load() > last_task_id)
                {
                    //JTCX_EXCEPTION_TRY
                    last_task_id += 1;
                    auto task = _total_tasks[last_task_id];
                    _logger->TRACE("----------- cur task {}/{} -----------", last_task_id, _total_tasks.size());

                    // 1. fujian
                    fujianControl(task);

                    // 2. tracking
                    if(!callPursuit(1, task.map_id, task.path_file_location)){
                        _logger->ERROR("call pursuit fail while carrying out task!!");
                        _current_task_id.store(-1);
                    }

                }else{
                    std::this_thread::yield();  
                }
            }
        }
        // 清洁状态上报回调
        void cleaningStatusHandler(const PurePursuitStatusConstPtr &msg)
        {
            if(msg->state == "WORKING" && _total_odometry!=0)
            {
                int completed_task_points= 0;
                for(int i=0; i<_context.completed_task_id + 1; i++)
                {
                    completed_task_points +=_total_tasks[i].path_size;
                }
                //计算process;
                _way_point = std::max(0, (int)msg->waypoint);
                _context.process = 1.0*(_way_point + completed_task_points) / _total_odometry;
                _context.process = std::max(0.0, std::min(1.0,_context.process));
                // _logger->TRACE("completed_task_points: {}, waypoint: {}", completed_task_points, _way_point);
                // _logger->TRACE("_total_odometry: {}, process: {}", _total_odometry, _context.process);

                // info related process
                _context.odometry = _total_odometry *_context.process;
                _context.area = JTCX_CAR_WIDTH *_context.odometry;

                //设置当前时间为时间戳
                _task_handler->_time_tool->setStampTime();
                _context.stop_time = _task_handler->_time_tool->getStampTime();
                _context.cleaning_time = _task_handler->_time_tool->getDurationTime();

                //选取有用信息上传
                if(_battery_supporter.load())   _app_handler->uploadStatus(_context);

                // static bool single_heartbeat = true;
                // if (!_battery_supporter.load() &&  single_heartbeat){
                //     single_heartbeat = false;
                //     _context.exception=JtcxSingleton<CleaningException>::GetInstance()->LOW_POWER;
                //     _task_handler->updateContext2File(CLEANING_CONTEXT_JSON_PATH,_context);
                // }
            }
        }
        // 清洁任务停止都统一通过这个函数来停
        void stopCleaningTask(const std::string& cleaning_exception)
        {
            PurePursuitResult ppr;
            ppr.tracking_result = cleaning_exception;
            cleaningResultHandler(boost::make_shared<PurePursuitResult>(ppr));
        }
        // 清洁任务执行结果回调，成功、中断、失败等等
        void cleaningResultHandler(const PurePursuitResultConstPtr &msg){
            const auto& ecn = JtcxSingleton<CleaningException>::GetInstance();
            const auto& res = msg->tracking_result;
            
            // 如果在清洁过程中失败或终端才更新记录，否则不理
            if(isWorkingNow()){
                if(res == ecn->INTERRUPT || res == ecn->FAIL)
                {
                    _context.exception = res;
                    _task_handler->updateContext2File(_context, res);
                    _current_task_id.store(-1);
                    return;
                    //JTCX_EXCEPTION_THROW(PURSUIT_ERROR,msg->tracking_result);
                }
            }
            // 子任务成功
            if(res == ecn->SUCCESS)
            {
                _context.completed_task_id +=1;
                if(_context.completed_task_id <_total_tasks.size()-1)
                {
                    _current_task_id.store(_context.completed_task_id +1);
                    //每完成一个task,更新清洁记录,覆盖上一个task的清洁记录
                    _task_handler->updateContext2File(_context, ecn->NORMAL);
                }
                else{
                    _task_handler->updateContext2File(_context, ecn->NORMAL);
                    _current_task_id.store(-1);
                }
            }
        }
        // 统计平均速度
        void twistHandler(const geometry_msgs::TwistConstPtr& msg)
        {
            if(_cleaning_thread_ptr != nullptr)
            {
                _velocity.push_back(msg->linear.x);
                if(_velocity.size() > 50)   _velocity.pop_front();
                float twistSum = 0;
                for (const auto& e : _velocity) twistSum += e;
                _context.average_velocity = twistSum / (_velocity.size() + 1.0);
            }
        }
        bool callPursuit(int command=1,string map="",string path_name="",string mode = "path_tracking_with_path_name"){
            Pursuit srv;
            srv.request.command = command;
            srv.request.map = map;
            srv.request.path_name =path_name;
            srv.request.mode = mode;

            _logger->TRACE("req map_name: {}", srv.request.map);
            _logger->TRACE("req path_name: {}", srv.request.path_name);
            _logger->TRACE("call pursuit req cmd: {}", srv.request.command);
            if(ros::service::call("/pursuit", srv)){
                if(srv.response.status < 0 || srv.response.status > 4){
                    _logger->ERROR("call pursuit fail, code({}), why({})", srv.response.status, srv.response.message);
                    //JTCX_EXCEPTION_THROW(PURSUIT_ERROR,"call pursuit fail");
                    return false;
                }
                return true;
            }
            else{
                //JTCX_EXCEPTION_THROW(PURSUIT_ERROR,"cant call pursuit!!");
                _logger->ERROR("pursuit offline");
                return false;
            }
        }

        void controlRubber(std::string cmd)
        {
            if(cmd.empty())    return;
            MappingSave ms;
            ms.request.filename = cmd;
            _logger->TRACE("control rubber: {}", cmd);
            if(!ros::service::call("/auto_brush/set_cleaning_type", ms)){
                _logger->ERROR("rubber offline!!");
            }
        }

		void controlBrush(bool flag)
		{
			BcmCommand bcm;
			bcm.auto_clean = flag;
			_bcm_pub.publish(bcm);
            _logger->TRACE("{} brush!!", (flag ? "open" : "close"));
		}

		void fujianControl(const CleaningTaskInfo& task)
		{
			const auto& cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance();
			
			if(task.cleaning_mode.empty() || task.cleaning_mode == cleaning_mode->FROM_CHARGING ||
				task.cleaning_mode == cleaning_mode->TO_CHARGING || task.cleaning_mode == cleaning_mode->JOIN)
				{
					// 1. set rubber to small
					controlRubber("small");
					// 2. close brush
					controlBrush(false);
				}
			else
			{
				controlRubber(task.rub_type);
				controlBrush(true);
			}
		}
    };
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cleaning_task");
    RobotCleaningState rcs;
    ros::spin();
    return 0;
}