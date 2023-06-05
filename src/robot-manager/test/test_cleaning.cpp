#pragma once

// ros msg header
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// mobile_platform_msgs header
#include "mobile_platform_msgs/LocalizationLost.h"
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
#include <chrono>

// io
#include <fstream>
#include <boost/filesystem.hpp>
#include <nlohmann/json.hpp>
#include "JtcxLogWrapper.hpp"
#include <spdlog/fmt/fmt.h>
#include <spdlog/fmt/bundled/ranges.h>

using json = nlohmann::json;
using namespace std::chrono_literals;

static JtcxLogWrapper logger("RobotCleaningState", LOG_LEVEL::DEBUG);

static const std::string _save_context_file = std::string(std::getenv("HOME")) + "/.robot/config/cleaning/context.json";
static const size_t _max_record_num = 100;

namespace cleaningException{
    static const std::string NORMAL = "Normal";
    static const std::string INTERRUPT = "Interrupt";
    static const std::string UNKNOWN = "Unkown";
    static const std::string NODE_EXIT = "Node_Exit";
    static const std::string CALL_FAIL = "Call_Fail";
    static const std::string CALL_NO_REP = "Call_No_Reply";
    static const std::string PP_EXIT = "Pure_Pursuit_Exit";
}


class RobotCleaningState
{
    private:

        // ros related
        ros::NodeHandle _nh;

        ros::ServiceServer _rcs_srv;

        ros::Subscriber _chassis_sub;
        ros::Subscriber _start_sub;
        ros::Subscriber _status_sub;
        ros::Subscriber _result_sub;
        ros::Subscriber _twist_sub;
        ros::ServiceClient _control_cleaning_client;
        ros::ServiceClient _cleaning_logger_client;

        ros::Publisher _cleaning_state_pub;

        std::atomic_bool _start_check_health_flag{true};    ///< control check health state thread
        std::atomic_bool _run_flag{true};
        std::atomic_bool _run_exit_flag{true};

        bool _task_finish;
        std::mutex _task_finish_lock;
        std::condition_variable _task_finish_cv;

        int _cur_task_idx{-1};
        std::vector<float> _task_weight;

        std::vector<double> _fe_pts;    // size 5: first point x, y last point x, y, max_look_dist

        // std::unique_ptr<JtcxLocalizationHelper> _loc_helper;

        // context related 
        struct cleaningContext{
            std::string map;    
            std::string path;
            std::string mode;
            double area;
            double start_time;
            double cleaning_time;
            double stop_time;
            double average_velocity;
            double odometry;
            std::string exception;
            float process;
            int total_times;
            int first_idx;
            int path_size;

            void resetContext(){
                map = "";
                path= "";
                start_time = 0;
                cleaning_time = 0;
                mode = "";
                area = 0;
                stop_time = 0;
                average_velocity = 0;
                process = 0;
                odometry = 0;
                exception = "";
                total_times = 0;
                first_idx = -1;
                path_size = 0;
            }

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(cleaningContext, map, path,
            mode, area, start_time, cleaning_time, stop_time, average_velocity, odometry,
            exception, process, total_times, first_idx, path_size);

        }_context;

        std::deque<float> _velocity;

        float _last_process{0.0};
        bool _load_flag;

        std::vector<std::string> _mode_list{
            "VCU_EMERGENCY_STOP", "HUMAN_MANUAL", "VCU_MANUAL", "IPC_EMERGENCY",
            "IPC_MANUAL",         "IPC_AUTO",     "IPC_CLIENT"
        };

        bool _save_context_flag{true};
        std::unique_ptr<std::thread> _cleaning_thread_ptr;
        std::vector<mobile_platform_msgs::PursuitRequest> _task_list;

        std::thread _check_pursuit_health;
        std::atomic_bool _check_pursuit_flag{true};

    public:

        RobotCleaningState(){
            logger.logger->info("construct obj!!");
            // create dir
            auto task_file = bfs::path(_save_context_file);
            if(!bfs::exists(task_file.parent_path()))   bfs::create_directories(task_file.parent_path());  
    
            _rcs_srv = _nh.advertiseService("/ui/cleaning_task", &RobotCleaningState::cleaningHandler, this);

            // init ros related
            // _chassis_sub = _nh.subscribe("/chassis", 1, &RobotCleaningState::chassisHandler, this);
            _status_sub = _nh.subscribe("/pure_pursuit/status", 1, &RobotCleaningState::cleaningStatusHandler, this);
            _result_sub = _nh.subscribe("/pure_pursuit/tracking_result", 10, &RobotCleaningState::cleaningResultHandler, this, ros::TransportHints().tcpNoDelay());
            _twist_sub = _nh.subscribe("/nav/cmd_vel", 1, &RobotCleaningState::twistHandler, this);
            
            _cleaning_state_pub = _nh.advertise<std_msgs::String>("/ui/logger/cleaning_status", 1); 

            _cleaning_thread_ptr.reset(new std::thread());  // initalize a empty obj
            _check_pursuit_health = std::thread(&RobotCleaningState::checkPursuitAlive, this);
        }

        ~RobotCleaningState(){
            // cleaning run thread
            std::unique_lock<std::mutex> lock(_task_finish_lock);
            if(_run_flag){
                _context.exception = cleaningException::NODE_EXIT;
                saveContextToFile();
            }
            // set flag in advance to make sure thread exit!!
            _task_finish = true;
            _run_flag = false;
            lock.unlock();
            _task_finish_cv.notify_one();   
            if(_cleaning_thread_ptr->joinable())    _cleaning_thread_ptr->join();
            _check_pursuit_flag = false;
            _check_pursuit_health.join();
            logger.logger->info("exit obj!!");
        }

        int open_popen(const std::string &cmd, std::vector<std::string> &out) {
            FILE *fp;
            const int sizebuf = 2048;
            char buff[sizebuf];
            out = std::vector<std::string>();
            fp = popen(cmd.c_str(), "r");
            if (!fp) std::cerr << "Couldn't start command" << std::endl;

            while (fgets(buff, sizeof(buff), fp)) {
                std::string cur_string = "";
                cur_string += buff;
                out.push_back(cur_string.substr(0, cur_string.size()));
            }
            return pclose(fp);
        }

        void checkPursuitAlive()
        {
            while (_check_pursuit_flag)
            {
                std::vector<std::string> out;
                open_popen("rosnode info /pure_pursuit 2>&1", out);

                auto it = std::find_if(out.begin(), out.end(), [](const std::string& e){
                    return (e.find("ERROR") != e.npos) || (e.find("cannot") != e.npos);
                });
                // if find the node is dead
                if(it != out.end())
                {   
                    // thread is still running
                    logger.logger->warn("node /pure_pursuit is died!!");
                    if(!_run_exit_flag){
                        _run_flag = false;
                        std::unique_lock<std::mutex> lock(_task_finish_lock);
                        _task_finish = true;
                        _context.exception = cleaningException::PP_EXIT;
                        saveContextToFile();
                        lock.unlock();
                        _task_finish_cv.notify_one();
                    }
                }
                std::this_thread::sleep_for(0.5s);
            }  
        }

        bool cleaningHandler(mobile_platform_msgs::PursuitRequest& req, mobile_platform_msgs::PursuitResponse& res)
        {
            if(!_run_exit_flag){
                std::string msg = "cleaning task thread does not finished completely!!";
                res.status = -1;
                res.message = msg;
                logger.logger->warn(msg);
                return true;
            }

            if(req.mode == "load_context"){
                if(!loadContextFromFile()){
                    res.status = -1;
                    res.message = "file may be not exsit or something wrong!!";
                    return true;
                }
            }else{
                if(!loadContextFromReq(req)){
                    res.status = -2;
                    res.message = "load context from req fail!!";
                    return true;
                }
                // judge loop
                if(req.task_index > 1){
                    logger.logger->trace("loop pts: {}", _fe_pts);
                    double diff_x = _fe_pts[2] - _fe_pts[0];
                    double diff_y = _fe_pts[3] - _fe_pts[1];
                    if(std::sqrt(diff_x * diff_x + diff_y * diff_y) > _fe_pts[4]){
                        std::string msg = "the route cant not loop!!";
                        logger.logger->error(msg);
                        res.message = msg;
                        res.status = 1;
                        return true;
                    }
                }
            }
            
            std::string msg;
            if(constructTask()){
                msg = "construct task success!";
                res.status = 0;
                logger.logger->info(msg);
                // start thread!!
                if(_cleaning_thread_ptr->joinable())    _cleaning_thread_ptr->join();
                _last_process = _context.process;
                _cleaning_thread_ptr.reset(new std::thread(&RobotCleaningState::run, this));
                
            }else{
                res.status = 1;
                msg = "construct task fail!";
                logger.logger->error(msg);
            }
            res.message = msg;
            return true;
        }

        bool constructTask()
        {
            // context is not correctly init!!
            if(_context.path_size == 0){
                logger.logger->error("context is not correctly init!!");
                return false;
            }

            std::vector<mobile_platform_msgs::PursuitRequest> req_vec;
            // 1. --------- start ----------
            // f(first_idx, process, path_size, times) => task
            mobile_platform_msgs::PursuitRequest req;
            _task_weight.clear();

            req.command = 1;
            req.mode = "p2p_tracking";
            req.map = _context.map;
            req.path_name = _context.path;

            json j = _context;
            logger.logger->info("context: {}", j.dump());

            int& path_size = _context.path_size;
            int& times = _context.total_times;
            float& process = _context.process;

            int total_points_num = path_size * times;
            int cur_idx = _context.first_idx + process * total_points_num;
            cur_idx %= path_size;

            int rest_points = (1 - process) * total_points_num; 
            int rest_pts_in_circle = path_size - cur_idx;
            int exceed_rest_points = rest_points - rest_pts_in_circle;
            req.start_idx = cur_idx;

            if(exceed_rest_points <= 0){
                req.end_idx = cur_idx + rest_points;
                req_vec.emplace_back(req);
                logger.logger->trace("task 0: [start_idx {}, end_idx {}", req.start_idx, req.end_idx);
                _task_weight.push_back(1.0 - process);
                _task_list.swap(req_vec);
                return true;
            }

            req.end_idx = path_size - 1;
            req_vec.emplace_back(req);
            logger.logger->trace("task 0: [start_idx {}, end_idx {}", req.start_idx, req.end_idx);
            _task_weight.push_back(1.0 * rest_pts_in_circle / total_points_num);

            rest_points -= rest_pts_in_circle;
            int circles = rest_points / path_size;
            int rest_circle = rest_points % path_size;

            for(int i=0; i<circles; i++){
                req.start_idx = 0;
                req.end_idx = path_size - 1;
                req_vec.emplace_back(req);
                logger.logger->trace("task {}: [start_idx {}, end_idx {}", i+1, req.start_idx, req.end_idx);
                _task_weight.push_back(1.0 * path_size / total_points_num);
            }

            if(rest_circle > 0){
                req.start_idx = 0;
                req.end_idx = rest_circle;
                req_vec.emplace_back(req);
                logger.logger->trace("task {}: [start_idx {}, end_idx {}", req_vec.size() - 1, req.start_idx, req.end_idx);
                _task_weight.push_back(1.0 * rest_circle / total_points_num);
            }
            logger.logger->trace("task weight: {}", _task_weight);
            _task_list.swap(req_vec);
            
            return true;
        }

        void run(){
            _run_exit_flag = false;
            logger.logger->info("start running cleaning task!!");
            logger.logger->trace("task list num: {}", _task_list.size());

            _run_flag = true;
            for(int i=0; i<_task_list.size(); i++){
                if(!_run_flag)  break;

                mobile_platform_msgs::Pursuit srv;
                srv.request = _task_list[i];
                // unset the flag fisrt before call tracking controller
                _task_finish = false;

                if(ros::service::call("/pursuit", srv)){
                    if(srv.response.status != 0){
                        logger.logger->error("pursuit called but get error: {}", srv.response.message);
                        _context.exception = cleaningException::CALL_FAIL;
                        saveContextToFile();
                        break;
                    }
                    // if success
                    _cur_task_idx = i;
                }else{
                    logger.logger->error("call pursuit no response!!");
                    _context.exception = cleaningException::CALL_NO_REP;
                    saveContextToFile();
                    break;
                }
                logger.logger->info("----------- executing task {} -----------", _cur_task_idx);
                std::unique_lock<std::mutex> lock(_task_finish_lock);
                while(!_task_finish){_task_finish_cv.wait(lock);}
                logger.logger->trace("cv is notified!!");
            }

            logger.logger->info("exit run thread");
            _run_exit_flag = true;
        }
            
        void saveContextToFile(){
            // 1. if context file exist
            std::vector<cleaningContext> cc_vec;
            if(bfs::exists(bfs::path(_save_context_file))){
                // read first
                std::ifstream inf(_save_context_file);
                json j = json::parse(inf);
                cc_vec = j.get<std::vector<cleaningContext>>();
                inf.close();
            }
            std::ofstream out(_save_context_file);
            if(!out.is_open()){
               auto msg = fmt::format("{} cant open!!", _save_context_file);
                // logger.logger->error(msg);
                throw std::runtime_error(msg);
            }
            _context.stop_time = ros::WallTime::now().toSec();
            _context.cleaning_time = _context.stop_time - _context.start_time;

            // if load, refresh old one
            // if not, push back
            if(_load_flag){
                cc_vec.back() = _context;
            }else{
                if(cc_vec.size() >= _max_record_num)    cc_vec.erase(cc_vec.begin());
                cc_vec.emplace_back(_context);
            }
            
            json j = cc_vec;
            out << j;
            out.close();
        }

        /**
         * @brief when entering, some context need to load from file
         * 
         */
        bool loadContextFromFile(){
            // only if file exist then load
            if(bfs::exists(bfs::path(_save_context_file))){
                std::ifstream inf(_save_context_file);
                json j = json::parse(inf);
                inf.close();
                std::vector<cleaningContext> cc_vec = j.get<std::vector<cleaningContext>>();
                if(!cc_vec.empty())     _context = cc_vec.back();
                else{
                    logger.logger->error("context file is empty or invalid!!");
                    return false;
                }

                if(_context.exception == cleaningException::NORMAL){
                    logger.logger->warn("last context is a normal one, no need to reload this context!!");
                    return false;
                }
                logger.logger->info("context is loaded with path name {}", _context.path);
                
                _load_flag = true;
                return true;
            }
            return false;
        }

        // if from req, the cleaning should be new one!!
        bool loadContextFromReq(const mobile_platform_msgs::PursuitRequest& req)
        {
            _context.resetContext();
            _context.map = req.map;
            _context.path = req.path_name;
            _context.total_times = req.task_index;

            _context.start_time = ros::WallTime::now().toSec();

            mobile_platform_msgs::Pursuit srv;
            srv.request = req;
            if(ros::service::call("/pursuit/info", srv)){
                if(srv.response.status == 0){
                    _context.path_size = srv.response.waypoint_num;
                    _context.first_idx = srv.response.first_idx;
                    _fe_pts = srv.response.pts;
                    _load_flag = false;
                    return true;
                }
                logger.logger->error("call fail for {}", srv.response.message);
            }else{
                logger.logger->error("can not ping /pursuit/info!!");
                return false;
            }
        }

        // void chassisHandler(const mobile_platform_msgs::ChassisConstPtr& msg){}

        void uploadStatus()
        {
            json jv;
            jv["plan_name"] = _context.path;
            jv["clean_times"] = _context.total_times;
            // jv["current_times"] = int(cur_process * tmp) + 1;
            jv["current_times"] = int(ceil(_context.process * _context.total_times));
            
            jv["area"] = _context.area;
            jv["odometry"] = _context.odometry;
            jv["duration"] = ros::WallTime::now().toSec() - _context.start_time;
            std_msgs::String msg;
            msg.data = jv.dump();
            _cleaning_state_pub.publish(msg);
        }

        void cleaningStatusHandler(const mobile_platform_msgs::PurePursuitStatusConstPtr& msg){
            if(msg->state == "WORKING" && _cur_task_idx != -1){
                auto& task = _task_list[_cur_task_idx];

                _context.process = _last_process;
                for (int i = 0; i < _cur_task_idx; i++)
                    _context.process += _task_weight[i]; 
                
                _context.process += _task_weight[_cur_task_idx] * (msg->waypoint - task.start_idx) / (task.end_idx - task.start_idx);
                _context.process = std::max(0.0f, std::min(1.0f, _context.process));

                logger.logger->trace("process: {}", _context.process);
                // info related process
                _context.odometry = 0.2 * (_context.total_times * _context.path_size) * _context.process;
                _context.area = 0.8 * _context.odometry;

                uploadStatus();
            }
        }
        
        void cleaningResultHandler(const mobile_platform_msgs::PurePursuitResultConstPtr& msg){
            logger.logger->trace("cleaning stop by reason: {}", msg->tracking_result);
            std::unique_lock<std::mutex> lock(_task_finish_lock);
            _task_finish = true;

            if(msg->tracking_result == "Success"){
                // if tracking success, delete last context!!
                if(_cur_task_idx == _task_list.size() - 1){
                    _context.exception = cleaningException::NORMAL;
                    saveContextToFile();
                }
            }else{
                _context.exception = cleaningException::INTERRUPT;
                saveContextToFile();
                // if not success, thread should exit!!
                _run_flag = false;  
            }

            lock.unlock();
            logger.logger->trace("receive result and unlock");
            _task_finish_cv.notify_one();
            logger.logger->trace("result: notify thread!!");
        }

        void twistHandler(const geometry_msgs::TwistConstPtr& msg){
            _velocity.push_back(msg->linear.x);
            if(_velocity.size() > 50)   _velocity.pop_front();

            float twistSum = 0;
            for (const auto& e : _velocity)
                twistSum += e;

            _context.average_velocity = twistSum / (_velocity.size() + 1.0);
        }

};


int main(int argc, char *argv[])
{

    ros::init(argc, argv, "test_cleaning_node");
    
    RobotCleaningState rcs;

    ros::spin();

    return 0;
}
