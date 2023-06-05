#pragma once

// fsm
#include "JtcxRobotStatesBase.hpp"
#include "JtcxLocalizationHelper.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxExternalEventQueueManager.hpp"

// ros msg header
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

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

// json
#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief keep alive during whole life cycle of fsm, remind cleaning regularly
 * 
 */
class CleaningTaskScheduler
{
private:

    struct Task{
        double ts;  ///< time stamp
        std::string map_name;
        std::string plan_name;
        int times;  

        bool operator<(const Task& r){
            return (difftime(ts, r.ts) < 0);
        }
        NLOHMANN_DEFINE_TYPE_INTRUSIVE(Task, ts, map_name, plan_name, times);
    };


    const float _valid_time_interval = 5.0; ///< when set time, it should be 5s later than now

    std::mutex _task_queue_lock;
    std::priority_queue<Task> _task_queue;  ///< queue containing task in lastest
    std::unique_ptr<std::thread> _timer;    ///< check when to execute cleaning task

    CleaningTaskScheduler();
    CleaningTaskScheduler(const CleaningTaskScheduler&) = delete;
    CleaningTaskScheduler& operator=(const CleaningTaskScheduler&) = delete;
    
public:
    static const std::string _task_file;

    static CleaningTaskScheduler& getInstance(){
        static CleaningTaskScheduler e;
        return e;
    }

    /**
     * @brief external event
     * 
     * @return true 
     * @return false 
     */
    bool pushBackTask(const Task&);

    /**
     * @brief check time when it is time to clean 
     * 
     */
    void timerHandler();

    void saveTaskToFile();
    void loadTaskFromFile();

    ~CleaningTaskScheduler();
};

// singleton for cleaning external interface 
class ExternalCleaningInterface
{
public:
    static ExternalCleaningInterface& getInstance(){
        static ExternalCleaningInterface e;
        return e;
    }

    /**
     * @brief : control tracking controller
     * 
     */
    void controlCleaningHandler(mobile_platform_msgs::Pursuit::Request&, mobile_platform_msgs::Pursuit::Response&, std::promise<bool>&);    


    // ----------- getter and setter -----------
    std::string getMapName(){return _map_name;}
    std::string getTaskName(){return _task_name;}
    int getTimes(){return _times;}

private:
    ExternalCleaningInterface(){};
    ExternalCleaningInterface(const ExternalCleaningInterface&);
    ExternalCleaningInterface& operator=(const ExternalCleaningInterface&);

    // tmp data
    std::string _map_name;
    std::string _task_name;
    int _times;

};

class RobotCleaningState : public RobotStateBase
{
    public:

        RobotCleaningState();

        ~RobotCleaningState();
            
        /**
         * @brief when cleaning, following state must be checked
         * 0. localizaiton node
         * 1. localization performance
         * 2. tracking controller node
         */
        virtual void checkHealthState() override;

        /**
         * @brief when cleaning, some data need to collect and do some calculation 
         * 
         */
        void statisticCleaningData();

        /**
         * @brief when exiting, some context need to persist to file for loading next time 
         * 
         */
        void saveContextToFile();

        /**
         * @brief when entering, some context need to load from file
         * 
         */
        void loadContextFromFile();

        void chassisHandler(const mobile_platform_msgs::ChassisConstPtr&);
        void cleaningStartHandler(const nav_msgs::PathConstPtr&);
        void cleaningStatusHandler(const mobile_platform_msgs::PurePursuitStatusConstPtr&);
        void cleaningResultHandler(const mobile_platform_msgs::PurePursuitResultConstPtr&);
        void twistHandler(const geometry_msgs::TwistConstPtr&);

    private:

        // ros related
        ros::Subscriber _chassis_sub;
        ros::Subscriber _start_sub;
        ros::Subscriber _status_sub;
        ros::Subscriber _result_sub;
        ros::Subscriber _twist_sub;
        ros::ServiceClient _control_cleaning_client;
        ros::ServiceClient _cleaning_logger_client;


        std::atomic_bool _start_check_health_flag{true};    ///< control check health state thread

        std::unique_ptr<JtcxLocalizationHelper> _loc_helper;

        // context related 
        struct cleaningContext{
            std::string map;
            std::string path;
            double start_time;
            double cleaning_time;
            std::string mode;
            double area;
            double stop_time;
            double average_velocity;
            float process;
            double odometry;
            std::string exception;

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
            }

            NLOHMANN_DEFINE_TYPE_INTRUSIVE(cleaningContext, map, path, start_time,
            mode, area, cleaning_time, stop_time, average_velocity, process, odometry, exception);
        }_context;

        std::vector<std::string> _mode_list{
            "VCU_EMERGENCY_STOP", "HUMAN_MANUAL", "VCU_MANUAL", "IPC_EMERGENCY",
            "IPC_MANUAL",         "IPC_AUTO",     "IPC_CLIENT"
        };

        bool _save_context_flag{true};

    public:
        static const std::string _save_context_file;
};