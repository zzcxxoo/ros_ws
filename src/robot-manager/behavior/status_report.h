#include <ros/ros.h>
#include <string>
#include "common/common.h"
#include <common/JtcxLogWrapper.hpp>
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include <mobile_platform_msgs/Chassis.h>
#include <mobile_platform_msgs/LocalizationLost.h>
#include <mobile_platform_msgs/PurePursuitStatus.h>
#include <mobile_platform_msgs/Agent.h>
using namespace mobile_platform_msgs;
using json = nlohmann::json;

extern JtcxLogWrapper* jlw_for_status_report;
#define SLG jlw_for_status_report->logger

namespace status
{
    static const std::string CLEAN = "clean";
    static const std::string PAUSE = "pause";
    static const std::string IDLE = "idle";
    static const std::string UPGRADE = "upgrade";
    static const std::string RECALL = "recall";
    static const std::string CHARGE = "charge";
} // namespace status

struct Props{
    int battery;
    int battery_threshold;
    int sound;
    std::string working_status;
    std::string location_status;
    json gps;
    std::string rubber;
    
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(Props, battery, battery_threshold,
    sound, working_status, location_status, gps, rubber);
};

class HeartbeatStatus{
private:
    ros::WallTimer _timer;
    int _cnt_limit{10};

protected:
    int _cnt{0};
    std::string _stat;
    bool _dead{false};

public:
    void init(ros::NodeHandle& n, float dur, int l){
        _cnt_limit = l;
        _timer = n.createWallTimer(ros::WallDuration(dur), &HeartbeatStatus::timeHandler, this);
    };
    std::string getStatus(){return _stat;}

    virtual void dead() = 0;
    
    void timeHandler(const ros::WallTimerEvent& e){
        if(!_dead && ++_cnt > _cnt_limit){
            SLG->warn("some topic cant be received, dead is called!!");
            dead();
        }
    }   
};

class LocStatus : public HeartbeatStatus
{
private:
    ros::Subscriber _sub;

public:

    LocStatus(ros::NodeHandle& n, std::string topic, float dur, int l){
        SLG->trace("init LocStatus obj!!");
        init(n, dur, l);
        _sub = n.subscribe(topic, 1, &LocStatus::handler, this);
        // init first status
        dead();
    }

    void handler(const LocalizationLostConstPtr& ptr){
        _dead = false;
        _cnt = 0;
        _stat = (ptr->lost ? "lost" : "normal");
    }

    virtual void dead() override{
        _dead = true;
        _stat = "off";
    }
};

class PureStatus : public HeartbeatStatus
{
private:
    ros::Subscriber _sub;

public:
    PureStatus(ros::NodeHandle& n, std::string topic, float dur, int l){
        SLG->trace("init PureStatus obj!!");
        init(n, dur, l);
        _sub = n.subscribe(topic, 1, &PureStatus::handler, this);
        dead();
    }

    void handler(const PurePursuitStatusConstPtr& ptr){
        _dead = false;
        _cnt = 0;

        std::string state = ptr->state;
        if(state == "IDLE"){_stat = status::IDLE;}
        else if(state == "PAUSE"){_stat = status::PAUSE;}
        else if(state == "WORKING"){
            std::string tmp;
            if(ros::param::get("/tracking_path_name", tmp)){
                if(tmp == "recall"){
                    _stat = status::RECALL;
                    return;
                }
            }
            _stat = status::CLEAN;
        }else{_stat = "error";}
    }

    virtual void dead() override{
        _dead = true;
        _stat = status::IDLE;
    }
};

class CleanStatus : public HeartbeatStatus
{
    private:
    ros::Subscriber _sub;

public:
    CleanStatus(ros::NodeHandle& n, std::string topic, float dur, int l){
        SLG->trace("init CleanStatus obj!!");
        init(n, dur, l);
        _sub = n.subscribe(topic, 1, &CleanStatus::handler, this);
        dead();
    }

    void handler(const std_msgs::StringConstPtr& ptr){
        _dead = false;
        _cnt = 0;
        _stat = ptr->data;
    }

    virtual void dead() override{
        _dead = true;
        _stat = "{}";
    }
};

class StatusReport{

private:    
    // ros
    ros::NodeHandle _nh;
    
    ros::Subscriber _chassis_sub;
    ros::Subscriber _gps_sub;
    ros::Subscriber _cleaning_sub;

    ros::ServiceServer _props_srv;

    Props _pps;

    std::unique_ptr<LocStatus> _loc_ptr;
    std::unique_ptr<PureStatus> _pp_ptr;
    std::unique_ptr<CleanStatus> _cn_ptr;

public:
    StatusReport();

    bool getProps(AgentRequest& req, AgentResponse& res);

    std::string getPos();

    std::string getStatus();
    
    void chassisHandler(const ChassisConstPtr& ptr);
    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& ptr);
    void cleaningHandler(const std_msgs::StringConstPtr& ptr);
};
