// io
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <common/nlohmann/json.hpp>
using json = nlohmann::json;

// ros
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>
#include <mobile_platform_msgs/Chassis.h>
using namespace mobile_platform_msgs;

// stl
#include <string>
#include <vector>
#include <chrono>

// log
#include <common/JtcxLogWrapper.hpp>
#include <spdlog/fmt/bundled/ranges.h>
static JtcxLogWrapper lg("basic_check", LOG_LEVEL::DEBUG);
#define LG lg.logger

std::vector<std::string> open_popen(const std::string &cmd) {
    FILE *fp;
    const int sizebuf = 2048;
    char buff[sizebuf];
    std::vector<std::string> out;
    fp = popen(cmd.c_str(), "r");
    if (!fp){
        std::cerr << "Couldn't start command" << std::endl;
        return out;
    }
    while (fgets(buff, sizeof(buff), fp)) {
        std::string cur_string = "";
        cur_string += buff;
        out.push_back(cur_string.substr(0, cur_string.size()));
    }
    pclose(fp);
    return out;
}

bool checkNodeAlive(const std::string& node)
{
    auto out = open_popen(fmt::format("rosnode info /{} 2>&1", node));

    auto it = std::find_if(out.begin(), out.end(), [](const std::string& e){
        return (e.find("ERROR") != e.npos) || (e.find("cannot") != e.npos);
    });
    // if find the node is dead
    return (it == out.end());
}

static json rule;
static json g_status;
std::mutex g_lock_status;

namespace GlobalConfigStatus
{
    static const std::string normal = "normal";    
    static const std::string exception = "exception";
    static const std::string node_dead = "node dead";
    static const std::string hz_wrong = "hz wrong";
}

void initConfig(std::string rule_file)
{
    std::ifstream inf(rule_file);
    rule = json::parse(inf);
    inf.close();

    json v;
    v["status"] = GlobalConfigStatus::exception;
    v["msg"] = R"(
        {
            "nodes" : {},
            "topics" : {}
        }
    )"_json;

    for(auto& e : rule.items()){
        g_status[e.key()] = v;
    }
}


class CheckBasic
{

public:
    CheckBasic(std::string n, json config) : _name(n){
        if(config.contains("nodes"))
            _nodes = config["nodes"].get<std::vector<std::string>>();

        if(config.contains("topics"))
            _topics = config["topics"].get<std::vector<json>>();

        for(const auto& e : _nodes){
            g_status[n]["msg"]["nodes"][e] = GlobalConfigStatus::exception;
        }
        for(const auto& e : _topics){
            auto tn = e["name"].get<std::string>();
            g_status[n]["msg"]["topics"][tn] = GlobalConfigStatus::exception;
            _topics_cnt[tn] = 0;
        }
    }

    void initRosReleated(ros::NodeHandle& nh){
        auto tr = nh.createWallTimer(ros::WallDuration(1.), &CheckBasic::checkHealthCallback, this);
        _timers.emplace_back(std::move(tr));
        tr = nh.createWallTimer(ros::WallDuration(5.), &CheckBasic::checkNodesCallback, this);
        _timers.emplace_back(std::move(tr));
        for(const auto& e : _topics){
            tr = nh.createWallTimer(ros::WallDuration(_win_size), std::bind(&CheckBasic::checkTopicsCallback, this, std::placeholders::_1, e));
            _timers.emplace_back(std::move(tr));

            // init sub
            auto n = e["name"].get<std::string>();
            auto tn = fmt::format("/{}", n);
            auto sub = nh.subscribe<topic_tools::ShapeShifter>(tn, 1, std::bind(&CheckBasic::topicCallback, this, std::placeholders::_1, n));
            _topics_sub.emplace_back(std::move(sub));
        }
    }

protected:

    std::string _name;
    std::vector<std::string> _nodes;
    std::vector<json> _topics;
    const double _win_size{2.};
    std::map<std::string, std::atomic_uint64_t> _topics_cnt;
    std::vector<ros::Subscriber> _topics_sub;
    std::vector<ros::WallTimer> _timers;
    
    std::atomic_int8_t _health_count{0};
    const int _health_thresh{10};

    void checkHealthCallback(const ros::WallTimerEvent& e)
    {
        _health_count++;
        LG->trace("health cnt: {}", (int)_health_count);
        if(_health_count >= _health_thresh){
            _health_count = _health_thresh;
            g_status[_name]["status"] = GlobalConfigStatus::normal;
        }
    }

    void checkNodesCallback(const ros::WallTimerEvent& e)
    {
        for(const auto& e: _nodes){
            g_lock_status.lock();
            if(!checkNodeAlive(e)){
                LG->warn("node {} is dead", e);
                _health_count = 0;
                g_status[_name]["status"] = GlobalConfigStatus::exception;
                g_status[_name]["msg"]["nodes"][e] = GlobalConfigStatus::node_dead;
            }else{
                g_status[_name]["msg"]["nodes"][e] = GlobalConfigStatus::normal;
            }
            g_lock_status.unlock();
        }
    }

    void checkTopicsCallback(const ros::WallTimerEvent& e, json& j){

        std::string topic_name = j["name"];
        float expect_hz = j["hz"];
        float offset_hz = j["offset"];
        auto af = 1. * _topics_cnt[topic_name] / _win_size;
        LG->trace("topic /{}'s rate: {}", topic_name, af);
        _topics_cnt[topic_name] = 0;

        g_lock_status.lock();
        if(std::abs(expect_hz - af) > offset_hz)
        {
            _health_count = 0;
            g_status[_name]["status"] = GlobalConfigStatus::exception;
            g_status[_name]["msg"]["topics"][topic_name] = GlobalConfigStatus::hz_wrong;
        }else{
            g_status[_name]["msg"]["topics"][topic_name] = GlobalConfigStatus::normal;
        }
        g_lock_status.unlock();
    }

    void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& n){
        _topics_cnt[n]++;
    }

};

class CANIPC : public CheckBasic
{
private:
    ros::Subscriber _sub;
public:
    using CheckBasic::CheckBasic;

    void initSub(ros::NodeHandle& nh){
        _sub = nh.subscribe("/chassis", 1, &CANIPC::chassisCallback, this);
    }

    void chassisCallback(const ChassisConstPtr& msg){
        static double lastTime = ros::WallTime::now().toSec();
        double curTime = ros::WallTime::now().toSec();
        if(curTime - lastTime < 2.0)   return;
        lastTime = curTime;
        
        std::lock_guard<std::mutex> lk(g_lock_status);
        if(msg->driving_mode == msg->VCU_EMERGENCY_STOP){
            _health_count = 0;
            g_status[_name]["status"] = GlobalConfigStatus::exception;
            g_status[_name]["msg"]["topics"]["chassis"] = "vcu emergency stop";
            return;
        }else if(msg->driving_mode == msg->IPC_EMERGENCY){
            _health_count = 0;
            g_status[_name]["status"] = GlobalConfigStatus::exception;
            g_status[_name]["msg"]["topics"]["chassis"] = "vcu emergency stop";
            return;
        }

        for(const auto& e : msg->error_codes){
            if(e.error_priority > e.ERROR_PRIORITY_WARNING){
                _health_count = 0;
                g_status[_name]["status"] = GlobalConfigStatus::exception;
                g_status[_name]["msg"]["topics"]["chassis"] = e.reason;
                return;
            }
        }

        g_status[_name]["msg"]["topics"]["chassis"] = GlobalConfigStatus::normal;    
    }
};

int main(int argc, char *argv[])
{
#ifndef LOG_CONSOLE
    lg.initLogFile("/var/log/jtcx/robot-manager", "basic_check.log");
#endif
    ros::init(argc, argv, "basic_check_node");
    ros::NodeHandle nh;
    auto status_pub = nh.advertise<std_msgs::String>("/robot_manager/system/status", 1);
    initConfig(argv[1]);
    LG->info("init basic check: {}", g_status.dump());

    std::vector<std::unique_ptr<CheckBasic>> cbv;
    for(auto& e : rule.items()){
        if(e.key() == "CAN"){
            auto p = std::make_unique<CANIPC>(e.key(), e.value());
            p->initRosReleated(nh);
            p->initSub(nh);
            cbv.emplace_back(std::move(p));
            continue;
        }
        
        auto p = std::make_unique<CheckBasic>(e.key(), e.value());
        p->initRosReleated(nh);
        cbv.emplace_back(std::move(p));
    }

    ros::WallTimerCallback statusCallBack = [&](const ros::WallTimerEvent& e) -> void{
        std_msgs::String msg;
        msg.data = g_status.dump();
        status_pub.publish(msg);
    };
    auto status_pub_timer = nh.createWallTimer(ros::WallDuration(1.), statusCallBack);

    ros::MultiThreadedSpinner spinner(8);
    spinner.spin();

    return 0;
}