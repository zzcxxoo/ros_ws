// ros msg header
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>

// mobile_platform_msgs header
#include "mobile_platform_msgs/LocalizationLost.h"
#include "mobile_platform_msgs/Pursuit.h"
#include "mobile_platform_msgs/PurePursuitStatus.h"
#include "mobile_platform_msgs/PurePursuitResult.h"
#include "mobile_platform_msgs/Chassis.h"
#include "mobile_platform_msgs/MappingSave.h"
#include "mobile_platform_msgs/TargetPoint.h"
#include "mobile_platform_msgs/Agent.h"
#include "mobile_platform_msgs/HomeToDock.h"
#include "mobile_platform_msgs/BcmCommand.h"

// std
#include <vector>
#include <queue>
#include <ctime>
#include <chrono>
#include <map>

// io/log
#include <fstream>
#include <boost/filesystem.hpp>
#include "nlohmann/json.hpp"
#include "JtcxLogWrapper.hpp"

// sys
#include <signal.h>
#include <uuid/uuid.h>

using json = nlohmann::json;
using namespace std::chrono_literals;
using namespace mobile_platform_msgs;

// log
static JtcxLogWrapper logger("RobotCleaningState", LOG_LEVEL::DEBUG);
static const std::string _save_log_dir = "/var/log/jtcx/robot-manager";
#define LG logger.logger

static const std::string _save_context_file = std::string(std::getenv("HOME")) + "/.robot/data/cleaning/context.json";

static const size_t _max_record_num = 100;
const float _path_gap_dist{0.3};
const float _car_width{1.2};

// global variables
static std::string g_timing_task_id;    // use for tell who call /ui/cleaning/control, if not empty, then timing task call

// msg coming from tracking result
namespace cleaningException{
    static const std::string SUCCESS = "Success";
    
// --------------- pursuit error ---------------------
    static const std::string INTERRUPT = "Manual_Stop";
    static const std::string FAIL = "Fail";

// --------------- unusally reasons
    static const std::string NODE_EXIT = "Node_Exit";
    static const std::string PP_EXIT = "Pure_Pursuit_Exit";
    static const std::string UNKNOWN = "Unkown";
}

template<class T>
static void serviceHelper(T& res, std::string msg, int code){
    if(code == 0){
        LG->info("service success: {}", msg);
    }else{
        LG->error("service error: {}", msg);
    }
    res.msg = msg;
    res.code = code;
}

template<class T>
static void clearVector(std::vector<T>& v){
    std::vector<T> tmp;
    v.swap(tmp);
}

template<class T>
static void addHeader(T& msg){
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
}

static std::string genReportId() noexcept
{
    auto now = std::time(0);
    tm *ltm = localtime(&now);
    char cr[64];
    std::sprintf(cr, "task%02d%02d%02dx%02d%02d%02d", -100 + ltm->tm_year, 1 + ltm->tm_mon,
                ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);
    return std::string(cr);
}

static std::string genUUID() noexcept
{
    uuid_t uuid;
    char str[36];

    uuid_generate(uuid);
    uuid_unparse(uuid, str);

    return std::string(str);
}

static std::string getMapDir(std::string map) noexcept
{
    return std::string(std::getenv("HOME")) + "/.robot/data/maps/" + map + "/";
}

geometry_msgs::Pose getPoseFromTwoPose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
	auto x = p2.position.x - p1.position.x;
	auto y = p2.position.y - p1.position.y;

	geometry_msgs::Quaternion q;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(std::atan2(y, x)), q);

	geometry_msgs::Pose po;
	po = p1;
	po.orientation = q;

	return po;
}

bool callAgent(std::string topic, std::string srv_id, const json& msg, std::string& res)
{
    Agent srv;
    srv.request.trace_id = genUUID();
    srv.request.service_id = srv_id;
    srv.request.data = msg.dump();
    LG->trace("req: {}", srv.request.data);
    if(ros::service::call(topic, srv)){
        if(srv.response.code == 0){
            res = srv.response.result;
            return true;
        }
        LG->error("agent call fail for: {}", srv.response.msg);
    }else
        LG->error("agent offline!!");
    
    return false;
}


struct cleaningContext{
    std::string report_id;
    std::string map;    
    std::string plan_name;    
    std::string tasks;
    int task_id;
    int req_id;
    int time_id;
    std::string mode;
    std::string exception;
    double area;
    double start_time;
    double cleaning_time;
    double stop_time;
    double average_velocity;
    double odometry;
    double process;

    void resetContext(){
        report_id           = "";
        map                 = "";
        plan_name           = "";
        tasks               = "";
        task_id             = 0;
        req_id              = 0;
        time_id             = 0;          

        mode                = "";
        exception           = "";
        area                = 0;
        start_time          = 0;
        cleaning_time       = 0;
        stop_time           = 0;
        average_velocity    = 0;
        odometry            = 0;
        process             = 0;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(cleaningContext, report_id, map, plan_name, tasks, task_id, req_id,
    time_id, mode, exception, area, start_time, cleaning_time, stop_time, average_velocity,
    odometry, process);

    json toCleaningReport(){
        json j;
        j["report_id"] = genReportId();
        j["area"] = area;
        j["average_velocity"] = average_velocity;
        j["cleaning_time"] = cleaning_time;
        j["mode"] = mode;
        j["exception"] = exception;
        j["map_id"] = map;
        j["plan_id"] = plan_name;
        j["task_id"] = g_timing_task_id;
        j["progress"] = process;
        j["start_time"] = start_time;
        return j;
    }

    void sendReport(){
        std::string tmp;
        callAgent("/agent/report", "cleaning_record", toCleaningReport(), tmp);
    }
};

struct PathInfo
{
    int path_size{-1};
    geometry_msgs::Pose start, end;
    std::string name;
    std::string abs_path;

    static geometry_msgs::Pose lineToPose(const std::string& l){
        std::stringstream ss(l);
        geometry_msgs::Pose p;
        ss >> p.position.x      >> p.position.y
           >> p.orientation.x   >> p.orientation.y 
           >> p.orientation.z   >> p.orientation.w;
        p.position.z = 0;
        return p;
    }

    void operator=(const PathInfo& pi){
        path_size = pi.path_size;
        start = pi.start;
        end = pi.end;
        name = pi.name;
    }

    bool parse(std::string path){
        std::ifstream inf(path);
        if(!inf.is_open()){
            LG->error("parse path fail: {}", path);
            return false;
        }
        abs_path = path;
        // name = <zone_name>_<name>
        auto zone_name = bfs::path(path).parent_path().stem().string();
        name = zone_name + "_" + bfs::path(path).stem().string();
        LG->info("path info name: {}", name);

        std::string line;
        std::vector<std::string> lines;
        while(std::getline(inf, line)){
            lines.emplace_back(std::move(line));
        }
        inf.close();
        
        path_size = lines.size();
        if(path_size < 1.0 / 0.2){
            LG->error("path size too small");
            return false;
        }

        auto p1 = lineToPose(lines[0]);
        auto p2 = lineToPose(lines[1]);
        start = getPoseFromTwoPose(p1, p2);
        
        p1 = lineToPose(lines[path_size-2]);
        p2 = lineToPose(lines[path_size-1]);
        end = getPoseFromTwoPose(p1, p2);
        
        return true;
    }

    static std::vector<geometry_msgs::PoseStamped> parseCSVToVec(std::string path)
    {
        std::vector<geometry_msgs::PoseStamped> psv;
        std::ifstream inf(path);
        std::string line;
        while(std::getline(inf, line)){
            geometry_msgs::PoseStamped psd;
            addHeader(psd);
            auto p = lineToPose(line);
            psd.pose = p;
            psv.emplace_back(std::move(psd));
        }
        inf.close();
        return psv;
    }

    static bool joinPath(std::string map_name, std::string zone_dir, const PathInfo& from, const PathInfo& to)
    {
        std::string fn = zone_dir + (from.name == to.name ? from.name + "_loop.csv" : fmt::format("from_{}_to_{}.csv", from.name, to.name));

        float x1, y1, x2, y2;
        x1 = from.end.position.x;
        y1 = from.end.position.y;
        x2 = to.start.position.x;
        y2 = to.start.position.y;

        if(std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) > 1.0){
            TargetPoint srv;
            srv.request.map_name = map_name;
            geometry_msgs::PoseStamped ps;
            ps.pose = from.end;
            srv.request.first_path.poses.push_back(ps);
            ps.pose = to.start;
            srv.request.second_path.poses.push_back(ps);
            
            if(ros::service::call("/target_point_planner", srv))
            {
                if(srv.response.status == 0){
                    const auto& p = srv.response.planner_path;
                    std::ofstream out(fn);
                    LG->info("create join path: {}", fn);
                    if(out.is_open()){
                        for(const auto& e : p.poses){
                            auto line = fmt::format("{} {} {} {} {} {}", e.pose.position.x, e.pose.position.y,
                                        e.pose.orientation.x, e.pose.orientation.y, e.pose.orientation.z, e.pose.orientation.w);
                            out << line << std::endl;
                        }
                        out.close();
                        return true;          
                    }
                }else{
                    LG->error("call a start error code({}), msg({})", srv.response.status, srv.response.message);
                }
            }
            return false;
        }
        return true;
    }

    static void pathToFile(const nav_msgs::Path& path, std::string fn)
    {
        std::ofstream out(fn);
        for(auto const& pst : path.poses){
            auto const& p = pst.pose;
            auto line = fmt::format("{} {} {} {} {} {}", p.position.x, p.position.y,
                        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
            out << line << std::endl;
        }
        out.close();
    }

    // ------------- for vis ---------------
    static ros::Publisher* _vis_start_end_pub;
    static ros::Publisher* _vis_path_pub;

    void visPubStartAndEnd(){
        if(_vis_start_end_pub->getNumSubscribers() > 0){
            geometry_msgs::PoseArray pa;
            pa.header.frame_id = "map";
            pa.header.stamp = ros::Time::now();
            pa.poses.push_back(start);
            pa.poses.push_back(end);
            _vis_start_end_pub->publish(pa);
        }
    }

    void visPubPath(){
        if(_vis_path_pub->getNumSubscribers() > 0){
            nav_msgs::Path p;
            addHeader(p);
            p.poses = std::move(parseCSVToVec(abs_path));
            _vis_path_pub->publish(p);
        }
    }
};
ros::Publisher* PathInfo::_vis_start_end_pub;
ros::Publisher* PathInfo::_vis_path_pub;

/**
 * @brief TaskMsg的初始化，先传入地图初步构造，再从json完全构造 
 * 
 */
struct TaskMsg
{
    std::string _zone_name;
    int _times;
    std::string _mode;
    std::string _rubtype;
    
    std::string _map;
    std::string _zone_dir;

    TaskMsg(std::string map){
        _map = map;
        _zone_dir = getMapDir(map) + "zone/";
    }

    void from_json(const json& j){
        _zone_name = j["zone_id"];
        _mode = j["mode"];
        _rubtype = j["rubtype"];
        _times = j["times"].get<int>();
    }

    PursuitRequest to_req(){
        PursuitRequest req;
        req.command = 1;
        req.map = _map;
        req.path_name = getCSV();
        req.mode = "path_tracking_with_path_name";
        return req;
    }

    static PursuitRequest to_req(std::string map, std::string path){
        PursuitRequest req;
        req.command = 1;
        req.map = map;
        req.path_name = path;
        req.mode = "path_tracking_with_path_name";
        return req;
    }

    std::string getSpecificZoneDir() noexcept {return _zone_dir + _zone_name + "/";}
    std::string getCSV() noexcept {return getSpecificZoneDir() + _mode + ".csv";}
};

static std::mutex task_lk;
static std::condition_variable task_cv;
class Task
{
private:

    std::vector<PursuitRequest> _reqs;
    std::vector<PathInfo> _pis;

    std::vector<float> _weight;
    int _path_size{0};

    int _times;
    int _req_idx{0};
    int _time_idx{0};

    bool _req_done;
    int _why_exit;
    
    std::string _rubtype;
    bool _use_brush{false};

public:
    // exit code
    static const int TASK_DONE = 0;
    static const int TASK_STOP = -1;
    static const int TASK_FAIL = -2;
    static const int TASK_OFFLINE = -3;
    static std::map<int, std::string> CODE_TO_STRING;

    static std::map<int, std::string> initCodeToStringMap()
    {
        std::map<int, std::string> ret{
            {TASK_DONE, "Normal"},
            {TASK_STOP, "Interrupt"},
            {TASK_FAIL, "Error"},
            {TASK_OFFLINE, "Error"}
        };
        return ret;
    }

    static ros::Publisher* _brush_control_ptr;

    // ---------- get req info ------------
    bool  getReqDone()    {return _req_done;}
    int   getReqIdx()     {return _req_idx;}
    const std::vector<PursuitRequest>& getReqsVec() const {return _reqs;}

    int getPathSize()           const  {return _path_size;}
    int getTimes()              const  {return _times;}
    float getReqWeight()        const  {return _weight[_req_idx];}
    float getReqWeight(int id)  const  {return _weight[id];}
    void setUseBrush()                 {_use_brush = true;}

    void notifyDone(){
        std::unique_lock<std::mutex> lk(task_lk);
        _req_done = true;
        task_cv.notify_all();
    }

    void exit(int code){
        setExitCode(code);
        notifyDone();
    }

    void setTimes(int t){_times = t;}
    void setExitCode(int e){_why_exit = e;}
    void setRubType(const std::string& t){_rubtype = t;}

    void sendRubCmd(){
        if(_rubtype.empty())    return;
        MappingSave ms;
        ms.request.filename = _rubtype;
        ros::service::call("/auto_brush/set_cleaning_type", ms);
        auto tmp = ms.response;
        LG->info("call cleaning type: status({}), message({})", tmp.status, tmp.message);        
    }

    void sendBrushCmd(int flag){
        BcmCommand bcm;
        bcm.auto_clean = flag;
        _brush_control_ptr->publish(bcm);
    }

    // when recover process !!
    void setIdx(int t, int r){
        _time_idx = t;
        _req_idx = r;
    }

    void reset(){
        _time_idx = 0;
        _req_idx = 0;
        _path_size = 0;
        clearVector(_reqs);
        clearVector(_pis);
        clearVector(_weight);

        _use_brush = false;
        _rubtype = "";
    }

    void push_back(const PursuitRequest& req, const PathInfo& pi)
    {
        _reqs.push_back(req);
        _pis.push_back(pi);
        _path_size += pi.path_size;
        _weight.resize(_pis.size());
        for (int i = 0; i < _pis.size(); i++) _weight[i] = 1.0 * _pis[i].path_size / _path_size;
    }

    int exec(){
        int num = _times * _reqs.size();
        int st = _time_idx * _reqs.size() + _req_idx;
        
        // switch rub type
        sendRubCmd();
        if(_use_brush)  sendBrushCmd(1);

        for(int i=st; i<num; i++)
        {
            _req_idx = i % _reqs.size();
            _time_idx = i / _reqs.size();

            LG->info("------- executing time_idx({}) req_idx({}) ---------", _time_idx, _req_idx);

            Pursuit srv;
            srv.request = _reqs[_req_idx];
            _req_done = false;
            
            LG->trace("req map_name: {}", srv.request.map);
            LG->trace("req path_name: {}", srv.request.path_name);
            LG->trace("req cmd: {}", srv.request.command);

            // try to vis
            _pis[_req_idx].visPubStartAndEnd();
            _pis[_req_idx].visPubPath();

            if(ros::service::call("/pursuit", srv))
            {
                if(srv.response.status < 3){
                    // wait done, someone will notify
                    std::unique_lock<std::mutex> lk(task_lk);
                    while(!_req_done)   task_cv.wait(lk);

                    if(_why_exit != TASK_DONE)     break; 

                }else{
                    LG->error("call pursuit fail, code({}), why({})", srv.response.status, srv.response.message);
                    _why_exit = TASK_FAIL;
                    break;
                }
            }else{
                LG->error("cant call pursuit!!");
                _why_exit = TASK_OFFLINE;
                break;
            }
        }

        // only success/fail/stop/offline
        // close brush
       if(_use_brush)   sendBrushCmd(0);
        return _why_exit;
    }

    float getProcess(int waypoint){
        // cal the last process
        float bp = 1.0 * _time_idx / _times;
        
        float mp{0};
        for (int i = 0; i < _req_idx; i++)    mp += _weight[i] / _times;
        
        float lp = 1.0 * waypoint / _path_size / _times;

        LG->trace("get process from waypoint({}): {} {} {}", waypoint, bp, mp, lp);
        return bp + mp + lp;       
    }

    void save(cleaningContext& cont){
        cont.req_id = _req_idx;
        cont.time_id = _time_idx;
    }

};
std::map<int, std::string> Task::CODE_TO_STRING = Task::initCodeToStringMap();
ros::Publisher* Task::_brush_control_ptr;

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

    // run thread flag
    std::atomic_bool _run_exit_flag{true};

    // task related
    std::vector<Task> _task_list;
    std::vector<float> _task_weight;
    double _total_odometry{0};

    int _cur_task_idx{-1};

    cleaningContext _context;

    std::deque<float> _velocity;

    float _last_process{0.0};
    bool _load_flag;

    std::vector<std::string> _mode_list{
        "VCU_EMERGENCY_STOP", "HUMAN_MANUAL", "VCU_MANUAL", "IPC_EMERGENCY",
        "IPC_MANUAL",         "IPC_AUTO",     "IPC_CLIENT"
    };

    std::unique_ptr<std::thread> _cleaning_thread_ptr;

    std::thread _check_pursuit_health;
    std::atomic_bool _check_pursuit_flag{true};

public:

    RobotCleaningState(){
        LG->info("construct obj!!");
        // create dir
        auto task_file = bfs::path(_save_context_file);
        if(!bfs::exists(task_file.parent_path()))   bfs::create_directories(task_file.parent_path());  

        // init ros related
        _rcs_srv = _nh.advertiseService("/ui/cleaning/control", &RobotCleaningState::cleaningHandler, this);
        
        // _chassis_sub = _nh.subscribe("/chassis", 1, &RobotCleaningState::chassisHandler, this);
        _status_sub = _nh.subscribe("/pure_pursuit/status", 1, &RobotCleaningState::cleaningStatusHandler, this);
        _result_sub = _nh.subscribe("/pure_pursuit/tracking_result", 10, &RobotCleaningState::cleaningResultHandler, this, ros::TransportHints().tcpNoDelay());
        _twist_sub = _nh.subscribe("/nav/cmd_vel", 1, &RobotCleaningState::twistHandler, this);
        
        _check_pursuit_health = std::thread(&RobotCleaningState::checkPursuitAlive, this);
        _cleaning_thread_ptr = std::unique_ptr<std::thread>(new std::thread());

        signal(SIGINT, &RobotCleaningState::stopTrackingAndExit);
        signal(SIGTERM, &RobotCleaningState::stopTrackingAndExit);
    }

    ~RobotCleaningState(){
        // cleaning run thread
        if(!_run_exit_flag){
            stopCleaningTask(cleaningException::NODE_EXIT);
        }
        // set flag in advance to make sure thread exit!!
        if(_cleaning_thread_ptr->joinable())    _cleaning_thread_ptr->join();

        _check_pursuit_flag = false;
        _check_pursuit_health.join();
        LG->info("exit obj!!");
    }

    void stopCleaningTask(std::string cleaning_exception)
    {
        PurePursuitResult ppr;
        ppr.tracking_result = cleaning_exception;
        cleaningResultHandler(boost::make_shared<PurePursuitResult>(ppr));
    }

    static void stopTracking(){
        Pursuit srv;
        srv.request.command = 2;    
        ros::service::call("/pursuit", srv);
        LG->trace("stop call code {}, message {}", srv.response.status, srv.response.message);
    }

    static void stopTrackingAndExit(int sig){
        LG->info("stop tracking and exit!!");
        stopTracking();
        ros::shutdown();
    }

    int open_popen(const std::string &cmd, std::vector<std::string> &out) {
        FILE *fp;
        const int sizebuf = 2048;
        char buff[sizebuf];
        out = std::vector<std::string>();
        fp = popen(cmd.c_str(), "r");
        if (!fp)    LG->error("Couldn't start command");

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
                LG->warn("node /pure_pursuit is died!!");
                if(!_run_exit_flag){
                    stopCleaningTask(cleaningException::PP_EXIT);
                }
            }
            std::this_thread::sleep_for(0.5s);
        }  
    }

    bool cleaningHandler(AgentRequest& req, AgentResponse& res)
    {   
        // 0. parse req
        json j = json::parse(req.data);
        std::string type = j["type"];

        res.trace_id = req.trace_id;

        if(type == "pause" || type == "resume"){
            Pursuit srv;
            srv.request.command = (type == "pause" ? 3 : 4);
            if(ros::service::call("/pursuit", srv)){
                LG->info("pause or resume: code: {} || msg: {}", srv.response.status, srv.response.message);
            }else{
                LG->error("pursuit offline!!");
            }
            return true;
        }else if(type == "stop"){
            stopTracking();
            res.code = 0;
            res.msg = "ok";
            return true;
        }

        if(type != "start"){
            std::string msg = "invalid type!!";
            res.code = -1;
            res.msg = msg;
            LG->warn(msg);
            return true;
        }
        
        // 1. check wether it's cleaning now ??
        if(!_run_exit_flag){
            std::string msg = "cleaning task thread does not finished completely!!";
            res.code = -1;
            res.msg = msg;
            LG->warn(msg);
            return true;
        }

        // 2. parse request
        if(!loadContext(j["content"])){
            serviceHelper(res, "file may be not exsit or something wrong!", -1);
            return true;
        }
        
        std::string msg;
        if(constructTasks()){
            msg = "construct task success!";
            serviceHelper(res, msg, 0);
            
            if(_cleaning_thread_ptr->joinable())    _cleaning_thread_ptr->join();
            _cleaning_thread_ptr.reset(new std::thread(&RobotCleaningState::run, this));
        }else{
            msg = "construct task fail!";
            serviceHelper(res, msg, 1);
        }
        return true;
    }

    bool constructTasks()
    {
        LG->trace("------------- step in construct tasks --------------");
        clearVector(_task_list);
        std::vector<json> tasks = json::parse(_context.tasks).get<std::vector<json>>();

        // 1. prepare from_charging
        // join from_charing with first task
        TaskMsg first_task(_context.map);
        first_task.from_json(tasks.front());
        
        PathInfo pi_from, pi_to, pi_join;
        std::string from_charing_path = first_task.getSpecificZoneDir() + "from_charging.csv";
        pi_from.parse(from_charing_path);
        pi_to.parse(first_task.getCSV());

        if(!PathInfo::joinPath(_context.map, first_task.getSpecificZoneDir(), pi_from, pi_to)){
            LG->error("join first path fail!!");
            return false;
        }

        // 2. make the first task: from_charging + join_path (2 req)
        // pathinfo + req = task
        // !! remember to set task times
        Task tk;
        tk.setTimes(1);
        auto req = TaskMsg::to_req(_context.map, from_charing_path);
        tk.push_back(req, pi_from);
        
        std::string join_path = first_task.getSpecificZoneDir() + fmt::format("from_{}_to_{}.csv", pi_from.name, pi_to.name);
        if(bfs::exists(bfs::path(join_path))){
            pi_join.parse(join_path);
            req = TaskMsg::to_req(_context.map, join_path);
            tk.push_back(req, pi_join);
        }

        _task_list.emplace_back(tk);
        tk.reset();

        // 3. traverse all tasks
        // 3.1 if times > 1, firstly make loop and exec times-1, the last time just exec task itself

        for(int i=0; i<tasks.size(); i++){
            auto const& t = tasks[i];
            TaskMsg tm(_context.map);
            tm.from_json(t);
            PathInfo pi;
            pi.parse(tm.getCSV());

            if(tm._times > 1){
                
                tk.setTimes(tm._times - 1);
                tk.push_back(tm.to_req(), pi);
                tk.setRubType(tm._rubtype);
                tk.setUseBrush();
                
                // make loop
                if(!PathInfo::joinPath(_context.map, tm.getSpecificZoneDir(), pi, pi)){
                    LG->error("join {} fail!!", pi.name);
                    return false;
                }
                
                // check exist
                join_path = tm.getSpecificZoneDir() + fmt::format("{}_loop.csv", pi.name);
                if(bfs::exists(bfs::path(join_path))){
                    pi_join.parse(join_path);
                    req = TaskMsg::to_req(_context.map, join_path);
                    tk.push_back(req, pi_join);
                }
                _task_list.emplace_back(tk);
                tk.reset();
            }
            
            // push the last one and maybe join the path with the next task
            tk.setTimes(1);
            tk.setRubType(tm._rubtype);
            tk.setUseBrush();
            tk.push_back(tm.to_req(), pi);

            if(i+1 < tasks.size()){

                TaskMsg tm_next(_context.map);
                tm_next.from_json(tasks[i+1]);
                PathInfo pi_next;
                pi_next.parse(tm_next.getCSV());
                // make join
                if(!PathInfo::joinPath(_context.map, tm.getSpecificZoneDir(), pi, pi_next)){
                    LG->error("join {} with {} fail!!", pi.name, pi_next.name);
                    return false;
                }
                // check exist
                join_path = tm.getSpecificZoneDir() + fmt::format("from_{}_to_{}.csv", pi.name, pi_next.name);
                if(bfs::exists(bfs::path(join_path))){
                    pi_join.parse(join_path);
                    req = TaskMsg::to_req(_context.map, join_path);
                    tk.push_back(req, pi_join);
                }
            }

            _task_list.emplace_back(tk);
            tk.reset();  
        }

        // 4. join last task with to_charging
        first_task.from_json(tasks.back());
        pi_from.parse(first_task.getCSV());

        // generate to_charging by home to dock
        std::string to_charing_path = first_task.getSpecificZoneDir() + "to_charging.csv";
        HomeToDock htd;
        htd.request.start.pose.pose = pi_from.end;
        if(ros::service::call("/auto_dock/home_to_dock", htd)){
            if(htd.response.path.poses.size() < 1.0 / 0.2){
                LG->error("auto docking get too little path poses!!");
                return false;
            }

            // write to to_charging.csv
            PathInfo::pathToFile(htd.response.path, to_charing_path);
            pi_to.parse(to_charing_path);
            
        }else{
            LG->error("auto docking offline!!");
            return false;
        }   

        // std::string to_charing_path = first_task.getSpecificZoneDir() + "to_charging.csv";
        // PathInfo::pathToFile(htd.response.path, to_charing_path);
        
        // if(!PathInfo::joinPath(_context.map, first_task.getSpecificZoneDir(), pi_from, pi_to)){
        //     LG->error("join last path fail!!");
        //     return false;
        // }

        // join_path = first_task.getSpecificZoneDir() + fmt::format("from_{}_to_{}.csv", pi_from.name, pi_to.name);
        // if(bfs::exists(bfs::path(join_path))){
        //     pi_join.parse(join_path);
        //     req = TaskMsg::to_req(_context.map, join_path);
        //     tk.push_back(req, pi_join);
        // }

        tk.push_back(TaskMsg::to_req(_context.map, to_charing_path), pi_to);
        tk.setTimes(1);
        _task_list.emplace_back(tk);

        // init task weight
        int task_num = _task_list.size();
        _task_weight.resize(task_num);
        size_t path_size{0};
        for (int i = 0; i < task_num; i++)  path_size += _task_list[i].getPathSize() * _task_list[i].getTimes();
        for (int i = 0; i < task_num; i++)  _task_weight[i] = 1.0 * _task_list[i].getPathSize() * _task_list[i].getTimes() / path_size;

        LG->info("total path size: {}", path_size);
        LG->info("task weight: {}", _task_weight);

        // cal total odometry
        _total_odometry = 0;
        for (size_t i = 0; i < _task_list.size(); i++)
        {
            const auto& task = _task_list[i];
            _total_odometry += _path_gap_dist * task.getTimes() * task.getPathSize();
        }
        LG->info("cal total odometry {} m", _total_odometry);
        
        // print all tasks
        for(int i=0; i<_task_list.size(); i++){
            const auto& reqs = _task_list[i].getReqsVec();
            for(const auto& r : reqs)
                LG->info("---- task {}: {} ----", i, r.path_name);
        }

        return true;            
    }

    void run(){
        _run_exit_flag = false;

        LG->info("start running cleaning task!!");
        LG->trace("task list num: {}", _task_list.size());

        _task_list[_context.task_id].setIdx(_context.time_id, _context.req_id);
        int res;
        for(int i=_context.task_id; i<_task_list.size(); i++){
            _cur_task_idx = i;
            LG->info("----------- executing task {} -----------", i);
            
            // cal last process
            _last_process = 0;
            for (int j = 0; j < i; j++)   _last_process += _task_weight[j];
            LG->trace("----------- last_process {}-----------", _last_process);

            res = _task_list[i].exec();
            LG->trace("cv is notified!!");

            if(res < Task::TASK_DONE){
                LG->error("error happen while executing task {}: code({})", i, res);
                break;
            }
        }

        if(res == Task::TASK_DONE){
            LG->info("all tasks are finished!!");
            _context.process = 1.0;
        }
        _context.exception = Task::CODE_TO_STRING[res];
        saveContextToFile();
        LG->info("exit run thread");
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
            logger.logger->error(msg);
            return;
        }
        
        _context.task_id = _cur_task_idx;
        _task_list[_cur_task_idx].save(_context);
        _context.stop_time = ros::WallTime::now().toSec();
        _context.cleaning_time = _context.stop_time - _context.start_time;

        _context.sendReport();
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

    // if from req, the cleaning should be new one!!
    bool loadContext(const json& j)
    {
        LG->trace("---------- step in load context ----------");
        LG->trace("get req: {}", j.dump());
        std::string mode = "new";
        if(j.contains("mode"))  mode = j["mode"];
        // only if timing task coming!!
        g_timing_task_id = "";
        if(j.contains("task_id"))   g_timing_task_id = j["task_id"];   

        if(mode == "new"){
            _context.resetContext();
            _context.report_id = genReportId();
            _context.map = j["map_id"];
            _context.plan_name = j["plan_id"];
            _context.tasks = j["zones"].dump();
            _context.start_time = ros::WallTime::now().toSec();
            _load_flag = false;
            _last_process = 0;
            return true;
        }

        // only if file exist then load
        if(bfs::exists(bfs::path(_save_context_file))){
            std::ifstream inf(_save_context_file);
            json j = json::parse(inf);
            inf.close();
            std::vector<cleaningContext> cc_vec = j.get<std::vector<cleaningContext>>();
            if(!cc_vec.empty())     _context = cc_vec.back();
            else{
                LG->error("context file is empty or invalid!!");
                return false;
            }

            if(_context.exception == Task::CODE_TO_STRING[static_cast<int>(Task::TASK_DONE)]){
                LG->warn("last context is a normal one, no need to reload this context!!");
                return false;
            }

            _load_flag = true;
            LG->info("context is loaded with task: {}", _context.tasks);

            return true;
        }
        return false;
    }

    // void chassisHandler(const mobile_platform_msgs::ChassisConstPtr& msg){}

    /**
     * @brief : upload to phone 
     * 
     */
    void uploadStatus()
    {
        // limit freq
        static double last = ros::WallTime::now().toSec();
        if(ros::WallTime::now().toSec() - last < 2.0)   return;

        last = ros::WallTime::now().toSec();
        json jv;
        jv["map_id"] = _context.map;
        jv["plan_id"] = _context.plan_name;
        jv["task_id"] = g_timing_task_id;
        // jv["current_times"] = int(cur_process * tmp) + 1;
        jv["progress"] = _context.process;
        jv["area"] = _context.area;
        jv["odom"] = _context.odometry;
        jv["duration"] = ros::WallTime::now().toSec() - _context.start_time;

        std::string tmp;
        callAgent("/agent/props", "cleaning_progress", jv, tmp);
    }

    void cleaningStatusHandler(const mobile_platform_msgs::PurePursuitStatusConstPtr& msg){
        if(msg->state == "WORKING" && !_run_exit_flag){
            auto& task = _task_list[_cur_task_idx];
            
            _context.process = _last_process + _task_weight[_cur_task_idx] * task.getProcess(msg->waypoint);
            _context.process = std::max(0.0, std::min(1.0, _context.process));

            LG->trace("process: {}", _context.process);
            // info related process
            _context.odometry = _total_odometry * _context.process;
            _context.area = _car_width * _context.odometry;

            uploadStatus();
        }
    }
    
    void cleaningResultHandler(const PurePursuitResultConstPtr& msg){
        if(!_run_exit_flag){
            LG->info("get cleaning reason: {}", msg->tracking_result);
            if(msg->tracking_result == cleaningException::SUCCESS){
                _task_list[_cur_task_idx].exit(Task::TASK_DONE);

            }else if(msg->tracking_result == cleaningException::INTERRUPT){
                _task_list[_cur_task_idx].exit(Task::TASK_STOP);

            }else if(msg->tracking_result == cleaningException::NODE_EXIT || msg->tracking_result == cleaningException::PP_EXIT
                    || msg->tracking_result == cleaningException::UNKNOWN || msg->tracking_result == cleaningException::FAIL)
            {
                _task_list[_cur_task_idx].exit(Task::TASK_FAIL);              
            }
        }
    }

    void twistHandler(const geometry_msgs::TwistConstPtr& msg){
        if(!_run_exit_flag){
            _velocity.push_back(msg->linear.x);
            if(_velocity.size() > 50)   _velocity.pop_front();

            float twistSum = 0;
            for (const auto& e : _velocity)
                twistSum += e;

            _context.average_velocity = twistSum / (_velocity.size() + 1.0);
        }
    }

};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_cleaning_node", ros::InitOption::NoSigintHandler);
    logger.initLogFile(_save_log_dir, "cleaning.log");

    // ---------- for vis ----------
    ros::NodeHandle nh;

    ros::Publisher visPoint = nh.advertise<geometry_msgs::PoseArray>("/cleaning/vis_start_end", 5, true);
    ros::Publisher visPath = nh.advertise<nav_msgs::Path>("/cleaning/vis_path", 5, true);
    ros::Publisher brushControlPub = nh.advertise<BcmCommand>("/auto/clean_cmd", 5);

    PathInfo::_vis_start_end_pub = &visPoint;
    PathInfo::_vis_path_pub = &visPath;
    Task::_brush_control_ptr = &brushControlPub;

    RobotCleaningState rcs;
    ros::spin();
    return 0;
}
