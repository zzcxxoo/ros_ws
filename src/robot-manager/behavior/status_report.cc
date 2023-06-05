#include "status_report.h"

StatusReport::StatusReport(){
    SLG->trace("status report is constructed!!");
    _chassis_sub = _nh.subscribe("/chassis", 1, &StatusReport::chassisHandler, this);
    _gps_sub = _nh.subscribe("/novatel718d/pos", 1, &StatusReport::gpsHandler, this);

    _props_srv = _nh.advertiseService("/ui/props", &StatusReport::getProps, this);

    _loc_ptr.reset(new LocStatus(_nh, "/ui/localization_lost", 1.0, 2));
    _pp_ptr.reset(new PureStatus(_nh, "/pure_pursuit/status", 1.0, 2));
}

bool StatusReport::getProps(AgentRequest& req, AgentResponse& res){
    res.trace_id = req.trace_id;
    agentHelper(jlw_for_status_report->logger, res, "ok", 0);

    if(req.service_id == "status"){
        res.result = getStatus();
        SLG->info("status: {}", res.result);
    }else if(req.service_id == "position"){
        res.result = getPos();
        SLG->info("pos: {}", res.result);
    }else{
        agentHelper(jlw_for_status_report->logger, res, "invalid service id", -1);
    }
    return true;
}

std::string StatusReport::getPos(){
    std::vector<float> p;
    json res;
    res["map_id"] = getSelectedMap();
    if(ros::param::get("/robot_pose", p)){
        float yaw = 2 * std::atan2(p[5], p[6]);
        res["pos"] = std::vector<float>{p[0], p[1], yaw};
    }else{
        res["pos"] = std::vector<float>{0, 0, 0};
    }

    return res.dump();
}

void StatusReport::chassisHandler(const ChassisConstPtr& ptr)
{

    _pps.battery = std::min((int)ptr->bms.battery_soc_percentage, 100);
    _pps.working_status = "";
    if(ptr->bms.charger_connected == 1) _pps.working_status = status::CHARGE;
    if(ptr->driving_mode == ptr->VCU_EMERGENCY_STOP && ptr->vcu.emergency_button == ptr->vcu.ON)
        _pps.working_status = status::UPGRADE;
    
    if(_pps.working_status == "")   _pps.working_status = _pp_ptr->getStatus();

    json j = _pps;
    auto msg = fmt::format("props: {}", j.dump());
    LOG_WITH_GAP(jlw_for_status_report->logger, trace, 5.0, msg);
}

std::string StatusReport::getStatus()
{
    json res;
    int tmp;

    _pps.gps = json::object();
    cropNum(_pps.battery, 0, 100);

    if(!ros::param::get("/param/battery_threshold", tmp))   tmp = 0;
    _pps.battery_threshold = tmp;

    if(!ros::param::get("/param/voice", tmp))   tmp = 70;
    _pps.sound = tmp;

    std::string stmp;
    if(!ros::param::get("/auto_brush/rubtype", stmp))   stmp = "small";
    _pps.rubber = stmp;

    _pps.location_status = _loc_ptr->getStatus();

    res = _pps;
    return res.dump();
}

void StatusReport::gpsHandler(const sensor_msgs::NavSatFixConstPtr& ptr)
{
    json gps;
    gps["status"] = (int)(ptr->status.status);
    gps["latitude"] = ptr->latitude;
    gps["longitude"] = ptr->longitude;
    _pps.gps = gps.dump();

    LOG_WITH_GAP(jlw_for_status_report->logger, trace, 5.0, "gps: " + gps.dump());
}

    