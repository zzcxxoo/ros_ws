/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "waypoint_manager.h"
#include <yaml-cpp/yaml.h>
#include "common/common.h"
#include "mobile_platform_msgs/HomeToDock.h"

WaypointEntry::WaypointEntry(geometry_msgs::Pose pose) { this->pose_ = pose; }

std::shared_ptr<JtcxLogWrapper> WaypointManager::s_logger;
#define WLG WaypointManager::s_logger->logger

void WaypointManager::Init(ros::NodeHandle *nh_ptr) {
  nh_ptr_ = nh_ptr;

  // std::vector<WaypointEntry> waypoint_list;

  current_goal_.target_pose.header.frame_id = "map";
  current_goal_index_ = 0;

  current_goal_nearest_point_.target_pose.header.frame_id = "map";
  retry_time_ = 0;
  cur_speed_ = 0;
  battery_threshold_ = 0;

  // variables for navigation status
  nav_status_.last_coordinate["positon"] = {{"x", 0}, {"y", 0}, {"z", 0}};
  nav_status_.last_coordinate["orientation"] = {
      {"x", 0}, {"y", 0}, {"z", 0}, {"w", 0}};
  nav_status_.cur_coordinate["position"] = {{"x", 0}, {"y", 0}, {"z", 0}};
  nav_status_.cur_coordinate["orientation"] = {
      {"x", 0}, {"y", 0}, {"z", 0}, {"w", 0}};
  // nh_ptr_->setParam("navigation_status",nav_status);

  // Convert tic rate to a ROS rate
  autodocking_started_ = false;
  patrol_current_no_ = 0;
  is_moving_to_waypoint_ = NULL;
  moving_to_nearest_point_ = false;
  arrive_at_nearest_point_ = false;
  state_ = 0;

  // this is to indicate no move base task yet, thus ready to initiate
  last_movebase_result_ = -1;
  last_movebase_status_ = -1;

  move_base_unknown_count_ = 0;

  // maintain sync with nav and mapping status
  navigation_started_ = false;
  mapping_started_ = false;

  UpdateInternalFlags();

  // publish msg to arduino to stop publishing charging voltage info
  pub_start_waypoint_ =
      nh_ptr_->advertise<std_msgs::Int16>("/ui/start_waypoint", 2);
  pub_fail_waypoint_ =
      nh_ptr_->advertise<std_msgs::Int16>("/ui/fail_waypoint", 2);
  pub_success_waypoint_ =
      nh_ptr_->advertise<std_msgs::Int16>("/ui/success_waypoint", 2);
  active_waypoint_ = -1;

  // 消息订阅端
  movebase_result_subscriber_ = nh_ptr_->subscribe(
      "/move_base/result", 1, &WaypointManager::UpdateMovebaseResult, this);
  movebase_status_subscriber_ = nh_ptr_->subscribe(
      "/move_base/status", 1, &WaypointManager::UpdateMovebaseStatus, this);

  chassis_battery_subscriber_ =
      nh_ptr_->subscribe("/chassis", 1, &WaypointManager::UpdateBattery, this);

  // 服务接收端
  clear_costmap_service_ =
      nh_ptr_->serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  autodocking_service_ =
      nh_ptr_->serviceClient<DockingStart>("/autodocking/start");
  path_planning_service_ = nh_ptr_->serviceClient<PathPlanningExecute>(
      "/move_base/self_path_plan/execute");

  // --------------- cleaning ------------------
  path_tracking_client_ = nh_ptr_->serviceClient<Pursuit>("/pursuit");
  ui_path_tracking_ = nh_ptr_->advertiseService(
      "/ui/path_tracking", &WaypointManager::HandlePathTracking, this);

  // ---------------- maybe useless ----------------
  ui_goto_coordinate_ = nh_ptr_->advertiseService(
      "/ui/goto/coordinate", &WaypointManager::HandleGoToCoordinate, this);
  ui_goto_waypoint_ = nh_ptr_->advertiseService(
      "/ui/goto/waypoint", &WaypointManager::HandleGoToWaypoint, this);
  pub_home_to_dock_ =  nh_ptr_->advertise<std_msgs::Bool>("/autodocking/home_to_dock", 1);

  // ------------- charging -------------
  ui_goto_chargingpoint_ = nh_ptr_->advertiseService(
      "/ui/goto/chargingpoint", &WaypointManager::HandleGoToChargingPoint,
      this);
  ui_charging_start_ = nh_ptr_->advertiseService(
      "/ui/charging/start", &WaypointManager::StartCharging, this);
  ui_charging_stop_ = nh_ptr_->advertiseService(
      "/ui/charging/stop", &WaypointManager::StopCharging, this);

  // ------------- recalling ---------------
  ui_recall_ = nh_ptr->advertiseService("/ui/recall", &WaypointManager::RecallHandler, this);

  WLG->info("init waypoint manager!!");
}

void WaypointManager::ClearCostmap() {  // Test (Done)
  WLG->trace("canceling all goals");
  move_base_.cancelAllGoals();
  WLG->trace("canceling all goals FINISHED");
  std_srvs::Empty srv;
  WLG->trace("clearing costmap");
  if (clear_costmap_service_.call(srv)) {
    WLG->trace("clearing costmap FINISHED");

    WLG->trace("connect sucess!!!!!!!");
  } else {
  }
}
void WaypointManager::ResetPatrol() {
  self_path_plan_ = false;
  is_moving_to_waypoint_ = false;
  moving_to_nearest_point_ = false;
  arrive_at_nearest_point_ = false;
  first_time_move_to_nearest_point_ = true;
  state_ = 0;
  retry_time_ = 0;
}
void WaypointManager::SetCurrentGoalPose(const geometry_msgs::Pose &pose,
                                         std_msgs::Int32 index) {
  current_goal_.target_pose.pose = pose;
  current_goal_.target_pose.header.stamp = ros::Time::now();
  current_goal_index_ = index.data;

  WLG->trace("current goal pose set complete ");
}
void WaypointManager::SetCurrentGoalCoordinate(GoToCoordinate::Request &req,
                                               GoToCoordinate::Response &res) {
  current_goal_.target_pose.pose.position.x = req.x;
  current_goal_.target_pose.pose.position.y = req.y;
  geometry_msgs::Quaternion quaternion =
      tf::createQuaternionMsgFromRollPitchYaw(0, 0, req.yaw);
  current_goal_.target_pose.pose.orientation.x = quaternion.x;
  current_goal_.target_pose.pose.orientation.y = quaternion.y;
  current_goal_.target_pose.pose.orientation.z = quaternion.z;
  current_goal_.target_pose.pose.orientation.w = quaternion.w;
  WLG->trace(
      "the curret goal pose position x is {}, the current goal pose x is "
      "{}, the current goal pose w is {}",
      req.x, quaternion.x, quaternion.w);
  res.message = "set Done!";
}
bool WaypointManager::SetCurrentGoalToChargingPoint() {
  if (nh_ptr_->hasParam("/chargingpoints")) {
    XmlRpc::XmlRpcValue ps;
    WLG->trace("get the /chargingpoints ok!");
    nh_ptr_->getParam("/chargingpoints", ps);
    for (auto point = ps.begin(); point != ps.end(); point++) {
      WLG->trace("the info type is {}", point->second.getType());
      WLG->trace("the info size is {}", point->second.size());
      WLG->trace("{}", point->first);

      if (point->first == "orientation") {
        current_goal_.target_pose.pose.orientation.x =
            static_cast<float>(static_cast<int>(point->second["x"]));
        current_goal_.target_pose.pose.orientation.y =
            static_cast<float>(static_cast<int>(point->second["y"]));
        current_goal_.target_pose.pose.orientation.z =
            static_cast<float>(static_cast<int>(point->second["z"]));
        current_goal_.target_pose.pose.orientation.w =
            static_cast<float>(static_cast<int>(point->second["w"]));
      } else if (point->first == "position") {
        current_goal_.target_pose.pose.position.x =
            static_cast<float>(static_cast<int>(point->second["x"]));
        current_goal_.target_pose.pose.position.y =
            static_cast<float>(static_cast<int>(point->second["y"]));
        current_goal_.target_pose.pose.position.z =
            static_cast<float>(static_cast<int>(point->second["z"]));
      }
    }
    WLG->trace("the target_pose position x is {} the target_pose orientation y is {}",
              current_goal_.target_pose.pose.position.x,
              current_goal_.target_pose.pose.position.y);
    return true;
  } else {
    return false;
  }
}
bool WaypointManager::GoToWaypoint(std_msgs::Int32 no) {
  if (no.data >= 0 && no.data <= waypointlist_.size()) {
    ClearCostmap();

    SetCurrentGoalPose(waypointlist_[no.data].pose_, no);
    move_base_.sendGoal(current_goal_);
    WLG->trace("goal sent!");
    pub_start_waypoint_.publish(no);
    active_waypoint_ = no.data;
    return true;
  } else {
    return false;
  }
}
void WaypointManager::GoToCoordinate(GoToCoordinate::Request &req,
                                     GoToCoordinate::Response &res) {
  ClearCostmap();
  SetCurrentGoalCoordinate(req, res);
  move_base_.sendGoal(current_goal_);
  WLG->trace("coordinate sent!");
}
bool WaypointManager::GoToChargingPoint(GoToChargingPoint::Request &req,
                                        GoToChargingPoint::Response &res) {
  ClearCostmap();
  if (SetCurrentGoalToChargingPoint()) {
    move_base_.sendGoal(current_goal_);
    WLG->trace("coordinate sent!");
    return true;
  } else {
    return false;
  }
}

void WaypointManager::GoToWaypointFailed()  // question
{
  ClearCostmap();
  tf::StampedTransform transform;
  if (first_time_move_to_nearest_point_) {
    robot_tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    double delta_x =
        transform.getOrigin().x() - current_goal_.target_pose.pose.position.x;
    double delta_y =
        transform.getOrigin().y() - current_goal_.target_pose.pose.position.y;
    double theta = atan2(delta_y, delta_x);
    first_time_move_to_nearest_point_ = false;
    theta += retry_time_ * 3.14 / 4;
    retry_time_++;
    retry_time_ %= 8;
    geometry_msgs::Quaternion quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(0, 0, theta + 3.14);
    current_goal_nearest_point_.target_pose.pose.orientation.x = quaternion.x;
    current_goal_nearest_point_.target_pose.pose.orientation.y = quaternion.y;
    current_goal_nearest_point_.target_pose.pose.orientation.z = quaternion.z;
    current_goal_nearest_point_.target_pose.pose.orientation.w = quaternion.w;
    current_goal_nearest_point_.target_pose.pose.position.x =
        current_goal_.target_pose.pose.position.x + 0.8 * cos(theta);
    current_goal_nearest_point_.target_pose.pose.position.y =
        current_goal_.target_pose.pose.position.y + 0.8 * sin(theta);
    current_goal_nearest_point_.target_pose.pose.position.z =
        current_goal_.target_pose.pose.position.z;
    move_base_.sendGoal(current_goal_nearest_point_);
  }
}

void WaypointManager::UpdateInternalFlags() {
  if (nh_ptr_->hasParam("process_status")) {
    nh_ptr_->getParam("/process_status/navigation", navigation_started_);
    nh_ptr_->getParam("/process_status/mapping", mapping_started_);
  }
}

void WaypointManager::UpdateMovebaseResult(
    const move_base_msgs::MoveBaseActionResultConstPtr &msg) {
  WLG->trace("got move_base result {}", msg->status.goal_id.id);
  last_movebase_result_ = msg->status.status;
  nh_ptr_->setParam("navigation_status/last_status", last_movebase_result_);
  std_msgs::Int32 data;
  data.data = active_waypoint_;
  
  if (state_ == 2 || state_ == 4) {
    if (last_movebase_result_ == 3) {
      is_moving_to_waypoint_ = false;
      if (moving_to_nearest_point_) {
        moving_to_nearest_point_ = false;
        arrive_at_nearest_point_ = true;

        GoToWaypoint(data);
      } else {
        pub_success_waypoint_.publish(data);
        if (state_ != 4) state_ = 0;
      }
    } else if (last_movebase_result_ == 4) {
      is_moving_to_waypoint_ = false;
      if (arrive_at_nearest_point_) {
        pub_fail_waypoint_.publish(data);
        if (state_ != 4) state_ = 0;
      } else {
        moving_to_nearest_point_ = true;
        GoToWaypointFailed();
      }
    }
  } else if (state_ == 3) {
    if (last_movebase_result_ == 3) {
      is_moving_to_waypoint_ = false;
      WLG->trace("behavior finished and success waypoint {}", active_waypoint_);
      pub_success_waypoint_.publish(data);
    } else if (last_movebase_result_ == 4) {
      is_moving_to_waypoint_ = false;
      WLG->trace("behavior finished and FAIL waypoint {}", active_waypoint_);
      pub_fail_waypoint_.publish(data);
    }
  }
}

void WaypointManager::UpdateMovebaseStatus(
    const actionlib_msgs::GoalStatusArrayConstPtr &data) {
  if (data->status_list.size() > 0) {
    last_movebase_result_ =
        data->status_list[data->status_list.size() - 1].status;
    nh_ptr_->setParam("navigation_status/status", last_movebase_status_);
  }
  if (nh_ptr_->hasParam("/autodocking/alive")) {
    XmlRpc::XmlRpcValue a;
    nh_ptr_->getParam("/autodocking/alive", a);
    if (static_cast<int>(a)) {
      if (nh_ptr_->hasParam("/chargingpoints")) {
        XmlRpc::XmlRpcValue b;
        nh_ptr_->getParam("/chargingpoints", b);
        auto cur_ps = b.begin();
        tf::StampedTransform transform;
        try {
          robot_tf.lookupTransform("/map", "/baselink", ros::Time(0),
                                   transform);
          if (abs(transform.getOrigin().x() -
                  static_cast<float>(
                      static_cast<double>(cur_ps->second["x"]))) < 0.25 &&
              abs(transform.getOrigin().y() -
                  static_cast<float>(
                      static_cast<double>(cur_ps->second["y"]))) < 0.25) {
            nh_ptr_->setParam("navigation_status/at_charging_point", 1);
          } else {
            nh_ptr_->setParam("navigation_status/at_charging_point", 0);
          }
        } catch (tf::TransformException &ex) {
          ros::Duration(1.0).sleep();
          return;
        }
      } else {
        return;
      }
    }
  } else {
    return;
  }
}

void WaypointManager::UpdateBattery(const mobile_platform_msgs::ChassisConstPtr &msg) {
  static bool rechargingFlag = false;
  int battery = msg->bms.battery_soc_percentage;
  if(!ros::param::get("/param/battery_threshold", battery_threshold_))  battery_threshold_ = 0;

  std::string info = fmt::format("UpdateBattery battery_threshold is : {} / {}", battery_threshold_, battery);
  LOG_WITH_GAP(WLG, trace, 5.0, info);

  if (!rechargingFlag && battery < battery_threshold_) {
    WLG->warn("battery is under the lowest threshold ({} < {}), recharging immediately!!", battery, (int)battery_threshold_);

    // recall!!!
    Agent ag;
    if(RecallHandler(ag.request, ag.response)){
      WLG->info("low battery and recharging: code({}), msg({})", ag.response.code, ag.response.msg);
      rechargingFlag = true;
    }
    return;
  }

  if (battery > battery_threshold_) {
    rechargingFlag = false;
  }
}

bool WaypointManager::HandleGoToCoordinate(GoToCoordinate::Request &req,
                                           GoToCoordinate::Response &res) {
  UpdateInternalFlags();
  if (!navigation_started_) {
    res.message = "GoToCoordinate but navigation haven\'t started";
    res.status = -17;
  }
  ResetPatrol();
  state_ = 2;
  GoToCoordinate(req, res);
  return true;
}

bool WaypointManager::HandleGoToWaypoint(GoToWaypoint::Request &req,
                                         GoToWaypoint::Response &res) {
  UpdateInternalFlags();
  if (!navigation_started_) {
    res.status = -18;
    res.message = "GoToWaypoint but navigation haven\'t started";
    return true;
  }
  ResetPatrol();
  std_msgs::String msg;
  msg.data = "forced reload";
  state_ = 2;
  HandleReloadWaypoint(msg);
  std_msgs::Int32 no;
  no.data = req.msg;
  if (GoToWaypoint(no)) {
    res.status = 0;
    res.message = "OK";
    return true;
  } else {
    res.status = -13;
    res.message =
        "ChargingExecute invoked but could not find a valid destination";
    return true;
  }
  return true;
}

bool WaypointManager::HandleGoToChargingPoint(
    GoToChargingPoint::Request &req, GoToChargingPoint::Response &res) {
  UpdateInternalFlags();

  if (!navigation_started_) {
    res.message =
        "ChargingExecute can not start because navigation haven\'t started";
    res.status = -16;
    return true;
  }
  ResetPatrol();
  autodocking_started_ = false;
  state_ = 4;
  if (GoToChargingPoint(req, res)) {
    res.message = "OK";
    res.status = 0;
    return true;
  } else {
    res.message =
        "ChargingExecute invoked but could not find a valid destination";
    res.status = 13;
    return true;
  }
}

void WaypointManager::HandleReloadWaypoint(std_msgs::String req) {
  WLG->trace("{}", req.data);
  WLG->trace("renewing waypoint list, making sure all navigation stops");
  XmlRpc::XmlRpcValue waypoint = XmlRpc::XmlRpcValue();

  if (nh_ptr_->hasParam("/waypoints"))
    nh_ptr_->getParam("/waypoints", waypoint);
  waypointlist_ = {};
  int32_t no = 0;
  geometry_msgs::Pose pose;
  for (auto point = waypoint.begin();
       point != waypoint.end() && no < waypoint.size(); point++, no++) {
    WLG->trace("the info type is {}", point->second.getType());
    WLG->trace("the info size is {}", point->second.size());
    WLG->trace("{}", point->first);

    if (point->first == "orientation") {
      pose.orientation.x =
          static_cast<double>(static_cast<int>(point->second["x"]));
      pose.orientation.y =
          static_cast<double>(static_cast<int>(point->second["y"]));
      pose.orientation.z =
          static_cast<double>(static_cast<int>(point->second["z"]));
      pose.orientation.w =
          static_cast<double>(static_cast<int>(point->second["w"]));
    } else if (point->first == "position") {
      pose.position.x =
          static_cast<double>(static_cast<int>(point->second["x"]));
      pose.position.y =
          static_cast<double>(static_cast<int>(point->second["y"]));
      pose.position.z =
          static_cast<double>(static_cast<int>(point->second["z"]));
    }

    double num = static_cast<double>(static_cast<int>(point->second["x"]));
    WLG->trace("i can get this {}", num);
  }
  WaypointEntry waypointentry = WaypointEntry(pose);
  waypointlist_.push_back(waypointentry);
  WLG->trace("the target_pose position x is {} \n the target_pose orientation y "
              "is {}",pose.position.x, pose.orientation.y);

  n_waypoints_ = waypointlist_.size();
  WLG->trace(" the count is {}", n_waypoints_);
}

bool WaypointManager::HandlePathTracking(Pursuit::Request &req,
                                         Pursuit::Response &res) {

  std::string path_name = req.path_name;
  if (path_name.substr(0,4) == "zone") {
    res.status = -1;
    res.message = "Zone directory can't tracking!";
    return true;
  }

  std::string selected_map;
  req.mode = "path_tracking_with_path_name";

  if (nh_ptr_->getParam("/selected_map", selected_map)) {
    req.map = selected_map;
    req.mode = "path_tracking_with_path_name";

    // absolute path
    req.path_name = MAP_DIRECTORY + selected_map + "/path/" + req.path_name + ".csv";

    WLG->trace("path tracking map: {}", req.map);
    WLG->trace("path tracking path_name: {}", req.path_name);
    WLG->trace("path tracking command: {}", req.command);
    WLG->trace("path tracking mode: {}", req.mode);
    WLG->trace("path tracking task_index: {}", req.task_index);
  } else {
    WLG->error("selected map cant get!!");
    res.status = -1;
    res.message = "selected map cant get!!";
    return true;
  }

  Pursuit srv;
  srv.request = req;

  if (path_tracking_client_.call(srv))
    serviceHelper(s_logger->logger, res, "call pursuit: " + srv.response.message,
                  srv.response.status);
  else
    serviceHelper(s_logger->logger, res, "path_tracking_client_call defeat!!", -1);

  return true;
}

bool WaypointManager::StartCharging(GoToChargingPoint::Request &req,
                                    GoToChargingPoint::Response &res) {
  WLG->trace("start charging!");
  if (EnableCharingService(true)) {
    res.status = 0;
    res.message = "Robot starts charging service.";
  } else {
    res.status = -1;
    res.message = "Charging service call is failed or service is ON already.";
  }
  return true;
}

bool WaypointManager::StopCharging(GoToChargingPoint::Request &req,
                                   GoToChargingPoint::Response &res) {
  if (EnableCharingService(false)) {
    res.status = 0;
    res.message = "Robot stops charging service successfully.";
  } else {
    res.status = -1;
    res.message =
        "Charging service can not be stopped or service is OFF already.";
  }
  return true;
}

bool WaypointManager::EnableCharingService(bool enable) {
  // bool process_status = true;
  // ros::param::get("/process_status/navigation", process_status);

  if (!getEgoStatus("navigation")) {
    // }
    // if(!process_status){
    // try to turn on navigation
    WLG->warn("navigation is not turned on, now try to turn it on...");
    // check whether in status of navigation
    mobile_platform_msgs::SystemdService ss_srv;
    ss_srv.request.service_name = "navigation";
    ss_srv.request.turn_on = true;
    if (!ros::service::call("/ui/systemdservice", ss_srv)) {
      WLG->error("EnableCharingService: unknown reason!!");
      return false;
    }
    if (ss_srv.response.status != 0) {
      WLG->error("EnableCharingService: %s", ss_srv.response.message);
      return false;
    }
  }
  std_msgs::Bool msg;
  msg.data = enable;
  pub_home_to_dock_.publish(msg);
  return true;
}

bool WaypointManager::RecallHandler(AgentRequest& req, AgentResponse& res)
{
  // 1. get current pose
  // 2. call home to dock

  res.trace_id = req.trace_id;

  std::vector<float> pose;
  geometry_msgs::Pose p;
  
  if(ros::param::get("/robot_pose", pose) && pose.size() >= 7)
  {
    p.position.x = pose[0];
    p.position.y = pose[1];
    p.position.z = pose[2];
    p.orientation.x = pose[3];
    p.orientation.y = pose[4];
    p.orientation.z = pose[5];
    p.orientation.w = pose[6];

    nav_msgs::Odometry odom;
    odom.pose.pose = p; 

    HomeToDock htd;
    htd.request.start = odom;
    if(ros::service::call("/auto_dock/home_to_dock", htd)){
      if(htd.response.path.poses.size() < 1.0 / 0.2){
        agentHelper(s_logger->logger, res, "auto docking get too little path poses!!", -1);
        return true;
      }
      // stop pursuit first
      Pursuit ps;
      ps.request.command = 2;
      ros::service::call("/pursuit", ps);

      // call pursuit
      ps.request.map = getSelectedMap();
      ps.request.command = 1;
      ps.request.mode = "path_tracking_with_path_data";
      ps.request.path = htd.response.path;
      ps.request.path_name = "recall";
      ps.request.task_index = 1;

      if(ros::service::call("/pursuit", ps)){
        if(ps.response.status == 0){
          agentHelper(s_logger->logger, res, "ok", 0);
          return true;      
        }
        agentHelper(s_logger->logger, res, ps.response.message, -1);
        return true;
      }
      agentHelper(s_logger->logger, res, "pursuit offline", -1);
    }else{
      agentHelper(s_logger->logger, res, "auto-docking offline", -1);
    }
  }else{
    agentHelper(s_logger->logger, res, "invalid robot pose", -1);
  }

  return true;
}