/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "param_manager.h"

#include <mobile_platform_msgs/MappingSave.h>
#include <mobile_platform_msgs/VolumeChange.h>
#include "common/OssUploadHelper.hpp"

const int ds_num = 500;

std::string generate_id() {
	std::string res;
	srand((int)time(NULL));
	for (int i = 0; i < 8; i++) {
		res += std::to_string((rand() % 10));
	}
	return res;
}

void ParamManager::Init(ros::NodeHandle *nh_ptr) {
  nh_ptr_ = nh_ptr;
  g_patrolling_ = false;
  g_navifation_state_ = "";
  g_autodocking_state_ = "";
  init_pose_saved_ = false;
  XmlRpc::XmlRpcValue pose;
  pose.setSize(8);
  for (int i = 0; i < pose.size(); i++) {
	if (i == 6)
	  pose[i] = 1;
	else
	  pose[i] = 0;
  }
  nh_ptr_->setParam("/robot_pose", pose);
  nh_ptr_->setParam("/tracking_path", "");
  // geometry_msgs::Pose path_point;
  // path_point.position.x = 0;
  // path_point.position.y = 0;
  // path_point.position.z = 0;
  // path_point.orientation.x = 0;
  // path_point.orientation.y = 0;
  // path_point.orientation.z = 0;
  // path_point.orientation.w = 1;

  // publishers
  service_update_virtual_line_ =
	  nh_ptr->advertise<std_msgs::Int16>("/prohibition_layer/update", 3);
  amcl_initpose_pub_ =
	  nh_ptr_->advertise<geometry_msgs::PoseWithCovarianceStamped>(
		  "/initialpose", 1);
  // loc_stat_pub_ =
	//   nh_ptr_->advertise<LocalizationLost>("/ui/localization_lost", 1);
  e_stop_pub_ =
	  nh_ptr_->advertise<geometry_msgs::Twist>("/velocity_smoother/e_stop", 1);
  // servers
  ui_waypoint_save_ = nh_ptr->advertiseService(
	  "/ui/waypoint/save", &ParamManager::HandleWaypointSave, this);
  ui_waypoint_load_ = nh_ptr->advertiseService( 
	  "/ui/waypoint/load", &ParamManager::HandleWaypointLoad, this);
  ui_initpose_save_ = nh_ptr->advertiseService(
	  "/ui/initpose/save", &ParamManager::HandleInitialPoseSave, this);
  ui_initpose_load_ = nh_ptr->advertiseService(
	  "/ui/initpose/load", &ParamManager::HandleInitialPoseLoad, this);
  ui_charging_save_ = nh_ptr->advertiseService(
	  "/ui/charging/save", &ParamManager::HandleChargingSave, this);
  ui_charging_load_ = nh_ptr->advertiseService(
	  "/ui/charging/load", &ParamManager::HandleChargingLoad, this);
  ui_virtualline_save_ = nh_ptr->advertiseService(
	  "/ui/virtualline/save", &ParamManager::HandleVirtualLineSave, this);
  ui_virtualline_load_ = nh_ptr->advertiseService(
	  "/ui/virtualline/load", &ParamManager::HandleVirtualLineLoad, this);
  ui_path_planning_list_ = nh_ptr->advertiseService(
	  "/ui/path_planning/list", &ParamManager::GetPathList, this);
  ui_path_planning_save_ = nh_ptr->advertiseService(
	  "/ui/path_planning/save", &ParamManager::HandlePathSave, this);
  ui_path_planning_load_ = nh_ptr->advertiseService(
	  "/ui/path_planning/load", &ParamManager::HandlePathLoad, this);
  ui_path_planning_delete_ = nh_ptr->advertiseService(
	  "/ui/path_planning/delete", &ParamManager::HandlePathDelete, this);
  ui_system_state_ = nh_ptr->advertiseService(
	  "/ui/system_state", &ParamManager::HandleBaseStatus, this);
  ui_waypoint_upload_ = nh_ptr->advertiseService(
	  "/ui/waypoint/upload", &ParamManager::HandleWaypointUpload, this);

  ui_localization_lost_ = nh_ptr_->subscribe(
	  "/ui/localization_lost", 1, &ParamManager::HandleJudgeLocalizationLost, this);
  ui_semantic_layer_save_ = nh_ptr_->advertiseService(
	  "/ui/semanticlayer/save", &ParamManager::HandleSemanticLayerSave, this);
  ui_semantic_layer_edit_ = nh_ptr_->advertiseService(
	  "/ui/semanticlayer/edit", &ParamManager::HandleSemanticLayerEdit, this);
  ui_semantic_layer_delete_ =
	  nh_ptr_->advertiseService("/ui/semanticlayer/delete",
								&ParamManager::HandleSemanticLayerDelete, this);
  ui_zoning_coverage = nh_ptr_->advertiseService(
	  "/ui/zoning/coverage", &ParamManager::HandleZoningCoverage, this);
  ui_zoning_coverage_consolidation = nh_ptr_->advertiseService(
	  "/ui/zoning_coverage/consolidation",
	  &ParamManager::HandelZoningCoverageConsolidation, this);

  // Subscriber
  amcl_amcl_status_ = nh_ptr_->subscribe("/amcl/amcl_status", 1,
										 &ParamManager::HandleAmclStatus, this);
  odom_ =
	  nh_ptr_->subscribe("/odom", 1, &ParamManager::HandleRobotPoseOdom, this);
 
  // clients
  convex_ploygon_client_ = nh_ptr_->serviceClient<CoveragePointMap>(
	  "/planner/boustrophedon_coverage");
  coverage_surround_client_ =
	  nh_ptr_->serviceClient<JTCoveragePlanning>("/planner/coverage_planning");

  passpoint_planner_client_ =
	  nh_ptr_->serviceClient<PassingPoint>("/pass_point_planner");
  targetpoint_planner_client_ =
	  nh_ptr_->serviceClient<TargetPoint>("/target_point_planner");

  astar_start_pub_ = nh_ptr_->advertise<geometry_msgs::PoseStamped>("/astar_start", 1, true);
  astar_end_pub_ = nh_ptr_->advertise<geometry_msgs::PoseStamped>("/astar_end", 1, true);
  zone_api_gong_client_ = nh_ptr_->serviceClient<ZoneApiSrv>("/zone_api/gong");
  zone_api_hui_client_ = nh_ptr_->serviceClient<ZoneApiSrv>("/zone_api/hui");

  ui_setting_ = nh_ptr_->advertiseService("/ui/setting", &ParamManager::settingHandler, this);

  LG->info("param manager init!!");
}

bool ParamManager::HandleInitialPoseSave(InitPoseSave::Request &req,
										 InitPoseSave::Response &res) {
  nh_ptr_->setParam("/amcl/initial_pose_x", req.pose.position.x);
  nh_ptr_->setParam("/amcl/initial_pose_y", req.pose.position.y);
  double roll, pitch, yaw;
  tf::Quaternion qq(req.pose.orientation.x, req.pose.orientation.y,
					req.pose.orientation.z, req.pose.orientation.w);
  tf::Matrix3x3(qq).getRPY(roll, pitch, yaw);
  nh_ptr_->setParam("/amcl/initial_pose_a", yaw);
  nh_ptr_->setParam("/amcl/initial_cov_aa", 0.1);
  nh_ptr_->setParam("/amcl/initial_cov_xx", 0.25);
  nh_ptr_->setParam("/amcl/initial_cov_yy", 0.25);
  nh_ptr_->setParam("/amcl/ext/initial_pose_x", req.pose.position.x);
  nh_ptr_->setParam("/amcl/ext/initial_pose_y", req.pose.position.y);
  nh_ptr_->setParam("/amcl/ext/initial_pose_a", yaw);

  if (SaveRosParamToFile("/amcl", "amcl.yaml")) {
	// std::string command = "sudo -u ";
	// command += USER_NAME;
	// command +=
	//     " bash -c \"source /opt/ros/$ROS_DISTRO/setup.bash;rosparam dump "
	//     "~/.robot/config/amcl_params.yaml /amcl\"";
	// open_system(command);

	geometry_msgs::PoseWithCovarianceStamped initpose;
	initpose.header.frame_id = "map";
	initpose.header.stamp = ros::Time::now();
	initpose.pose.pose = req.pose;
	initpose.pose.covariance[0] = 0.25;
	initpose.pose.covariance[7] = 0.25;
	initpose.pose.covariance[35] = 0.06853891;
	amcl_initpose_pub_.publish(initpose);
	res.message = "OK";
	res.status = 0;

	return true;
  } else {
	res.message = "Saving InitialPose failed!";
	res.status = -1;
	return true;
  }
}
bool ParamManager::HandleInitialPoseLoad(InitPoseLoad::Request &req,
										 InitPoseLoad::Response &res) {
  geometry_msgs::Pose pose;
  float x, y, a;
  if (!LoadRosParamFromFile("/amcl", "amcl.yaml")) {
	res.message = "Parameters have not been initialized yet.";
	res.status = 0;
	res.pose = pose;
	return true;
  }
  if (!nh_ptr_->hasParam("/amcl/ext/initial_pose_x") ||
	  !nh_ptr_->hasParam("/amcl/ext/initial_pose_y") ||
	  !nh_ptr_->hasParam("/amcl/ext/initial_pose_a")) {
	nh_ptr_->getParam("/amcl/ext/initial_pose_x", x);
	nh_ptr_->getParam("/amcl/ext/initial_pose_y", y);
	nh_ptr_->getParam("/amcl/ext/initial_pose_a", a);
  }

  pose.position.x = x;
  pose.position.y = y;
  tf::Quaternion qq;
  qq = tf::createQuaternionFromRPY(0, 0, a);
  pose.orientation.x = qq[0];
  pose.orientation.y = qq[1];
  pose.orientation.z = qq[2];
  pose.orientation.w = qq[3];

  res.message = "OK";
  res.pose = pose;
  res.status = 0;
  return true;
}
// save map related parameters to files in the map folder
bool ParamManager::SaveRosParamToFile(const std::string& param_name,
									  const std::string& file_name) {
  if (!nh_ptr_->hasParam("/selected_map")) {
	LG->error("paramservice has no member /selected_map");
	return false;
  }
  std::string map_name;
  nh_ptr_->getParam("/selected_map", map_name);
  std::string map_dir = MAP_DIRECTORY + map_name + "/";
  std::string rosparam_name = " " + param_name + "\"";
  std::string command = "sudo -u ";
  command += USER_NAME;
  command += " bash -c \"source /opt/ros/$ROS_DISTRO/setup.bash;rosparam dump ";
  command += map_dir + file_name + rosparam_name;
  LG->trace("cmd is : {}", command);
  open_system(command);
  return true;
}
// load map related parameters from files in the map folder
bool ParamManager::LoadRosParamFromFile(const std::string& param_name,
										const std::string& file_name) {
  if (!nh_ptr_->hasParam("/selected_map")) {
	LG->error("paramservice has no member /selected_map");
	return false;
  }

  std::string map_name;
  nh_ptr_->getParam("/selected_map", map_name);
  std::string map_dir = MAP_DIRECTORY + map_name + "/";
  std::string rosparam_name = " " + param_name + "\"";
  std::string command = "sudo -u ";
  command += USER_NAME;
  command += " bash -c \"source /opt/ros/$ROS_DISTRO/setup.bash;rosparam load ";
  command += map_dir + file_name + rosparam_name;

  if (access((map_dir + file_name).c_str(), F_OK)) {
	LG->error("file {} not exist, LoadRosParamFromFile failed!",
			map_dir + file_name);
	return false;
  }
  open_popen(command);

  return true;
}
bool ParamManager::HandleWaypointSave(WaypointSave::Request &req,
									  WaypointSave::Response &res) {
  int index = 0;
  XmlRpc::XmlRpcValue waypoint;
  for (int i = 0; i < req.poses.size(); i++) {
	waypoint[index]["position"]["x"] = req.poses[i].position.x;
	waypoint[index]["position"]["y"] = req.poses[i].position.y;
	waypoint[index]["position"]["z"] = req.poses[i].position.z;
	waypoint[index]["orientation"]["x"] = req.poses[i].orientation.x;
	waypoint[index]["orientation"]["y"] = req.poses[i].orientation.y;
	waypoint[index]["orientation"]["z"] = req.poses[i].orientation.z;
	waypoint[index]["orientation"]["w"] = req.poses[i].orientation.w;
	index++;
  }
  nh_ptr_->setParam("/waypoints", waypoint);
  if (SaveRosParamToFile("/waypoints", "waypoints.yaml")) {
	res.message = "OK";
	res.status = 0;
	return true;
  } else {
	res.message = "Saving waypoints failed";
	res.status = -1;
	return true;
  }
  return true;
}
bool ParamManager::HandleWaypointLoad(WaypointLoad::Request &req,
									  WaypointLoad::Response &res) {
  std::vector<geometry_msgs::Pose> waypoints;
  if (!(LoadRosParamFromFile("/waypoints", "waypoints.yaml"))) {
	res.status = 1;
	res.message = "load waypoints failed";
	return true;
  }
  XmlRpc::XmlRpcValue ps;
  if (nh_ptr_->hasParam("/waypoints")) {
	nh_ptr_->getParam("/waypoints", ps);
	for (int i = 0; i < ps.size(); i++) {
	  try {
	    geometry_msgs::Pose pose;
	    pose.position.x = ps[i]["position"]["x"];
	    pose.position.y = ps[i]["position"]["y"];
	    pose.position.z = ps[i]["position"]["z"];
	    pose.orientation.x = ps[i]["orientation"]["x"];
	    pose.orientation.y = ps[i]["orientation"]["y"];
	    pose.orientation.z = ps[i]["orientation"]["z"];
	    pose.orientation.w = ps[i]["orientation"]["w"];
	  } catch (const XmlRpc::XmlRpcException &e) {
	    LG->error("XmlRpcException : %s", e.getMessage().c_str());
	  }
	  waypoints.push_back(pose);
	}
  }
  res.poses = waypoints;
  res.message = "OK";
  return true;
}

bool ParamManager::HandleChargingSave(ChargingSave::Request &req,
									  ChargingSave::Response &res) {
//   int index = 0;
//   XmlRpc::XmlRpcValue chargingpoint;
//   for (auto point = req.poses.begin(); point != req.poses.end(); point++) {
// 	chargingpoint[index]["position"]["x"] = point->position.x;
// 	chargingpoint[index]["position"]["y"] = point->position.y;
// 	chargingpoint[index]["position"]["z"] = point->position.z;
// 	chargingpoint[index]["orientation"]["x"] = point->orientation.x;
// 	chargingpoint[index]["orientation"]["y"] = point->orientation.y;
// 	chargingpoint[index]["orientation"]["z"] = point->orientation.z;
// 	chargingpoint[index]["orientation"]["w"] = point->orientation.w;
// 	index++;
//   }
//   nh_ptr_->setParam("/chargingpoints", chargingpoint);
//   if (!SaveRosParamToFile("/chargingpoints", "chargingpoints.yaml"))
//   {
// 	res.message = "Saving chargingpoints failed";
// 	res.status = -1;
//   }

	InitPoseLoad ipl;
	if(!ros::service::call("/autodocking/get_pose", ipl)){
		LG->error("auto-docking offline!");
		responseHelper(res, -1, "auto-docking offline!");
		return true;
	}

	// write to layer.json
	if(ipl.response.status != 0){
		LG->error("fail to get charing point!");
		responseHelper(res, -1, "fail to get charing point!");
		return true;
	}

	const auto& p = ipl.response.pose;
	std::vector<float> data(3, 0);
	data[0] = p.position.x;
	data[1] = p.position.y;
	// ------------- trans data to image -------------
	data[2] = MapImageTrans::toAng(M_PI - tf::getYaw(p.orientation));
	auto map_name = getSelectedMap();
	std::string yn = MAP_DIRECTORY + map_name + "/" + map_name + ".yaml";
	mit_.setYaml(yn);
	auto tv = mit_.mapToImage(std::vector<float>{data[0], data[1]});
	data[0] = tv[0];
	data[1] = tv[1];

	json j;
	j["property"] = "charging_point";
	j["type"] = "point";
	j["map_name"] = map_name;
	j["message"]["data"] = data;
	j["message"]["name"] = generate_id();

	OssUpload oss;
	oss.request.filename = j.dump();
	HandleSemanticLayerSave(oss.request, oss.response);

	if(oss.response.status != 0){
		std::string msg = "save charging point fail: " + oss.response.message;
		LG->error("{}", msg);
		responseHelper(res, -1, msg);
		return true;
	}

	return true;
}

bool ParamManager::HandleChargingLoad(ChargingLoad::Request &req,
									  ChargingLoad::Response &res) {
  std::vector<geometry_msgs::Pose> waypoints;
  if (!(LoadRosParamFromFile("/chargingpoints", "chargingpoints.yaml"))) {
	res.status = 1;
	res.message = "load chargingpoints failed";
	return true;
  }
  XmlRpc::XmlRpcValue ps;
  if (nh_ptr_->hasParam("/chargingpoints")) {
	nh_ptr_->getParam("/chargingpoints", ps);
  }

  for (int i = 0; i < ps.size(); i++) {
	geometry_msgs::Pose pose;
	try {
	  pose.position.x = ps[i]["position"]["x"];
	  pose.position.y = ps[i]["position"]["y"];
	  pose.position.z = ps[i]["position"]["z"];
	  pose.orientation.x = ps[i]["orientation"]["x"];
	  pose.orientation.y = ps[i]["orientation"]["y"];
	  pose.orientation.z = ps[i]["orientation"]["z"];
	  pose.orientation.w = ps[i]["orientation"]["w"];
	} catch (const XmlRpc::XmlRpcException &e) {
	  LG->error("XmlRpcException: {}", e.getMessage());
	}
	waypoints.push_back(pose);
	nh_ptr_->setParam("/charging_points/origin_x", pose.position.x);
	nh_ptr_->setParam("/charging_points/origin_y", pose.position.y);
	tf::Quaternion quaternion(pose.orientation.x, pose.orientation.y,
								pose.orientation.z, pose.orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
	nh_ptr_->setParam("/charging_points/origin_theta", yaw);
  }
  res.poses = waypoints;
  res.message = "OK";
  return true;
}
bool ParamManager::HandleVirtualLineSave(VirtualLineSave::Request &req,
										 VirtualLineSave::Response &res) {
  int index = 0;
  XmlRpc::XmlRpcValue lines;
  for (int i = 0; i < (req.poses.size() / 2); i++) {
	XmlRpc::XmlRpcValue line;
	geometry_msgs::Point p1 = req.poses[i * 2].position;
	geometry_msgs::Point p2 = req.poses[i * 2 + 1].position;
	p1.x = round(p1.x * 1000) / 1000;
	p1.y = round(p1.y * 1000) / 1000;
	p2.x = round(p2.x * 1000) / 1000;
	p2.y = round(p2.y * 1000) / 1000;
	line[0]["x"] = p1.x;
	line[0]["y"] = p1.y;
	line[1]["x"] = p2.x;
	line[1]["y"] = p2.y;
	lines[index] = line;
	index++;
  }
  nh_ptr_->setParam(
	  "/move_base/global_costmap/costmap_prohibition_layer/prohibition_areas",
	  lines);
  nh_ptr_->setParam(
	  "/move_base/global_costmap/costmap_prohibition_layer/enabled", true);
  nh_ptr_->setParam(
	  "/move_base/global_costmap/costmap_prohibition_layer/fill_polygons",
	  true);
  nh_ptr_->setParam(
	  "/move_base/local_costmap/costmap_prohibition_layer/prohibition_areas",
	  lines);
  nh_ptr_->setParam(
	  "/move_base/local_costmap/costmap_prohibition_layer/enabled", true);
  nh_ptr_->setParam(
	  "/move_base/local_costmap/costmap_prohibition_layer/fill_polygons", true);
  std_msgs::Int16 msg;
  msg.data = 0;
  service_update_virtual_line_.publish(msg);

  if (!SaveRosParamToFile("/move_base/global_costmap/costmap_prohibition_layer",
						 "virtualpoints.yaml")) {
	res.message = "Saving prohibition_areas failed";
	res.status = -1;
  }
  return true;
}

bool ParamManager::HandleVirtualLineLoad(VirtualLineLoad::Request &req,
										 VirtualLineLoad::Response &res) {
  if (!LoadRosParamFromFile(
	  "/move_base/global_costmap/costmap_prohibition_layer",
	  "virtualpoints.yaml")) {
	res.message = "load virtualpoints failed";
	res.status = -1;
	return true;
  }

  XmlRpc::XmlRpcValue lines;
  if (nh_ptr_->hasParam("/move_base/global_costmap/costmap_prohibition_layer/"
						"prohibition_areas")) {
	nh_ptr_->getParam(
		"/move_base/global_costmap/costmap_prohibition_layer/prohibition_areas",
		lines);
	LG->trace("HandleVirtualLineLoad: about to load virtualline!!");
	LG->trace("HandleVirtualLineLoad: size of lines: {}", lines.size());
	for (int i = 0; i < lines.size(); i++) {
	  geometry_msgs::Pose p1, p2;

	  p1.position.x = lines[i][0]["x"];
	  p1.position.y = lines[i][0]["y"];

	  p2.position.x = lines[i][1]["x"];
	  p2.position.y = lines[i][1]["y"];

	  res.poses.push_back(p1);
	  res.poses.push_back(p2);
	}
	res.message = "OK";
	res.status = 0;
  } else {
	LG->error("HandleVirtualLineLoad: params not existed!!");
  }
  return true;
}
bool ParamManager::GetPathList(PathPlanningList::Request &req,
							   PathPlanningList::Response &res) {
  if (!nh_ptr_->hasParam("/selected_map")) {
	LG->error("cant select map !");

	return false;
  }
  std::string map_name;
  nh_ptr_->getParam("/selected_map", map_name);
  std::string path_ptr = MAP_DIRECTORY + map_name + "/path";
  std::vector<std::string> path_files = getFileName(path_ptr, ".csv");
  std::vector<std::string> copy_file;
  path_ptr = MAP_DIRECTORY + map_name + "/zone";
  if (access(path_ptr.c_str(), F_OK)) 
	LG->error(" Didn't zone directory!");
  else {
	copy_file = getFileName(path_ptr, ".csv", "zone_");
	copy(copy_file.begin(), copy_file.end(),
	  std::back_insert_iterator<std::vector<std::string>>(path_files));

	std::vector<std::string> zone_name = getFileName(path_ptr);
	for (std::vector<std::string>::iterator it = zone_name.begin();
	  it != zone_name.end(); ++it) {
	  if (it->find(".") == -1) {
	  copy_file = getFileName(path_ptr + "/" + *it, ".csv", *it + "_");
	  copy(copy_file.begin(), copy_file.end(),
	  	std::back_insert_iterator<std::vector<std::string>>(path_files));
	  }
	}
  }
  
  last_path_files = path_files;
  res.names = path_files;
  res.status = 0;
  res.message = "OK";
  return true;
}
bool ParamManager::HandlePathLoad(PathPlanningLoad::Request &req,
								  PathPlanningLoad::Response &res) {
  std::vector<geometry_msgs::Pose> path;

  bool is_reqpath_exsit = false;
  for (auto name : last_path_files) {
	if (name == req.path_name) {
	  is_reqpath_exsit = true;
	  break;
	}
  }

  if (!nh_ptr_->hasParam("/selected_map")) {
	LG->error("paramservice has no member /selected_map");
	return false;
  }

  std::string map_name;
  nh_ptr_->getParam("/selected_map", map_name);
  
  if (!is_reqpath_exsit) {
	res.poses = path;
	res.message = "No such path";
	res.status = -1;
	return true;
  }

  std::string path_name = req.path_name;
  std::string sub_path_name = req.path_name;
  if (sub_path_name.substr(0,4) == "zone") {
	if ( '0' <= sub_path_name[5] && sub_path_name[5] <= '9') {
	  path_name = "/zone"  "/" + sub_path_name.substr(0,6) 
	  					+ "/" + sub_path_name.substr(7);
	}
	else path_name = "/zone"  "/" + sub_path_name.substr(5);
  }
  else  path_name = "/path/" + path_name; 
  
  std::ifstream pathrecord_input(MAP_DIRECTORY + map_name +
                                 path_name + ".csv");
  if (pathrecord_input) {
    std::string line;
	std::string field;

	while (std::getline(pathrecord_input, line)) {
	  std::stringstream sin(line);
	  int index = 0;
	  std::vector<float> data;
	  while (std::getline(sin, field, ' ')) {
	    data.push_back(std::stof(field));
	    if (++index == 6) {
	      break;
	    }
	  }
		if(data.empty())	break;
	  geometry_msgs::Pose pose;
	  for (int i = 0; i < 6;) {
	    pose.position.x = data[i++];
	    pose.position.y = data[i++];
	    pose.position.z = 0;
	    pose.orientation.x = data[i++];
	    pose.orientation.y = data[i++];
	    pose.orientation.z = data[i++];
	    pose.orientation.w = data[i++];
	  }

	  path.push_back(pose);
	}
	pathrecord_input.close();

	std::string string_list;
	std::vector<float> float_list;
	nh_ptr_->getParam("/tracking_path", string_list);
	std::istringstream str(string_list);
	std::string out;

	while (str >> out) {
	  float_list.push_back(atof(out.c_str()));
	}
	for (int i = 0; i < float_list.size();) {
	  geometry_msgs::Pose pose;
	  pose.position.x = float_list[i++];
	  pose.position.y = float_list[i++];
	  pose.position.z = 0;
	  pose.orientation.x = float_list[i++];
	  pose.orientation.y = float_list[i++];
	  pose.orientation.z = float_list[i++];
	  pose.orientation.w = float_list[i++];
	  path.push_back(pose);
	}

	if(path.size() < 5){
		res.message = "path size too small(less than 5)";
		res.status = -1;
		res.poses = path;
		res.closeloop = false;
		return true;
	}

	double temp_distance;
	temp_distance =
	  std::pow(path[0].position.y - path[path.size() - 1].position.y, 2) +
	  std::pow(path[0].position.x - path[path.size() - 1].position.x, 2);
	if (std::sqrt(temp_distance) <= 2) {
	  res.message = "load close loop path successfully";
	  res.status = 0;
	  res.poses = path;
	  res.closeloop = true;
	  return true;
	}
	res.message = "load path successfully";
	res.status = 0;
	res.poses = path;
	res.closeloop = false;
	return true;
  } else {
	pathrecord_input.close();
	res.poses = path;
	res.message = "load path failed";
	res.status = -1;
	return true;
  }

  return true;
}

bool ParamManager::HandlePathSave(PathPlanningSave::Request &req,
								  PathPlanningSave::Response &res) {
  PathPlanningList::Request request;
  PathPlanningList::Response response;
  request.msg = 0;
  GetPathList(request, response);
//   for (auto name : last_path_files) {
// 	if (req.path_name == name) {
// 	  res.message = "path name existed, save failed";
// 	  res.status = -1;
// 	  return true;
// 	}
//   }

  std::string src = MAP_DIRECTORY + "current/current_path_record.yaml";
  std::string dist = MAP_DIRECTORY + getSelectedMap() + "/path/" + req.path_name;
  open_system("cp " + src + " " + dist + ".csv");
  open_system("cp " + src + " " + dist + ".yaml");

  LG->info("path is save, and we upload it to app automatically!!");

  OssUpload srv;
  srv.request.filename = req.path_name;
  HandleWaypointUpload(srv.request, srv.response);
  res.status = srv.response.status;
  res.message = srv.response.message;

  return true;
}

bool ParamManager::HandlePathDelete(PathPlanningDelete::Request &req,
									PathPlanningDelete::Response &res) {
  LG->error("try to delete {}", req.path_name);
  PathPlanningList::Request request;
  PathPlanningList::Response response;
  request.msg = 0;
  GetPathList(request, response);
  for (auto name : last_path_files) {
	if (req.path_name == name) {
	  if (HelpDelete(req.path_name + ".csv")) {
		res.status = 0;
		res.message = "OK";
		return true;
	  } else {
		res.status = -2;
		res.message = "path delete may be failed, os returned error";
		return true;
	  }
	}
  }
  res.status = -1;
  res.message = "no such path";
  return true;
}
bool ParamManager::HelpDelete(std::string filename) {
  try {
	if (!nh_ptr_->hasParam("/selected_map")) {
	  LG->error("cant get the /selected_map from paramserver!");
	  return false;
	}
	std::string map_name;
	nh_ptr_->getParam("/selected_map", map_name);
	std::string path_dir = MAP_DIRECTORY + map_name + "/path/";
	LG->trace("delete path is {}", path_dir);
	LG->trace("delete filename is {}", filename);
	int flag = std::remove((path_dir + filename).c_str());
	if (!flag) {
	  LG->trace("Delete Success!!");
	  return true;
	} else {
	  LG->error("Delete Defeat!!");
	  return false;
	}
  } catch (const std::exception &e) {
	LG->error("exception: {}", e.what());
	return false;
  }
}
bool ParamManager::HandleBaseStatus(BaseStatus::Request &req,
									BaseStatus::Response &res) {
  if (nh_ptr_->hasParam("process_status")) {
	XmlRpc::XmlRpcValue process_status;
	nh_ptr_->getParam("process_status", process_status);
	if (process_status["mapping"]) {
	  res.mapping_running = 1;
	}

	if (process_status["navigation"]) res.navigation_running = 1;
	if (g_patrolling_) res.patrolling = 1;
  } else {
	res.mapping_running = 0;
	res.navigation_running = 0;
	res.patrolling = 0;
  }
  if (nh_ptr_->hasParam("/base_bringup")) {
	XmlRpc::XmlRpcValue base_bringup_status;
	nh_ptr_->getParam("/base_bringup", base_bringup_status);
	res.battery_voltage =
		static_cast<int>(base_bringup_status["battery_voltage"]);
	res.mc_feedback = static_cast<int>(base_bringup_status["mc_feedback"]);
  } else {
	res.battery_voltage = 0;
	res.mc_feedback = 0;
  }
  if (nh_ptr_->hasParam("/sensor_status")) {
	XmlRpc::XmlRpcValue sensor_status;
	nh_ptr_->getParam("/sensor_status", sensor_status);
	res.lidar_feed = static_cast<int>(sensor_status["lidar"]);
	res.sonar_feed = static_cast<int>(sensor_status["sonar"]);
  } else {
	res.lidar_feed = 0;
	res.sonar_feed = 0;
  }

  res.navigation_state = g_navifation_state_;
  res.autodocking_state = g_autodocking_state_;
  return true;
}

void ParamManager::HandleAmclStatus(std_msgs::Int32 amcl_status) {
  if (!amcl_status.data) {
	if (nh_ptr_->hasParam("/amcl/initial_pose_x") &&
		nh_ptr_->hasParam("/amcl/initial_pose_y") &&
		nh_ptr_->hasParam("/amcl/initial_pose_a")) {
	  geometry_msgs::Pose pose;
	  float x, y, a;
	  nh_ptr_->getParam("/amcl/initial_pose_x", x);
	  nh_ptr_->getParam("/amcl/initial_pose_y", y);
	  nh_ptr_->getParam("/amcl/initial_pose_a", a);
	  pose.position.x = x;
	  pose.position.y = y;
	  tf::Quaternion qq;
	  qq = tf::createQuaternionFromRPY(0, 0, a);
	  pose.orientation.x = qq[0];
	  pose.orientation.y = qq[1];
	  pose.orientation.z = qq[2];
	  pose.orientation.w = qq[3];
	  initpose = geometry_msgs::PoseWithCovarianceStamped();
	  initpose.header.frame_id = "map";
	  initpose.header.stamp = ros::Time::now();
	  initpose.pose.pose = pose;
	  initpose.pose.covariance[0] = 0.25;
	  initpose.pose.covariance[7] = 0.25;
	  initpose.pose.covariance[35] = 0.06853891;
	  LG->info("save current pose as initial pose");
	  init_pose_saved_ = true;
	}
  } else {
	if (init_pose_saved_) {
	  init_pose_saved_ = false;
	  initpose.header.stamp = ros::Time::now();
	  amcl_initpose_pub_.publish(initpose);
	  LG->info("send initial pose to amcl");
	}
  }
}

void ParamManager::HandleRobotPoseOdom(nav_msgs::OdometryConstPtr robot_pose) {
  std::vector<float> pose;
  pose.push_back(robot_pose->pose.pose.position.x);
  pose.push_back(robot_pose->pose.pose.position.y);
  pose.push_back(robot_pose->pose.pose.position.z);
  pose.push_back(robot_pose->pose.pose.orientation.x);
  pose.push_back(robot_pose->pose.pose.orientation.y);
  pose.push_back(robot_pose->pose.pose.orientation.z);
  pose.push_back(robot_pose->pose.pose.orientation.w);
  pose.push_back(1);
  nh_ptr_->setParam("/robot_pose", pose);
}

bool ParamManager::HandleCreatePath(CreatePath::Request &req,
									CreatePath::Response &res) {
  printf("create path");
  CreatePath msg;
  msg.request = req;
  ServiceHandleNdtCreatePath(msg.request, msg.response);
  res.status = msg.response.status;
  res.message = "OK";

  return true;
}
bool ParamManager::HandleCreateZone(CreateZone::Request &req,
									CreateZone::Response &res) {
  printf("ndt create Zone");
  std::string filepath =
	  MAP_DIRECTORY + req.map_name + "/zone_" + req.path_name + ".csv";
  std::ofstream outFile;
  outFile.open(filepath, std::ios::out);
  for (int i = 0; i < req.coordinates.points.size(); i++) {
	outFile << req.coordinates.points[i].x << "," << req.coordinates.points[i].y
			<< std::endl;
  }
  outFile.close();
  CreateZone msg;
  msg.request = req;
  ServiceHandleNdtCreateZone(msg.request, msg.response);
  res.message = "OK";
  res.status = msg.response.status;
  return true;
}
bool ParamManager::ServiceHandleNdtCreatePath(CreatePath::Request &req,
											  CreatePath::Response &res) {
  return true;
}  // TODO
bool ParamManager::ServiceHandleNdtCreateZone(CreateZone::Request &req,
											  CreateZone::Response &res) {
  return true;
}  // TODO

void ParamManager::HandleJudgeLocalizationLost(
	const LocalizationLostConstPtr &msg) {
	if (msg->lost) {
		geometry_msgs::Twist e_stop;
		e_stop_pub_.publish(e_stop);
	}
}

bool ParamManager::HandleWaypointUpload(OssUpload::Request &req,
										OssUpload::Response &res) {
  // 1. check selected map
  auto mapName = getSelectedMap();
  if (mapName.empty()) {
	responseHelper(res, -1,
				   "HandleWaypointUpload: selected map is not existed!!");
	return true;
  }

  auto planName = req.filename;
  if (planName.empty()) {
	responseHelper(res, -1, "HandleWaypointUpload: path name is empty!!");
	return true;
  }

  auto path_dir = BF::path(MAP_DIRECTORY + mapName + "/path");
  if (!BF::exists(path_dir)) BF::create_directories(path_dir);

  std::string yamlfile =
	  MAP_DIRECTORY + mapName + "/path/" + planName + ".yaml";
  std::string csvfile = MAP_DIRECTORY + mapName + "/path/" + planName + ".csv";

  std::ifstream inf(yamlfile);
  if (inf.is_open()) {
	std::string line;
	std::vector<std::string> lines_vec;
	while (std::getline(inf, line)) {
	  if (line.size()) lines_vec.emplace_back(line);
	}
	// 判断是否需要下采样
	auto line_num = lines_vec.size();
	if (line_num > ds_num) {
	  // downsample
	  std::ofstream out(MAP_DIRECTORY + mapName + "/path/" + planName +
		  "_ds.yaml");
	  for (int i = 0; i < ds_num; i++) {
		auto l = lines_vec.at(int(1.0 * i / ds_num * line_num));
		out << l << std::endl;
	  }
	  out.close();
	}
	// 选择下采样文件上传，但文件名还是原名
	auto& oss = OssUploadHelper::getInstance();
	oss.setMapId(mapName);

	std::string ud_fn =
		(line_num > ds_num ? planName + "_ds.yaml" : planName + ".yaml");
	if (!oss.uploadOssFile("path/" + ud_fn, "path/" + planName + ".yaml") ||
		!oss.uploadPathMetaInfo(planName)) {
	  LG->error("HandleCreateCleaningPlan: oss upload failed");
	  responseHelper(res, -1, "HandleCreateCleaningPlan: oss upload failed");
	  return true;
	}

  } else {
	responseHelper(res, -1, "yamlfile cant open!!");
	return true;
  }

  LG->trace("path upload success!!");
  responseHelper(res, 0, "path upload success!!");
  return true;

  LG->trace("path upload success!!");
  responseHelper(res, 0, "path upload success!!");
  return true;
}

bool ParamManager::HandleSemanticLayerSave(OssUpload::Request &req,
										   OssUpload::Response &res) {
  // 1. parse string to json
  // 2. check .robot/maps/map_name(jt_demo_1th)/layer.json
  // 3. generate unique id for object
  // 4. supplement json message to layer.json
  LG->trace("receive json string {}", req.filename);

  auto receive_json = json::parse(req.filename);

  std::string property = receive_json["property"];
  std::string type = receive_json["type"];
  std::string map_name = receive_json["map_name"];
  receive_json["message"]["id"] = generate_id();

  // ------------- trans data to maps -------------
  std::string yn = MAP_DIRECTORY + map_name + "/" + map_name + ".yaml";
  mit_.setYaml(yn);

  auto to_map_json = receive_json["message"];
  auto data = to_map_json["data"].get<std::vector<float>>();
// 进来的点数据务必是webapp坐标系的，即像素坐标系，并且向上为0，顺时针。
// 下面转成地图坐标系，角度为向右为0，逆时针。
  for (int i = 0; i < data.size() - 1; i += 2) {
	std::vector<float> d_img{data[i], data[i + 1]};
	auto d_map = mit_.imageToMap(d_img);
	data[i] = d_map[0];
	data[i + 1] = d_map[1];
  }
  if (type == "point") {
	if (std::isnan(data.back())) data.back() = 0;
	else {
	  data.back() = MapImageTrans::toRad(90 - data.back());
	}
  }
  to_map_json["data"] = data;
  // ------------- trans data to maps -------------

  std::string dist_path = MAP_DIRECTORY + map_name + "/map";

  if (!BF::exists(BF::path(dist_path))) {
	BF::create_directory(BF::path(dist_path));
  }

  json res_json, in_map_json;

  if (IsFileExistent(BF::path(dist_path + "/layer_show.json"))) {
	std::ifstream input(dist_path + "/layer_show.json");
	std::ifstream inf(dist_path + "/layer.json");
	if (!input)
	  LG->error("input open layer.json error");
	input >> res_json;
	inf >> in_map_json;
	input.close();
	inf.close();
  }

  res_json[property][type].push_back(receive_json["message"]);
  in_map_json[property][type].push_back(to_map_json);

  std::ofstream out(dist_path + "/layer_show.json");
  std::ofstream out_in_map(dist_path + "/layer.json");
  if (!out)
	LG->error("output open layer.json error");
  out << std::setw(4) << res_json;
  out_in_map << std::setw(4) << in_map_json;
  out.close();
  out_in_map.close();
  responseHelper(res, 0, "save layer.json success!");

  return true;
}

bool ParamManager::HandleSemanticLayerEdit(OssUpload::Request &req,
										   OssUpload::Response &res) {
  // 1. parse string to json
  // 2. check .robot/maps/map_name(jt_demo_1th)/layer.json
  // 3. locate object by property,type,and id
  // 4. change json message from layer.json
  auto receive_json = json::parse(req.filename.c_str());
  std::string property = receive_json["property"];
  std::string type = receive_json["type"];
  std::string map_name = receive_json["map_name"];
  std::string edit_id = receive_json["id"];
  std::string dist_path = MAP_DIRECTORY + map_name + "/map";

  if (!IsFileExistent(BF::path(dist_path + "/layer.json"))) {
	LG->error("there is no layer.json to find please create first!");

	responseHelper(res, -1, "there is no layer.json please create first!");
  }
  std::ifstream input(dist_path + "/layer_show.json");
  if (!input)
	LG->error("input open layer.json error");
  json res_json;
  input >> res_json;
  input.close();

  // edit layer.json

  for (auto &item : res_json[property][type]) {
	if (item["id"] == edit_id) {
	  item["name"] = receive_json["name"];
	  break;
	}
  }

  std::ofstream out(dist_path + "/layer_show.json");
  if (!out)
	LG->error("output open layer.json error");
  out << std::setw(4) << res_json;
  out.close();
  responseHelper(res, 0, "edit layer.json success!");

  return true;
}

bool ParamManager::HandleSemanticLayerDelete(OssUpload::Request &req,
											 OssUpload::Response &res) {
  // 1. parse string to json
  // 2. check .robot/maps/map_name(jt_demo_1th)/layer.json
  // 3. locate object by property,type,and id
  // 4. change json message from layer.json
  auto receive_json = json::parse(req.filename.c_str());
  std::string property = receive_json["property"];
  std::string type = receive_json["type"];
  std::string map_name = receive_json["map_name"];

  std::string delete_id = receive_json["id"];
  std::string dist_path = MAP_DIRECTORY + map_name + "/map";

  if (!BF::exists(BF::path(dist_path))) {
	BF::create_directory(BF::path(dist_path));
  }

  if (!IsFileExistent(BF::path(dist_path + "/layer.json"))) {
	LG->warn("there is no layer.json to find , please create first!");
	responseHelper(res, -1, "there is no layer.json please create first!");
  }

  std::ifstream input(dist_path + "/layer_show.json");
  std::ifstream inf(dist_path + "/layer.json");
  if (!input)
	LG->error("input open layer.json error");
  json res_json, in_map_json;
  input >> res_json;
  inf >> in_map_json;
  input.close();
  inf.close();

  // delete obj from layer_show.json

  auto &json_base_show = res_json[property][type];
  for (int i = 0; i < json_base_show.size(); i++) {
	if (json_base_show[i]["id"] == delete_id) {
	  json_base_show.erase(json_base_show.begin() + i);
	  if (!json_base_show.size()) {

		auto path = nlohmann::json_pointer<json>{"/" + property};

		res_json[path].erase(type);

	  }
	  LG->info("delete layer_show.json obj success");
	  break;
	}
  }

  // delete from layer.json

  auto &json_base = in_map_json[property][type];
  for (int i = 0; i < json_base.size(); i++) {
	if (json_base[i]["id"] == delete_id) {
	  json_base.erase(json_base.begin() + i);

	  if (!json_base.size()) {

		auto path = nlohmann::json_pointer<json>{"/" + property};

		in_map_json[path].erase(type);

	  }
	  LG->info("delete layer.json obj success");
	  break;
	}
  }

  if (!res_json[property].size())res_json.erase(property);
  if (!in_map_json[property].size())in_map_json.erase(property);

  std::ofstream out(dist_path + "/layer_show.json");
  std::ofstream out_in_map(dist_path + "/layer.json");
  if (!out)
	LG->error("output open layer.json error");
  out << std::setw(4) << res_json;
  out_in_map << std::setw(4) << in_map_json;

  out.close();
  out_in_map.close();
  responseHelper(res, 0, "delete from  layer.json success!");
  return true;
}

bool ParamManager::HandleZoningCoverage(OssUpload::Request &req,
										OssUpload::Response &res) {
  /// 1. determine the partition to coverage based on json
  /// 2. call the getting map info api according to the algorithm
  /// 3. call the coverage api then save path to coverage.csv
  /// 4. using helper funtions to stitch both coverage path and edge path into both.csv

  auto receive_json = json::parse(req.filename);
  std::string map_name = receive_json["map_name"];
  std::string algorithm_id = receive_json["algorithm"];
  std::string zone_name = receive_json["zone_id"];
  std::string sub_zone_name = receive_json["sub_zone_id"];
  std::string dist_file_path =
	  MAP_DIRECTORY + map_name + "/zone/" + zone_name + "/" + sub_zone_name;

  sensor_msgs::Image map;

  /***----------------------------------------helper funtions--------------------------------------***/

  /// judge whether need to stitch two path
  auto whether_create_path = [](float x1, float y1, float x2, float y2) {
	float dx = x1 - x2, dy = y1 - y2;
	return sqrt(dx * dx + dy * dy) > 0.5;
  };

  auto readPathFromCsv = [](const std::string &filepath,
							std::vector<geometry_msgs::Pose> &path) {
	std::ifstream input(filepath);

	if (!input) LG->error("ifstream open coverage.csv fail!");

	std::string line;
	std::string field;

	while (std::getline(input, line)) {
	  geometry_msgs::Pose pose;

	  std::vector<float> data;
	  std::istringstream sin(line);

	  while (std::getline(sin, field, ' ')) {
		data.push_back(std::stof(field));
	  }

	  if (data.size() < 6)
		LG->error("coverage csv format wrong,please check !");

	  pose.position.x = data[0];
	  pose.position.y = data[1];
	  pose.position.z = 0;
	  pose.orientation.x = data[2];
	  pose.orientation.y = data[3];
	  pose.orientation.z = data[4];
	  pose.orientation.w = data[5];

	  path.push_back(pose);
	}

	input.close();
  };

  /// merge one path to another
  auto path_add = [](std::vector<geometry_msgs::Pose> &path1,
					 const std::vector<geometry_msgs::Pose> &path2) {
	for (auto p : path2) path1.push_back(p);
  };

  /// @brief: the stitch path operations
  /// 1. judge whether need to joining together two path
  /// 2. if the first step requires,call the targetpoint_path_planner
  /// 3. then merge the resulting waypoints
  auto stitch_path =
	  [&](const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2,
		  std::vector<std::vector<geometry_msgs::Pose>> &task_paths,
		  std::vector<geometry_msgs::Pose> &res_path,
		  int index) {  // 用于路径拼接
		float end_x1, end_y1, start_x1, start_y1;
		end_x1 = p1.position.x;
		end_y1 = p1.position.y;
		start_x1 = p2.position.x;
		start_y1 = p2.position.y;
		if (whether_create_path(end_x1, end_y1, start_x1, start_y1)) {
		  TargetPoint srv;
		  srv.request.first_path.poses.push_back(poseToPoseStamed(p1));
		  srv.request.second_path.poses.push_back(poseToPoseStamed(p2));
		  srv.request.map_name = map_name;
		  // ROS_WARN(" srv.request.first_path.poses %lf %lf",
		  //          srv.request.first_path.poses[0].pose.position.x,
		  //          srv.request.first_path.poses[0].pose.position.y);
		  // ROS_WARN(" srv.request.second_path.poses %lf %lf",
		  //          srv.request.second_path.poses[0].pose.position.x,
		  //          srv.request.second_path.poses[0].pose.position.y);
		  if (targetpoint_planner_client_.call(srv)) {
			if (srv.response.status == 0) {
			  LG->trace("targetpoint_planner_client call success!");
			  path_add(res_path, task_paths[index]);
			  path_add(res_path, pathToPoses(srv.response.planner_path));
			} else {
			  LG->error("targetpoint_planner_client call failed({}): {}!!", srv.response.status,
					   srv.response.message);
			  return false;
			}
		  } else {
			LG->error("targetpoint_planner_client call defeat!");
			return false;
		  }
		} else {
		  LG->info("dont need stitch path!");
		}
		if (index + 1 < task_paths.size())
		  path_add(res_path, task_paths[index + 1]);
		return true;
	  };

  /***------------------------------------------------------------------------------***/

  /***   call map info api and coverage method service ***/

  json req_json;
  req_json["map_name"] = map_name;
  req_json["zone_name"] = zone_name;
  req_json["sub_zone_name"] = sub_zone_name;
  ZoneApiSrv zone_api_srv;
  zone_api_srv.request.cmd = req_json.dump();

  std::vector<geometry_msgs::Pose>
	  res_path;  /// save coverage method callback response path
  auto poses = receive_json["poses"];
  geometry_msgs::Pose start_pose, end_pose;
  start_pose = pointToPose(poses["start_point"][0], poses["start_point"][1]);
  end_pose = pointToPose(poses["end_point"][0], poses["end_point"][1]);

  if (algorithm_id == "1") {
	/// call coverage hui
	if (zone_api_hui_client_.call(zone_api_srv)) {
	  JTCoveragePlanning coverage_srv;

	  coverage_srv.request.map = zone_api_srv.response.image;

	  coverage_srv.request.start_pose = start_pose;

	  coverage_srv.request.end_pose = end_pose;

	  if (coverage_surround_client_.call(coverage_srv)) {
		res_path = pathToPoses(coverage_srv.response.planner_path);
		LG->trace("******* {} ******", coverage_srv.response.message);
	  } else {
		LG->error("coverage_surround_client call fail!");
		responseHelper(res, -1, "coverage_surround_client call fail!");
		return true;
	  }
	} else {
	  LG->error("call zone_api_hui fail!");
	  responseHelper(res, -1, "call zone_api_hui fail!");
	  return true;
	}

  } else if (algorithm_id == "2") {
	/// call coverage gong
	if (zone_api_gong_client_.call(zone_api_srv)) {
	  CoveragePointMap coverage_srv;
	  coverage_srv.request.start_pose = start_pose;
	  coverage_srv.request.end_pose = end_pose;
	  coverage_srv.request.vertex = zone_api_srv.response.corners;
	  coverage_srv.request.valid_start_pose = true;

	  if (convex_ploygon_client_.call(coverage_srv)) {
		res_path = pathToPoses(coverage_srv.response.planner_path);

		LG->trace("******* {} ******", coverage_srv.response.message);

	  } else {
		LG->error("convex_ploygon_client call fail!");

		responseHelper(res, -1, "convex_ploygon_client call fail!");

		return true;
	  }
	} else {
	  LG->error("call zone_api_gong fail!");

	  responseHelper(res, -1, "call zone_api_gong fail!");

	  return true;
	}
  }

  /***   save coverage planner path to directory path  ***/
  // if empty, then main zone, need to save both automatically!!
  if (sub_zone_name == "") {
	savePlanPath(dist_file_path + "coverage.csv", res_path);

	/***      stitch total coverage path and edge path into both.csv       ***/
	std::string dist_path = MAP_DIRECTORY + map_name + "/zone/" + zone_name + "/";

	geometry_msgs::Pose last_end_position, cur_start_position;

	std::vector<geometry_msgs::Pose> join_path;  /// save last both.csv path

	std::vector<geometry_msgs::Pose> edge_path;

	readPathFromCsv(dist_path + "edge.csv", edge_path);

	std::vector<std::vector<geometry_msgs::Pose>> paths;
	paths.push_back(edge_path);
	paths.push_back(res_path);

	if (edge_path.size() < 5 || res_path.size() < 5) {
	  responseHelper(res, -1, "one of path size is too little!!");
	  return true;
	}

	last_end_position = getPoseFromTwoPoint(edge_path[edge_path.size() - 2], edge_path.back());
	cur_start_position = getPoseFromTwoPoint(res_path[0], res_path[1]);

	if (!stitch_path(last_end_position, cur_start_position, paths, join_path, 0)) {
	  res.status = -1;
	  res.message = "stitch path defeat for main zone!!";
	  astar_start_pub_.publish(poseToPoseStamed(last_end_position));
	  astar_end_pub_.publish(poseToPoseStamed(cur_start_position));
	  return true;
	}

	savePlanPath(dist_path + "/both.csv", join_path);

  } else {
	savePlanPath(dist_file_path + "/coverage.csv", res_path);
  }
  LG->trace("**********     save coverage.csv success!      ***********");

  return true;
}

bool ParamManager::HandelZoningCoverageConsolidation(
	OssUpload::Request &req, OssUpload::Response &resp) {
  auto receive_json = json::parse(req.filename);
  std::string map_name = receive_json["map_name"];
  std::string zone_name = receive_json["zone_name"];
  std::string dist_path = MAP_DIRECTORY + map_name + "/zone/" + zone_name + "/";

  /***                          helper functions                      ***/

  /// judge whether need to stitch two path
  auto whether_create_path = [](float x1, float y1, float x2, float y2) {
	float dx = x1 - x2, dy = y1 - y2;
	return sqrt(dx * dx + dy * dy) > 0.2;
  };

  auto poseToPoseStamed = [](const geometry_msgs::Pose &p) {
	geometry_msgs::PoseStamped res;
	res.pose = p;
	res.header.stamp = ros::Time::now();
	res.header.frame_id = "map";
	return res;
  };

  auto readPathFromCsv = [](const std::string &filepath,
							std::vector<geometry_msgs::Pose> &path) {
	std::ifstream input(filepath);

	if (!input) LG->error("ifstream open coverage.csv fail!");

	std::string line;
	std::string field;

	while (std::getline(input, line)) {
	  geometry_msgs::Pose pose;

	  std::vector<float> data;
	  std::istringstream sin(line);

	  while (std::getline(sin, field, ' ')) {
		data.push_back(std::stof(field));
	  }

	  if (data.size() < 6)
		LG->error("coverage csv format wrong,please check !");

	  pose.position.x = data[0];
	  pose.position.y = data[1];
	  pose.position.z = 0;
	  pose.orientation.x = data[2];
	  pose.orientation.y = data[3];
	  pose.orientation.z = data[4];
	  pose.orientation.w = data[5];

	  path.push_back(pose);
	}

	input.close();
  };

  /// merge one path to another
  auto path_add = [](std::vector<geometry_msgs::Pose> &path1,
					 const std::vector<geometry_msgs::Pose> &path2) {
	for (auto p : path2) path1.push_back(p);
  };

  /// @brief: the stitch path operations
  /// 1. judge whether need to joining together two path
  /// 2. if the first step requires,call the targetpoint_path_planner
  /// 3. then merge the resulting waypoints
  auto stitch_path =
	  [&](const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2,
		  std::vector<std::vector<geometry_msgs::Pose>> &task_paths,
		  std::vector<geometry_msgs::Pose> &res_path,
		  int index) {  // 用于路径拼接
		float end_x1, end_y1, start_x1, start_y1;
		end_x1 = p1.position.x;
		end_y1 = p1.position.y;
		start_x1 = p2.position.x;
		start_y1 = p2.position.y;
		if (whether_create_path(end_x1, end_y1, start_x1, start_y1)) {
		  TargetPoint srv;
		  srv.request.first_path.poses.push_back(poseToPoseStamed(p1));
		  srv.request.second_path.poses.push_back(poseToPoseStamed(p2));
		  srv.request.map_name = map_name;
		  // ROS_WARN(" srv.request.first_path.poses %lf %lf",
		  //          srv.request.first_path.poses[0].pose.position.x,
		  //          srv.request.first_path.poses[0].pose.position.y);
		  // ROS_WARN(" srv.request.second_path.poses %lf %lf",
		  //          srv.request.second_path.poses[0].pose.position.x,
		  //          srv.request.second_path.poses[0].pose.position.y);
		  if (targetpoint_planner_client_.call(srv)) {
			if (srv.response.status == 0) {
			  LG->trace("targetpoint_planner_client call success!");
			  path_add(res_path, task_paths[index]);
			  path_add(res_path, pathToPoses(srv.response.planner_path));
			} else {
			  LG->error("targetpoint_planner_client call failed({}): {}!!", srv.response.status,
					   srv.response.message);
			  return false;
			}
		  } else {
			LG->error("targetpoint_planner_client call defeat!");
			return false;
		  }
		} else {
		  LG->error("dont need stitch path!");
		}
		if (index + 1 < task_paths.size())
		  path_add(res_path, task_paths[index + 1]);
		return true;
	  };

  /***    read coverage.csv from sub_zones dir to vector pathes       ***/

  std::vector<std::vector<geometry_msgs::Pose>>
	  task_paths;  /// save path from coverage.csv

  auto json_base = receive_json["sub_zones"];

  for (auto &sub : json_base) {
	std::string sub_zone = sub["sub_zone_name"];

	std::vector<geometry_msgs::Pose> cur_path;

	readPathFromCsv(dist_path + sub_zone + "/coverage.csv", cur_path);

	task_paths.push_back(cur_path);
  }

  /***         then call the targetpoint_planner method and stitch task_paths
   * ***/

  std::vector<geometry_msgs::Pose> res_path;  /// save stitched res path

  if (task_paths.size() < 2) {
	LG->warn("dont need stich path, please give more than tow paths!!");
	responseHelper(resp, 0,
				   "dont need stich path , please give more than two paths!!");
  };

  geometry_msgs::Pose last_end_position;   /// end position of last path
  geometry_msgs::Pose cur_start_position;  /// start position of cur path

  /// 拼接路径
  for (int i = 0; i + 1 < task_paths.size(); i++) {
	const auto &last_paths = task_paths[i];
	const auto &cur_paths = task_paths[i + 1];

	if (last_paths.size() < 5 || cur_paths.size() < 5) {
	  responseHelper(resp, -1, "one of path size is too little!!");
	  return true;
	}

	last_end_position = getPoseFromTwoPoint(last_paths[last_paths.size() - 2], last_paths.back());
	cur_start_position = getPoseFromTwoPoint(cur_paths[0], cur_paths[1]);

	if (!stitch_path(last_end_position, cur_start_position, task_paths,
					 res_path, i)) {
	  resp.status = -1;
	  resp.message = "stitch path defeat in for loop!";
	  astar_start_pub_.publish(poseToPoseStamed(last_end_position));
	  astar_end_pub_.publish(poseToPoseStamed(cur_start_position));
	  return true;
	}
  }

  savePlanPath(dist_path + "coverage.csv", res_path);

  /***      stitch total coverage path and edge path into both.csv       ***/

  std::vector<geometry_msgs::Pose> join_path;  /// save last both.csv path

  std::vector<std::vector<geometry_msgs::Pose>> paths;

  std::vector<geometry_msgs::Pose> edge_path;

  readPathFromCsv(dist_path + "edge.csv", edge_path);

  paths.push_back(edge_path);
  paths.push_back(res_path);

  for (int i = 0; i + 1 < paths.size(); i++) {
	last_end_position = paths[i][paths[i].size() - 1];
	cur_start_position = paths[i + 1][0];
	if (!stitch_path(last_end_position, cur_start_position, paths, join_path,
					 i)) {
	  resp.status = -1;
	  resp.message = "stitch path defeat in for loop!";
	  LG->trace("point1: ({}, {}, {}, {}, {}, {})", last_end_position.position.x,
			   last_end_position.position.y, last_end_position.orientation.x,
			   last_end_position.orientation.y, last_end_position.orientation.z,
			   last_end_position.orientation.w);
	  LG->trace(
		  "point2: ({}, {}, {}, {}, {}, {})", cur_start_position.position.x,
		  cur_start_position.position.y, cur_start_position.orientation.x,
		  cur_start_position.orientation.y, cur_start_position.orientation.z,
		  cur_start_position.orientation.w);
	  return true;
	}
  }

  savePlanPath(dist_path + "/both.csv", join_path);

  return true;
}

std::vector<geometry_msgs::Pose> ParamManager::XmlRpcToPoses(
	const XmlRpc::XmlRpcValue &ps) {
  std::vector<geometry_msgs::Pose> res;
  for (int i = 0; i < ps.size(); i++) {
	geometry_msgs::Pose pose;
	pose.position.x = ps[i]["position"]["x"];
	pose.position.y = ps[i]["position"]["y"];
	pose.position.z = ps[i]["position"]["z"];
	pose.orientation.x = ps[i]["orientation"]["x"];
	pose.orientation.y = ps[i]["orientation"]["y"];
	pose.orientation.z = ps[i]["orientation"]["z"];
	pose.orientation.w = ps[i]["orientation"]["w"];
	res.push_back(pose);
  }
  return res;
}

geometry_msgs::Pose ParamManager::pointToPose(float x, float y) {
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0;

  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1.0;
  return pose;
}

geometry_msgs::PoseStamped ParamManager::poseToPoseStamped(
	const geometry_msgs::Pose &p) {
  geometry_msgs::PoseStamped res;
  res.pose = p;
  return res;
}

bool ParamManager::savePlanPath(const std::string &pathname,
								const vector<geometry_msgs::Pose> &pose) {
  std::ofstream out;

  out.open(pathname);

  if (out.fail()) {
	LG->error("save plan path {} fail!", pathname);
	out.close();
	return false;
  }
  for (auto &p : pose) {
	out << p.position.x << " " << p.position.y << " " << p.orientation.x << " "
		<< p.orientation.y << " " << p.orientation.z << " " << p.orientation.w
		<< std::endl;
  }

  LG->trace("save plan path {} success!", pathname);
  out.close();

  return true;
}

std::vector<geometry_msgs::Pose> ParamManager::pathToPoses(nav_msgs::Path &path) {
  std::vector<geometry_msgs::Pose> pose;
  for (auto it : path.poses) {
	pose.push_back(it.pose);
  }
  return pose;
}

bool ParamManager::settingHandler(AgentRequest& req, AgentResponse& res){
	res.trace_id = req.trace_id;

	if(req.service_id == "voice"){
		setVoiceValue(req, res);
	}else if(req.service_id == "battery_threshold"){
		setBatteryThreshold(req, res);
	}else if(req.service_id == "rubber"){
		setRubType(req, res);
	}else{
		agentHelper(jlw_for_param_manager->logger, res, "invalid service id", -1);
	}

	return true;
}

void ParamManager::setRubType(AgentRequest& req, AgentResponse& res){
	std::string mode = json::parse(req.data)["mode"];
	MappingSave ms;
	ms.request.filename = mode;
	if(ros::service::call("/auto_brush/set_cleaning_type", ms)){
		if(ms.response.status == 0){
			agentHelper(jlw_for_param_manager->logger, res, "ok", 0);
		}
		agentHelper(jlw_for_param_manager->logger, res, ms.response.message, ms.response.status);
	}else{
		agentHelper(jlw_for_param_manager->logger, res, "auto-brush offline!!", -1);
	}
}

void ParamManager::setVoiceValue(AgentRequest& req, AgentResponse& res){
	int voice = json::parse(req.data)["value"];

	VolumeChange vc;
	vc.request.volume = voice;
	if(ros::service::call("/VolumeChange", vc)){
		if(vc.response.status == 0){
			ros::param::set("/param/voice", voice);
			agentHelper(jlw_for_param_manager->logger, res, "ok", 0);
		}
		agentHelper(jlw_for_param_manager->logger, res, vc.response.message, vc.response.status);
	}else{
		agentHelper(jlw_for_param_manager->logger, res, "soundplay offline!!", -1);
	}
}

void ParamManager::setBatteryThreshold(AgentRequest& req, AgentResponse& res){
	// check msg is nan
	json j = json::parse(req.data);
	int value = j["value"];

	if (std::isnan(value)) {
		agentHelper(jlw_for_param_manager->logger, res, "Set Battery Threshlod invaild", -1);
		return;
	}

	// assign the msg to threshold
	value = std::min(value, 80);
	ros::param::set("/param/battery_threshold", value);
	agentHelper(jlw_for_param_manager->logger, res, "ok", 0);
}