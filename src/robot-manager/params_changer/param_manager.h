/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <actionlib/action_definition.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <dynamic_reconfigure/client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <mobile_platform_msgs/BaseStatus.h>
#include <mobile_platform_msgs/ChargingLoad.h>
#include <mobile_platform_msgs/ChargingSave.h>
#include <mobile_platform_msgs/ConvexPolygon.h>
#include <mobile_platform_msgs/CreatePath.h>
#include <mobile_platform_msgs/CreateZone.h>
#include <mobile_platform_msgs/InitPoseLoad.h>
#include <mobile_platform_msgs/InitPoseSave.h>
#include <mobile_platform_msgs/LocalizationLost.h>
#include <mobile_platform_msgs/OssUpload.h>
#include <mobile_platform_msgs/PathPlanningDelete.h>
#include <mobile_platform_msgs/PathPlanningList.h>
#include <mobile_platform_msgs/PathPlanningLoad.h>
#include <mobile_platform_msgs/PathPlanningSave.h>
#include <mobile_platform_msgs/TargetPoint.h>
#include <mobile_platform_msgs/PassingPoint.h>
#include <mobile_platform_msgs/ScanMatchingStatus.h>
#include <mobile_platform_msgs/VirtualLineLoad.h>
#include <mobile_platform_msgs/VirtualLineSave.h>
#include <mobile_platform_msgs/WaypointLoad.h>
#include <mobile_platform_msgs/WaypointSave.h>
#include <mobile_platform_msgs/ZoneApiSrv.h>
#include <mobile_platform_msgs/JTCoveragePlanning.h>
#include <mobile_platform_msgs/CoveragePointMap.h>
#include <mobile_platform_msgs/CreateZone.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <mobile_platform_msgs/MappingSelect.h>
#include <mobile_platform_msgs/Agent.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pwd.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <sensor_msgs/Image.h>
#include <stdio.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "xmlrpcpp/XmlRpcException.h"
#include "common/common.h"
#include "common/MapImageTrans.hpp"
#include "common/JtcxLogWrapper.hpp"

using mobile_platform_msgs::BaseStatus;
using mobile_platform_msgs::ChargingLoad;
using mobile_platform_msgs::ChargingSave;
using mobile_platform_msgs::CreatePath;
using mobile_platform_msgs::CreateZone;
using mobile_platform_msgs::InitPoseLoad;
using mobile_platform_msgs::InitPoseSave;
using mobile_platform_msgs::LocalizationLostConstPtr;
using mobile_platform_msgs::OssUpload;
using mobile_platform_msgs::PathPlanningDelete;
using mobile_platform_msgs::PathPlanningList;
using mobile_platform_msgs::PathPlanningLoad;
using mobile_platform_msgs::PathPlanningSave;
using mobile_platform_msgs::ScanMatchingStatus;
using mobile_platform_msgs::ScanMatchingStatusConstPtr;
using mobile_platform_msgs::VirtualLineLoad;
using mobile_platform_msgs::VirtualLineSave;
using mobile_platform_msgs::WaypointLoad;
using mobile_platform_msgs::WaypointSave;
using mobile_platform_msgs::ZoneApiSrv;
using mobile_platform_msgs::JTCoveragePlanning;
using mobile_platform_msgs::CoveragePointMap;

using mobile_platform_msgs::ConvexPolygon;
using mobile_platform_msgs::PassingPoint;
using mobile_platform_msgs::TargetPoint;

using namespace mobile_platform_msgs;

extern JtcxLogWrapper* jlw_for_param_manager;
#define LG jlw_for_param_manager->logger

class ParamManager {
 public:
  void Init(ros::NodeHandle *);

  bool HandleInitialPoseSave(InitPoseSave::Request &req,
							 InitPoseSave::Response &res);
  bool HandleInitialPoseLoad(InitPoseLoad::Request &req,
							 InitPoseLoad::Response &res);
  bool SaveRosParamToFile(
	  const std::string& param_name,
	  const std::string& file_name); // save map related parameters to files in the map folder
  bool LoadRosParamFromFile(
	  const std::string& param_name,
	  const std::string& file_name); // load map related parameters from files in the
  // map folder
  bool HandleWaypointSave(WaypointSave::Request &req,
						  WaypointSave::Response &res);
  bool HandleWaypointLoad(WaypointLoad::Request &req,
						  WaypointLoad::Response &res);
  bool HandleWaypointUpload(OssUpload::Request &req, OssUpload::Response &res);
  bool HandleChargingSave(ChargingSave::Request &req,
						  ChargingSave::Response &res);
  bool HandleChargingLoad(ChargingLoad::Request &req,
						  ChargingLoad::Response &res);
  bool HandleVirtualLineSave(VirtualLineSave::Request &req,
							 VirtualLineSave::Response &res);
  bool HandleVirtualLineLoad(VirtualLineLoad::Request &req,
							 VirtualLineLoad::Response &res);
  bool GetPathList(PathPlanningList::Request &req,
				   PathPlanningList::Response &res);
  bool HandlePathLoad(PathPlanningLoad::Request &req,
					  PathPlanningLoad::Response &res);
  bool HandlePathSave(PathPlanningSave::Request &req,
					  PathPlanningSave::Response &res);
  bool HandlePathDelete(PathPlanningDelete::Request &req,
						PathPlanningDelete::Response &res);
  bool HelpDelete(std::string filename);
  bool HandleBaseStatus(BaseStatus::Request &req, BaseStatus::Response &res);
  void HandleAmclStatus(std_msgs::Int32 amcl_status);
  void HandleRobotPoseOdom(nav_msgs::OdometryConstPtr robot_pose);
  bool HandleCreatePath(CreatePath::Request &req, CreatePath::Response &res);
  bool HandleCreateZone(CreateZone::Request &req, CreateZone::Response &res);

  bool ServiceHandleNdtCreatePath(CreatePath::Request &req,
								  CreatePath::Response &res);
  bool ServiceHandleNdtCreateZone(CreateZone::Request &req,
								  CreateZone::Response &res);
  bool HandleCreateCleaningPlan(mobile_platform_msgs::MappingSelectRequest &req,
								mobile_platform_msgs::MappingSelectResponse &resp);

  void HandleJudgeLocalizationLost(const LocalizationLostConstPtr &msg);

  bool HandleSemanticLayerSave(OssUpload::Request &req, OssUpload::Response &res);

  bool HandleSemanticLayerEdit(OssUpload::Request &req, OssUpload::Response &res);
  bool HandleSemanticLayerDelete(OssUpload::Request &req, OssUpload::Response &res);

  bool HandleZoningCoverage(OssUpload::Request &req, OssUpload::Response &res);

  bool HandelZoningCoverageConsolidation(OssUpload::Request &req, OssUpload::Response &res);
  std::vector<geometry_msgs::Pose> XmlRpcToPoses(const XmlRpc::XmlRpcValue &ps);

  geometry_msgs::Pose pointToPose(float x, float y);

  geometry_msgs::PoseStamped poseToPoseStamped(const geometry_msgs::Pose &p);

  bool savePlanPath(const std::string &pathname, const std::vector<geometry_msgs::Pose> &pose);

  std::vector<geometry_msgs::Pose> pathToPoses(nav_msgs::Path &);


  // app setting
  bool settingHandler(AgentRequest& req, AgentResponse& res);
  void setRubType(AgentRequest&, AgentResponse&);
  void setVoiceValue(AgentRequest& req, AgentResponse& res);
  void setBatteryThreshold(AgentRequest& req, AgentResponse& res);

 private:
  ros::NodeHandle *nh_ptr_;
  ros::Publisher astar_start_pub_;
  ros::Publisher astar_end_pub_;

  // Servers
  ros::ServiceServer ui_waypoint_save_;
  ros::ServiceServer ui_waypoint_load_;
  ros::ServiceServer ui_initpose_save_;
  ros::ServiceServer ui_initpose_load_;
  ros::ServiceServer ui_charging_save_;
  ros::ServiceServer ui_charging_load_;
  ros::ServiceServer ui_virtualline_save_;
  ros::ServiceServer ui_virtualline_load_;
  ros::ServiceServer ui_path_planning_list_;
  ros::ServiceServer ui_path_planning_save_;
  ros::ServiceServer ui_path_planning_load_;
  ros::ServiceServer ui_path_planning_delete_;
  ros::ServiceServer ui_system_state_;
  ros::ServiceServer ui_waypoint_upload_;

  ros::ServiceServer ui_semantic_layer_save_;
  ros::ServiceServer ui_semantic_layer_edit_;
  ros::ServiceServer ui_semantic_layer_delete_;
  ros::ServiceServer ui_zoning_coverage;
  ros::ServiceServer ui_zoning_coverage_consolidation;
  
  ros::ServiceServer ui_setting_;

  // Clients
  ros::ServiceClient convex_ploygon_client_;
  ros::ServiceClient coverage_surround_client_;
  ros::ServiceClient passpoint_planner_client_;
  ros::ServiceClient targetpoint_planner_client_;
  ros::ServiceClient zone_api_hui_client_;
  ros::ServiceClient zone_api_gong_client_;

  // Subscribers
  ros::Subscriber amcl_amcl_status_;
  ros::Subscriber ndt_pose_;
  ros::Subscriber odom_;
  ros::Subscriber scan_status_;
  ros::Subscriber ui_localization_lost_;
  ros::ServiceServer ui_create_cleaningplan_;

  // Publishers
  ros::Publisher amcl_initpose_pub_;
  ros::Publisher service_update_virtual_line_;
  ros::Publisher loc_stat_pub_;
  ros::Publisher e_stop_pub_;

  // flags
  bool g_patrolling_;
  std::string g_navifation_state_;
  std::string g_autodocking_state_;
  bool init_pose_saved_;

  // Pose
  geometry_msgs::Pose pose;
  geometry_msgs::PoseWithCovarianceStamped initpose;

  // filename
  std::vector<std::string> last_path_files;

  MapImageTrans mit_;

};
