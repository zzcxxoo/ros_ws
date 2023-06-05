//
// Created by jtcx on 10/19/22.
//
#include "JtcxCommon.hpp"
#include "ros/ros.h"
#include "JtcxExternalEventsBase.hpp"
#include "JtcxRobotStatesBase.hpp"

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

#include <mobile_platform_msgs/DockingStart.h>
#include <mobile_platform_msgs/GoToChargingPoint.h>
#include <mobile_platform_msgs/PathPlanningExecute.h>
#include <mobile_platform_msgs/PathPlanningRoute.h>
#include <mobile_platform_msgs/PatrolStart.h>
#include <mobile_platform_msgs/PatrolStop.h>
#include <mobile_platform_msgs/Pursuit.h>
#include <mobile_platform_msgs/ConvexPolygon.h>
#include <mobile_platform_msgs/TargetPoint.h>
#include <mobile_platform_msgs/PassingPoint.h>
#include <mobile_platform_msgs/MappingSave.h>
#include <mobile_platform_msgs/SystemdService.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <pwd.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

using mobile_platform_msgs::DockingStart;
using mobile_platform_msgs::PatrolStart;
using mobile_platform_msgs::PatrolStop;
using mobile_platform_msgs::Pursuit;
using mobile_platform_msgs::MappingSave;
using mobile_platform_msgs::SystemdService;

using mobile_platform_msgs::ConvexPolygon;
using mobile_platform_msgs::PassingPoint;
using mobile_platform_msgs::TargetPoint;

class ExternalLowFrequencyEventReceiver {

 public:

  ~ExternalLowFrequencyEventReceiver() {
	_ros_thread->join();
  }
  static std::unique_ptr<ExternalLowFrequencyEventReceiver> &getinstance() {
	return _pExternalLowFrequencyEventReceiver;
  };
  const SystemdService::Request &GetStartMappingReq() const {
	return _start_mapping_req;
  }
  const SystemdService::Request &GetStopMappingReq() const {
	return _stop_mapping_req;
  }
  const MappingSave::Request &GetSaveMappingReq() const {
	return _save_mapping_req;
  }
  const SystemdService::Request &GetStartNavigationReq() const {
	return _start_navigation_req;
  }
  const SystemdService::Request &GetStopNavigationReq() const {
	return _stop_navigation_req;
  }
  const SystemdService::Request &GetStartPathrecordReq() const {
	return _start_pathrecord_req;
  }
  const PatrolStart::Request &GetStartPathPlanningPursuitReq() const {
	return _start_path_planning_pursuit_req;
  }
  const PatrolStop::Request &GetStopPathPlanningPursuitReq() const {
	return _stop_path_planning_pursuit_req;
  }
  const ConvexPolygon::Request &GetConvexPolygonGenerateReq() const {
	return _convex_polygon_generate_req;
  }
  const PassingPoint::Request &GetPassPointPlannerReq() const {
	return _pass_point_planner_req;
  }
  const TargetPoint::Request &GetTargetPointPlannerReq() const {
	return _target_point_planner_req;
  }

 public:
  void SetStartMappingRes(const SystemdService::Response &start_mapping_res) {
	_start_mapping_res = start_mapping_res;
  }
  void SetStopMappingRes(const SystemdService::Response &stop_mapping_res) {
	_stop_mapping_res = stop_mapping_res;
  }
  void SetSaveMappingRes(const MappingSave::Response &save_mapping_res) {
	_save_mapping_res = save_mapping_res;
  }
  void SetStartNavigationRes(const SystemdService::Response &start_navigation_res) {
	_start_navigation_res = start_navigation_res;
  }
  void SetStopNavigationRes(const SystemdService::Response &stop_navigation_res) {
	_stop_navigation_res = stop_navigation_res;
  }
  void SetStartPathrecordRes(const SystemdService::Response &start_pathrecord_res) {
	_start_pathrecord_res = start_pathrecord_res;
  }
  void SetStartPathPlanningPursuitRes(const PatrolStart::Request &start_path_planning_pursuit_res) {
	_start_path_planning_pursuit_res = start_path_planning_pursuit_res;
  }
  void SetStopPathPlanningPursuitRes(const PatrolStop::Request &stop_path_planning_pursuit_res) {
	_stop_path_planning_pursuit_res = stop_path_planning_pursuit_res;
  }
  void SetConvexPolygonGenerateRes(const ConvexPolygon::Request &convex_polygon_generate_res) {
	_convex_polygon_generate_res = convex_polygon_generate_res;
  }
  void SetPassPointPlannerRes(const PassingPoint::Response &pass_point_planner_res) {
	_pass_point_planner_res = pass_point_planner_res;
  }
  void SetTargetPointPlannerRes(const TargetPoint::Response &target_point_planner_res) {
	_target_point_planner_res = target_point_planner_res;
  }
 public:
  std::promise<bool> &GetIsMappingStartCompleted() {
	return _is_mapping_start_completed;
  }
  std::promise<bool> &GetIsMappingStopCompleted() {
	return _is_mapping_stop_completed;
  }
  std::promise<bool> &GetIsMappingSaveCompleted() {
	return _is_mapping_save_completed;
  }
  std::promise<bool> &GetIsNavigationStartCompleted() {
	return _is_navigation_start_completed;
  }
  std::promise<bool> &GetIsNavigationStopCompleted() {
	return _is_navigation_stop_completed;
  }
  std::promise<bool> &GetIsPathrecordStartCompleted() {
	return _is_pathrecord_start_completed;
  }
  std::promise<bool> &GetIsPathPlanningStartPursuitCompleted() {
	return _is_path_planning_start_pursuit_completed;
  }
  std::promise<bool> &GetIsPathPlanningStopPursuitCompleted() {
	return _is_path_planning_stop_pursuit_completed;
  }
  std::promise<bool> &GetIsConvexPolygonGenerateCompleted() {
	return _is_convex_polygon_generate_completed;
  }
  std::promise<bool> &GetIsPassPointPlannerCompleted() {
	return _is_pass_point_planner_completed;
  }
  std::promise<bool> &GetIsTargetPointPlannerCompleted() {
	return _is_target_point_planner_completed;
  }
 private:
  ExternalLowFrequencyEventReceiver();

  ExternalLowFrequencyEventReceiver(const ExternalLowFrequencyEventReceiver &) = delete;
  ExternalLowFrequencyEventReceiver &operator=(const ExternalLowFrequencyEventReceiver &) = delete;
  static std::unique_ptr<ExternalLowFrequencyEventReceiver> _pExternalLowFrequencyEventReceiver;
  std::unique_ptr<std::thread> _ros_thread;   /// < external event listener thread pointer

  static void RosHandler() {
	ros::MultiThreadedSpinner spinner(2);  /// < 2 thread spin for high time consuming events
	ros::spin(spinner);
  }

 private:
  /*** ros servers ***/
  ros::NodeHandle _nh;
  ros::ServiceServer _ui_start_mapping;
  ros::ServiceServer _ui_stop_mapping;
//		ros::ServiceServer _ui_mapping_extend; /// TODO
  ros::ServiceServer _ui_save_mapping;
  ros::ServiceServer _ui_start_navigation;
  ros::ServiceServer _ui_stop_navigation;
//		ros::ServiceServer _ui_relocalize;   /// TODO
//		ros::Publisher _ui_start_autodocking;  /// publisher???
//		ros::Publisher _ui_stop_autodocking;
//		ros::ServiceServer _ui_upload_mapping; ///whether need
  ros::ServiceServer _ui_start_pathrecord;
  ros::ServiceServer _ui_path_planning_start_pursuit;
  ros::ServiceServer _ui_path_planning_stop_pursuit;
  ros::ServiceServer _ui_convex_polygon_generate;
  ros::ServiceServer _ui_pass_point_planner;
  ros::ServiceServer _ui_target_point_planner;

  /*** ros callback functions ***/
  bool HandleMappingStart(SystemdService::Request &req, SystemdService::Response &res);
  bool HandleMappingStop(SystemdService::Request &req, SystemdService::Response &res);
  bool HandleMappingExtend();
  bool HandleMappingSave(MappingSave::Request &req, MappingSave::Response &res);
  bool HandleStartNavigation(SystemdService::Request &req, SystemdService::Response &res);
  bool HandleStopNavigation(SystemdService::Request &req, SystemdService::Response &res);
  bool HandleRelocalize();
//		bool HandleStartAutodocking(std_msgs::Bool);
//		bool HandleStopAutodocking();
//		bool HandleMappingUpload(OssUpload::Request &req, OssUpload::Response &res);
  bool HandleStartPathrecord(SystemdService::Request &req, SystemdService::Response &res);
  bool HandlePathPlanningStartPursuit(PatrolStart::Request &req, PatrolStart::Response &res);
  bool HandlePathPlanningStopPursuit(PatrolStop::Request &req, PatrolStop::Response &res);
  bool HandleConvexPloygonGenerate(ConvexPolygon::Request &req, ConvexPolygon::Response &res);
  bool HandlePassPointPlanner(PassingPoint::Request &req, PassingPoint::Response &res);
  bool HandleTargetPointPlanner(TargetPoint::Request &req, PassingPoint::Response &res);

  /*** Stores User request informations for calling by exectors  ***/
  SystemdService::Request _start_mapping_req;
  SystemdService::Response _start_mapping_res;
  SystemdService::Request _stop_mapping_req;
  SystemdService::Response _stop_mapping_res;
  MappingSave::Request _save_mapping_req;
  MappingSave::Response _save_mapping_res;
  SystemdService::Request _start_navigation_req;
  SystemdService::Response _start_navigation_res;
  SystemdService::Request _stop_navigation_req;
  SystemdService::Response _stop_navigation_res;
  SystemdService::Request _start_pathrecord_req;
  SystemdService::Response _start_pathrecord_res;
  PatrolStart::Request _start_path_planning_pursuit_req;
  PatrolStart::Request _start_path_planning_pursuit_res;
  PatrolStop::Request _stop_path_planning_pursuit_req;
  PatrolStop::Request _stop_path_planning_pursuit_res;
  ConvexPolygon::Request _convex_polygon_generate_req;
  ConvexPolygon::Request _convex_polygon_generate_res;
  PassingPoint::Request _pass_point_planner_req;
  PassingPoint::Response _pass_point_planner_res;
  TargetPoint::Request _target_point_planner_req;
  TargetPoint::Response _target_point_planner_res;

  /*** when the exector completes its operation,it notifies the receiver ***/
  std::promise<bool> _is_mapping_start_completed;
  std::promise<bool> _is_mapping_stop_completed;
  std::promise<bool> _is_mapping_save_completed;
  std::promise<bool> _is_navigation_start_completed;
  std::promise<bool> _is_navigation_stop_completed;
  std::promise<bool> _is_pathrecord_start_completed;
  std::promise<bool> _is_path_planning_start_pursuit_completed;
  std::promise<bool> _is_path_planning_stop_pursuit_completed;
  std::promise<bool> _is_convex_polygon_generate_completed;
  std::promise<bool> _is_pass_point_planner_completed;
  std::promise<bool> _is_target_point_planner_completed;

};