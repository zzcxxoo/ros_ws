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
#include <mobile_platform_msgs/Chassis.h>
#include <mobile_platform_msgs/DockingStart.h>
#include <mobile_platform_msgs/DriveCommand.h>
#include <mobile_platform_msgs/GoToChargingPoint.h>
#include <mobile_platform_msgs/GoToCoordinate.h>
#include <mobile_platform_msgs/GoToWaypoint.h>
#include <mobile_platform_msgs/PathPlanningExecute.h>
#include <mobile_platform_msgs/PathPlanningRoute.h>
#include <mobile_platform_msgs/PatrolStart.h>
#include <mobile_platform_msgs/PatrolStop.h>
#include <mobile_platform_msgs/Pursuit.h>
#include <mobile_platform_msgs/SpeedControl.h>
#include <mobile_platform_msgs/SystemdService.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <mobile_platform_msgs/Agent.h>

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
#include <stdio.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <common/JtcxLogWrapper.hpp>

using mobile_platform_msgs::DockingStart;
using mobile_platform_msgs::DriveCommand;
using mobile_platform_msgs::GoToChargingPoint;
using mobile_platform_msgs::GoToCoordinate;
using mobile_platform_msgs::GoToWaypoint;
using mobile_platform_msgs::PathPlanningExecute;
using mobile_platform_msgs::PathPlanningRoute;
using mobile_platform_msgs::PatrolStart;
using mobile_platform_msgs::PatrolStop;
using mobile_platform_msgs::Pursuit;
using mobile_platform_msgs::SpeedControl;
using mobile_platform_msgs::SystemdService;

using namespace mobile_platform_msgs;

/* struct navigation status (rosparam)*/
struct NavigationStatus
{
	bool is_moving;
	std::unordered_map<std::string, std::vector<std::pair<std::string, int32_t>>>
		last_coordinate;
	std::unordered_map<std::string, std::vector<std::pair<std::string, int32_t>>>
		cur_coordinate;
	int32_t last_waypoint;
	int32_t last_status;
	int32_t cur_waypoint;
	int32_t goal_counter;
	NavigationStatus()
		: is_moving(false),
		  last_waypoint(-1),
		  last_status(-1),
		  cur_waypoint(-1),
		  goal_counter(0) {}
};

class WaypointEntry
{
public:
	geometry_msgs::Pose pose_;
	WaypointEntry();
	WaypointEntry(geometry_msgs::Pose);
};

class WaypointManager
{
public:
	void Init(ros::NodeHandle *);
	void CallAutoDockingStart();
	WaypointManager() : move_base_("move_base", true) {}
	void ClearCostmap();
	void ResetPatrol();
	void SetCurrentGoalPose(const geometry_msgs::Pose &pose,
							std_msgs::Int32 index);
	void SetCurrentGoalCoordinate(GoToCoordinate::Request &req,
								  GoToCoordinate::Response &res);
	bool SetCurrentGoalToChargingPoint();
	bool GoToWaypoint(std_msgs::Int32 no);
	void GoToCoordinate(GoToCoordinate::Request &req,
						GoToCoordinate::Response &res);
	bool GoToChargingPoint(GoToChargingPoint::Request &req,
						   GoToChargingPoint::Response &res);
	void HandleReloadWaypoint(std_msgs::String req);

	void GoToWaypointFailed();
	void UpdateInternalFlags();
	void UpdateMovebaseResult(
		const move_base_msgs::MoveBaseActionResultConstPtr &msg);
	void UpdateMovebaseStatus(
		const actionlib_msgs::GoalStatusArrayConstPtr &data);
	void UpdateMovebaseSpeed(const geometry_msgs::TwistConstPtr &data);
	void UpdateBattery(const mobile_platform_msgs::ChassisConstPtr &msg);
	bool HandleGoToCoordinate(GoToCoordinate::Request &req,
							  GoToCoordinate::Response &res);
	bool HandleGoToWaypoint(GoToWaypoint::Request &req,
							GoToWaypoint::Response &res);

	bool HandlePathTracking(Pursuit::Request &req, Pursuit::Response &res);
	bool HandleGoToChargingPoint(GoToChargingPoint::Request &req,
								 GoToChargingPoint::Response &res);
	bool StartCharging(GoToChargingPoint::Request &req,
					   GoToChargingPoint::Response &res);
	bool StopCharging(GoToChargingPoint::Request &req,
					  GoToChargingPoint::Response &res);
	bool EnableCharingService(bool);

	// app
	bool RecallHandler(AgentRequest& req, AgentResponse& res);


private:
	ros::NodeHandle *nh_ptr_;
	// TODO: Service Server
	ros::ServiceServer ui_patrol_start_server_;
	ros::ServiceServer ui_patrol_stop_server_;
	ros::ServiceServer ui_goto_coordinate_;
	ros::ServiceServer ui_goto_waypoint_;
	ros::ServiceServer ui_path_tracking_;
	ros::ServiceServer ui_goto_chargingpoint_;
	ros::ServiceServer ui_charging_start_;
	ros::ServiceServer ui_charging_stop_;
	ros::ServiceServer ui_battery_threshold_;
	ros::ServiceServer ui_recall_;

	// TODO: Service Client
	ros::ServiceClient autodocking_client_;
	ros::ServiceClient clear_costmap_service_;
	ros::ServiceClient autodocking_service_;
	ros::ServiceClient path_planning_service_;
	ros::ServiceClient systemd_service_;
	ros::ServiceClient path_tracking_client_;

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_;

	tf::TransformListener robot_tf;
	// TODO: Topic Subscriber
	ros::Subscriber movebase_result_sub_;
	ros::Subscriber movebase_result_subscriber_;
	ros::Subscriber movebase_status_subscriber_;
	ros::Subscriber movebase_speed_subscriber_;
	ros::Subscriber chassis_battery_subscriber_;
	ros::Subscriber return_electricity_subscriber_;

	// Topic Publishers;
	ros::Publisher turn_mainpc_pub_;
	ros::Publisher charging_relay1_pub_;
	ros::Publisher charging_relay2_pub_;
	ros::Publisher charging_relay_iknow_pub_;
	ros::Publisher pub_start_waypoint_;
	ros::Publisher pub_fail_waypoint_;
	ros::Publisher pub_success_waypoint_;
	ros::Publisher pub_vel_to_drive_; //转换 /nav/cmd
	ros::Publisher pub_home_to_dock_;

	// msgs
	move_base_msgs::MoveBaseGoal current_goal_;
	move_base_msgs::MoveBaseGoal current_goal_nearest_point_;

	// variables
	int battery_threshold_;
	int8_t return_electricity_;
	int32_t current_goal_index_;
	int32_t retry_time_;
	int32_t cur_speed_;
	NavigationStatus nav_status_;
	int32_t active_waypoint_;
	int32_t n_waypoints_;
	std::vector<WaypointEntry> waypointlist_ = {};

	// flags
	bool autodocking_started_;
	int32_t patrol_current_no_;
	bool is_moving_to_waypoint_;
	bool moving_to_nearest_point_;
	bool arrive_at_nearest_point_;
	int16_t state_;
	int16_t last_movebase_result_;
	int16_t last_movebase_status_;
	int16_t move_base_unknown_count_;
	bool navigation_started_;
	bool mapping_started_;
	bool self_path_plan_;
	bool first_time_move_to_nearest_point_;

public:
	static std::shared_ptr<JtcxLogWrapper> s_logger;
};
