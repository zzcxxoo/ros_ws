//
// Created by jtcx on 8/25/22.
//

/**
 * @copyright Copyright <JT-Innovation> (c) 2022
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
#include <mobile_platform_msgs/PurePursuitResult.h>
#include <mobile_platform_msgs/PurePursuitStatus.h>
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
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using mobile_platform_msgs::ChassisConstPtr;
using mobile_platform_msgs::PurePursuitResultConstPtr;
using mobile_platform_msgs::PurePursuitStatusConstPtr;

class CleaningLogger {
 private:
  /* data */
  struct logdata {
    std::string map = "";
    std::string path = "";
    double start_time = INT_MIN;
    std::string mode = "";
    double area = INT_MIN;
    double cleaning_time = 0;
    double stop_time_ = 0;
    double average_velocity = INT_MIN;
    float process = INT_MIN;
    double odometry = INT_MIN;
    std::string exception = "";
  } logger_;
  ros::NodeHandle* nh_ptr_;
  std::vector<std::string> modeList_ = {
      "VCU_EMERGENCY_STOP", "HUMAN_MANUAL", "VCU_MANUAL", "IPC_EMERGENCY",
      "IPC_MANUAL",         "IPC_AUTO",     "IPC_CLIENT"};

  double totalPointsNum_;
  bool enableAll_;
  double totalArea_;
  int task_index;
  int cleaning_times_;

  std::vector<float> twistQueue_;
  std::vector<logdata> loggers_;

 private:
  ros::Subscriber chassisSub_;
  ros::Subscriber purePursuitStartSub_;
  ros::Subscriber purePursuitStatusSub_;
  ros::Subscriber purePursuitResultSub_;
  ros::Subscriber twistSub_;

  ros::ServiceClient cleaningLoggerSrv;
  ros::Publisher cleaningStatusPub;

 private:
  void ChassisMsgHandler(const ChassisConstPtr &msg);
  void PurePursuitStartHandler(const nav_msgs::PathConstPtr &msg);
  void PurePursuitStatusHandler(const PurePursuitStatusConstPtr &msg);
  void PurePursuitResultHandler(const PurePursuitResultConstPtr &msg);
  void SaveLoggerToFile();
  void TwistHandler(const geometry_msgs::TwistConstPtr &msg);
  bool CheckLoggerParamsValid();
  void ResetLogger();

  void assignYamlValue(YAML::Node& config, int index);

 public:
  void Init(ros::NodeHandle *);
};

