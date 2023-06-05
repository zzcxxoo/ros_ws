/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <mobile_platform_msgs/Process.h>
#include <mobile_platform_msgs/ProcessStatus.h>
#include <mobile_platform_msgs/SystemdService.h>
#include <mobile_platform_msgs/VirtualLineLoad.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <mobile_platform_msgs/Agent.h>
#include <nav_msgs/Odometry.h>
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
#include <thread>

#include <common/JtcxLogWrapper.hpp>

using mobile_platform_msgs::Process;
using mobile_platform_msgs::ProcessStatus;
using mobile_platform_msgs::SystemdService;
using mobile_platform_msgs::VirtualLineLoad;

using namespace mobile_platform_msgs;

extern JtcxLogWrapper* jlw_for_job_manager;
#define JLG jlw_for_job_manager->logger

class ServiceJob {
 public:
  ServiceJob(std::string service_name, bool status = false) {
	service_name_ = service_name;
	status = status;
  }

  std::string GetServiceName() { return service_name_; }

 private:
  std::string service_name_;
  bool status_;
};

class JobManager {
 public:
  void Init(ros::NodeHandle *);
  bool HandleSystemdService(SystemdService::Request &req,
							SystemdService::Response &res);
  bool HandleProcessStatusService(ProcessStatus::Request &req,
								  ProcessStatus::Response &res);
  bool HandleShutdownIPCService(SystemdService::Request &req,
								SystemdService::Response &res);
  void CheckStatus(const ros::TimerEvent &);
  void locateThreadHandler();
  bool locateHandler(AgentRequest&, AgentResponse&);

  void checkSystemInDir();
  void CheckCpuTemprature();

  ~JobManager();

 private:
  ros::NodeHandle *nh_ptr_;
  ros::NodeHandle locate_node_;
  ros::CallbackQueue locate_queue_;
  std::unique_ptr<std::thread> locate_thd_ptr_;

  ros::Timer timer_;
  ros::ServiceServer ui_systemdservice_;

  ros::ServiceServer ui_process_status_;
  
  ros::ServiceServer ui_locate_server_;

  ros::ServiceServer ui_shutdown_IPC_;

  ros::ServiceClient service_load_virtual_line_;

  ros::Publisher pub_cpu_temp_;

  std::vector<bool> service_valid_list_;
  std::vector<ServiceJob> service_list_ = {
	  ServiceJob("move_base"), ServiceJob("mapping"),
	  ServiceJob("mapping_lio_sam"), ServiceJob("navigation"),
	  ServiceJob("localization"), ServiceJob("control_master"),
	  ServiceJob("behavior"), ServiceJob("autodocking"),
	  ServiceJob("pathrecord"), ServiceJob("boundary_recording")};

  bool navigation_started_;
  bool mapping_started_;
};
