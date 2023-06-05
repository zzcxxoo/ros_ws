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
#include <jsoncpp/json/json.h>
#include <mobile_platform_msgs/MappingDelete.h>
#include <mobile_platform_msgs/MappingEdit.h>
#include <mobile_platform_msgs/MappingList.h>
#include <mobile_platform_msgs/MappingRevert.h>
#include <mobile_platform_msgs/MappingSave.h>
#include <mobile_platform_msgs/MappingSelect.h>
#include <mobile_platform_msgs/MappingSelected.h>
#include <mobile_platform_msgs/MappingStart.h>
#include <mobile_platform_msgs/OssUpload.h>
#include <mobile_platform_msgs/SystemdService.h>
#include <mobile_platform_msgs/SavingAllMapAction.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/LoadMap.h>
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
#include "common/common.h"
#include "job_manager.h"

using mobile_platform_msgs::MappingDelete;
using mobile_platform_msgs::MappingEdit;
using mobile_platform_msgs::MappingList;
using mobile_platform_msgs::MappingRevert;
using mobile_platform_msgs::MappingSave;
using mobile_platform_msgs::MappingSelect;
using mobile_platform_msgs::MappingSelected;
using mobile_platform_msgs::OssUpload;
using mobile_platform_msgs::SystemdService;

extern JtcxLogWrapper* jlw_for_map_manager;
#define MLG jlw_for_map_manager->logger

class MapManager {
 public:

  void Init(ros::NodeHandle *);
  void initMapIdToMapName();
  bool GetValidMaps(MappingList::Request &req, MappingList::Response &res);
  bool HandleMappingSave(MappingSave::Request &req, MappingSave::Response &res);
  bool HelpDelete(std::string &filename);
  bool HandleMappingDelete(MappingDelete::Request &req,
						   MappingDelete::Response &res);
  bool HandleMappingSelect(MappingSelect::Request &req,
						   MappingSelect::Response &res);
  bool HandleMappingSelected(MappingSelected::Request &req,
							 MappingSelected::Response &res);
  bool HandleMappingRevert(MappingRevert::Request &req,
						   MappingRevert::Response &res);
  bool HandleMappingEdit(MappingEdit::Request &req, MappingEdit::Response &res);
  bool HandleMappingUpload(OssUpload::Request &req, OssUpload::Response &res);
  bool HandleBoundaryRecordingSave(MappingSave::Request &req, MappingSave::Response &res);
  bool HandleMappingDivisionCreateMain(MappingSave::Request &req, MappingSave::Response &res);
  bool HandleMappingDivisionCreateSub(MappingSave::Request &req, MappingSave::Response &res);
  bool HandleMappingDivisionRename(MappingSave::Request &req, MappingSave::Response &res);
  bool HandleMappingDivisionDelete(MappingSave::Request &req, MappingSave::Response &res);
  bool LoadRosParamFromFile(std::string &param_file, std::string &file_name);
  int SaveMapFromTopic(std::string &filename);
  bool CheckIfMapIsSelected();
  void setJobmanagerPtr(JobManager *ptr);

  void checkMappingStatus(const ros::TimerEvent &);

  void MappingSaveDoneHelper(std::string map_name, const actionlib::SimpleClientGoalState &state,
							 const mobile_platform_msgs::SavingAllMapResultConstPtr &result);

 private:
  ros::NodeHandle *nh_ptr_;
  ros::ServiceServer ui_mapping_save_;
  ros::ServiceServer ui_mapping_delete_;
  ros::ServiceServer ui_mapping_select_;
  ros::ServiceServer ui_mapping_selected_;
  ros::ServiceServer ui_mapping_revert_;
  ros::ServiceServer ui_mapping_edit_;
  ros::ServiceServer ui_mapping_list_;
  ros::ServiceServer ui_mapping_upload_;
  ros::ServiceServer ui_boundary_recording_save_;
  ros::ServiceServer ui_mapping_division_create_main_;
  ros::ServiceServer ui_mapping_division_create_sub_;

  ros::ServiceServer ui_mapping_division_save_custom_name_;
  ros::ServiceServer ui_mapping_division_delete_;

  ros::ServiceClient systemd_service_;
  ros::ServiceClient save_layer_client_;
  ros::ServiceClient divide_map_main_;
  ros::ServiceClient divide_map_sub_;

  std::unique_ptr<actionlib::SimpleActionClient<mobile_platform_msgs::SavingAllMapAction>> save_map_action_client_ptr_;

  ros::Publisher ui_is_mapsave_;

  bool navigation_started_;
  bool mapping_started_;

  std::vector<std::string> last_legit_files;

  ros::Timer realtime_map_uplload_;
  ros::Timer check_mapping_timer_;

  JobManager *jobPtr;
};