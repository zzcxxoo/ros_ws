//
// Created by jtcx on 10/19/22.
//

#pragma once

#include "JtcxCommon.hpp"
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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <mobile_platform_msgs/ChargingLoad.h>
#include <mobile_platform_msgs/ChargingSave.h>
#include <mobile_platform_msgs/InitPoseLoad.h>
#include <mobile_platform_msgs/InitPoseSave.h>
#include <mobile_platform_msgs/PathPlanningDelete.h>
#include <mobile_platform_msgs/PathPlanningList.h>
#include <mobile_platform_msgs/PathPlanningLoad.h>
#include <mobile_platform_msgs/PathPlanningSave.h>
#include <mobile_platform_msgs/VirtualLineLoad.h>
#include <mobile_platform_msgs/VirtualLineSave.h>
#include <mobile_platform_msgs/MappingDelete.h>
#include <mobile_platform_msgs/MappingEdit.h>
#include <mobile_platform_msgs/MappingList.h>
#include <mobile_platform_msgs/MappingSelect.h>
#include <mobile_platform_msgs/SystemdService.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <move_base_msgs/MoveBaseGoal.h>
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

using mobile_platform_msgs::ChargingLoad;
using mobile_platform_msgs::ChargingSave;

using mobile_platform_msgs::InitPoseLoad;
using mobile_platform_msgs::InitPoseSave;

using mobile_platform_msgs::PathPlanningDelete;
using mobile_platform_msgs::PathPlanningList;
using mobile_platform_msgs::PathPlanningLoad;
using mobile_platform_msgs::PathPlanningSave;

using mobile_platform_msgs::VirtualLineLoad;
using mobile_platform_msgs::VirtualLineSave;
using mobile_platform_msgs::MappingDelete;
using mobile_platform_msgs::MappingEdit;
using mobile_platform_msgs::MappingList;
using mobile_platform_msgs::MappingSelect;

using mobile_platform_msgs::SystemdService;

class ExternalHighFrequencyEventReceiver {
	public:
		static std::unique_ptr<ExternalHighFrequencyEventReceiver> &getinstance() {
			return _pExternalHighFrequencyEventReceiver;
		};

		std::promise<bool> &GetIsEventComplete() {
			return _is_event_complete;
		}

	public:
		const InitPoseSave::Request &GetInitialPoseSaveReq() const {
			return _initial_pose_save_req;
		}
		const InitPoseLoad::Request &GetInitialPoseLoadReq() const {
			return _initial_pose_load_req;
		}
		const ChargingSave::Response &GetChargingSaveReq() const {
			return _charging_save_req;
		}
		const ChargingLoad::Request &GetChargingLoadReq() const {
			return _charging_load_req;
		}
		const VirtualLineSave::Request &GetVirtuallineSaveReq() const {
			return _virtualline_save_req;
		}
		const VirtualLineLoad::Request &GetVirtuallineLoadReq() const {
			return _virtualline_load_req;
		}
		const PathPlanningSave::Request &GetPathPlanningSaveReq() const {
			return _path_planning_save_req;
		}
		const PathPlanningLoad::Request &GetPathPlanningLoadReq() const {
			return _path_planning_load_req;
		}
		const PathPlanningDelete::Request &GetPathPlanningDeleteReq() const {
			return _path_planning_delete_req;
		}
		const PathPlanningList::Request &GetPathPlanningListReq() const {
			return _path_planning_list_req;
		}
		const MappingDelete::Request &GetMappingDeleteReq() const {
			return _mapping_delete_req;
		}
		const MappingSelect::Request &GetMappingSelectReq() const {
			return _mapping_select_req;
		}
		const MappingEdit::Request &GetMappingEditReq() const {
			return _mapping_edit_req;
		}
		const MappingList::Request &GetMappingListReq() const {
			return _mapping_list_req;
		}
	public:
		void SetInitialPoseSaveRes(const InitPoseSave::Response &initial_pose_save_res) {
			_initial_pose_save_res = initial_pose_save_res;
		}
		void SetInitialPoseLoadRes(const InitPoseLoad::Response &initial_pose_load_res) {
			_initial_pose_load_res = initial_pose_load_res;
		}
		void SetChargingSaveRes(const ChargingSave::Request &charging_save_res) {
			_charging_save_res = charging_save_res;
		}
		void SetChargingLoadRes(const ChargingLoad::Response &charging_load_res) {
			_charging_load_res = charging_load_res;
		}
		void SetVirtuallineSaveRes(const VirtualLineSave::Response &virtualline_save_res) {
			_virtualline_save_res = virtualline_save_res;
		}
		void SetVirtualineLoadRes(const VirtualLineLoad::Response &virtualine_load_res) {
			_virtualine_load_res = virtualine_load_res;
		}
		void SetPathPlanningSaveRes(const PathPlanningSave::Response &path_planning_save_res) {
			_path_planning_save_res = path_planning_save_res;
		}
		void SetPathPlanningLoadRes(const PathPlanningLoad::Response &path_planning_load_res) {
			_path_planning_load_res = path_planning_load_res;
		}
		void SetPathPlanningDeleteRes(const PathPlanningDelete::Response &path_planning_delete_res) {
			_path_planning_delete_res = path_planning_delete_res;
		}
		void SetPathPlanningListRes(const PathPlanningList::Response &path_planning_list_res) {
			_path_planning_list_res = path_planning_list_res;
		}
		void SetMappingDeleteRes(const MappingDelete::Response &mapping_delete_res) {
			_mapping_delete_res = mapping_delete_res;
		}
		void SetMappingSelectRes(const MappingSelect::Response &mapping_select_res) {
			_mapping_select_res = mapping_select_res;
		}
		void SetMappingEditRes(const MappingEdit::Response &mapping_edit_res) {
			_mapping_edit_res = mapping_edit_res;
		}
		void SetMappingListRes(const MappingList::Response &mapping_list_res) {
			_mapping_list_res = mapping_list_res;
		}

	private:
		ExternalHighFrequencyEventReceiver() { std::make_unique<std::thread>(ExternalHighFrequencyEventReceiver::RosHandler); };
		ExternalHighFrequencyEventReceiver(const ExternalHighFrequencyEventReceiver &) = delete;
		ExternalHighFrequencyEventReceiver &operator=(const ExternalHighFrequencyEventReceiver &) = delete;

		static std::unique_ptr<ExternalHighFrequencyEventReceiver> _pExternalHighFrequencyEventReceiver;
		std::unique_ptr<std::thread> _ros_thread;   /// < external event listener thread pointer

		static void RosHandler() { ros::spin(); }

	private:
		/*** ros servers ***/
		ros::NodeHandle _nh;
//		ros::ServiceServer _ui_systemdservice_wakeup;
//		ros::ServiceServer _ui_rubber_pusher_open;     ///TODO
		ros::ServiceServer _ui_initpose_load;
		ros::ServiceServer _ui_initpose_save;
		ros::ServiceServer _ui_charging_save;
		ros::ServiceServer _ui_charging_load;
		ros::ServiceServer _ui_virtualline_save;
		ros::ServiceServer _ui_virtualline_load;
		ros::ServiceServer _ui_path_planning_list;
		ros::ServiceServer _ui_path_planning_save;
		ros::ServiceServer _ui_path_planning_load;
		ros::ServiceServer _ui_path_planning_delete;
//		ros::ServiceServer _ui_path_planning_edit;  ///TODO
		ros::ServiceServer _ui_mapping_delete;
		ros::ServiceServer _ui_mapping_select;
		ros::ServiceServer _ui_mapping_edit;
		ros::ServiceServer _ui_mapping_list;

		/*** ros callback functions ***/
		bool HandleInitialPoseSave(InitPoseSave::Request &req,
								   InitPoseSave::Response &res);
		bool HandleInitialPoseLoad(InitPoseLoad::Request &req,
								   InitPoseLoad::Response &res);
		bool HandleChargingSave(ChargingSave::Request &req,
								ChargingSave::Response &res);
		bool HandleChargingLoad(ChargingLoad::Request &req,
								ChargingLoad::Response &res);
		bool HandleVirtualLineSave(VirtualLineSave::Request &req,
								   VirtualLineSave::Response &res);
		bool HandleVirtualLineLoad(VirtualLineLoad::Request &req,
								   VirtualLineLoad::Response &res);
		bool HandlePathLoad(PathPlanningLoad::Request &req,
							PathPlanningLoad::Response &res);
		bool HandlePathSave(PathPlanningSave::Request &req,
							PathPlanningSave::Response &res);
		bool HandlePathDelete(PathPlanningDelete::Request &req,
							  PathPlanningDelete::Response &res);
		bool HandleGetPathList(PathPlanningList::Request &req,
							   PathPlanningList::Response &res);
		bool HandleMappingDelete(MappingDelete::Request &req,
								 MappingDelete::Response &res);
		bool HandleMappingSelect(MappingSelect::Request &req,
								 MappingSelect::Response &res);
		bool HandleMappingEdit(MappingEdit::Request &req, MappingEdit::Response &res);
		bool HandleGetValidMaps(MappingList::Request &req, MappingList::Response &res);

		/*** Stores User informations for calling by exectors  ***/
		InitPoseSave::Request _initial_pose_save_req;
		InitPoseSave::Response _initial_pose_save_res;
		InitPoseLoad::Request _initial_pose_load_req;
		InitPoseLoad::Response _initial_pose_load_res;
		ChargingSave::Response _charging_save_req;
		ChargingSave::Request _charging_save_res;
		ChargingLoad::Request _charging_load_req;
		ChargingLoad::Response _charging_load_res;
		VirtualLineSave::Request _virtualline_save_req;
		VirtualLineSave::Response _virtualline_save_res;
		VirtualLineLoad::Request _virtualline_load_req;
		VirtualLineLoad::Response _virtualine_load_res;
		PathPlanningSave::Request _path_planning_save_req;
		PathPlanningSave::Response _path_planning_save_res;
		PathPlanningLoad::Request _path_planning_load_req;
		PathPlanningLoad::Response _path_planning_load_res;
		PathPlanningDelete::Request _path_planning_delete_req;
		PathPlanningDelete::Response _path_planning_delete_res;
		PathPlanningList::Request _path_planning_list_req;
		PathPlanningList::Response _path_planning_list_res;
		MappingDelete::Request _mapping_delete_req;
		MappingDelete::Response _mapping_delete_res;
		MappingSelect::Request _mapping_select_req;
		MappingSelect::Response _mapping_select_res;
		MappingEdit::Request _mapping_edit_req;
		MappingEdit::Response _mapping_edit_res;
		MappingList::Request _mapping_list_req;
		MappingList::Response _mapping_list_res;
	private:
		std::promise<bool> _is_event_complete;

};
