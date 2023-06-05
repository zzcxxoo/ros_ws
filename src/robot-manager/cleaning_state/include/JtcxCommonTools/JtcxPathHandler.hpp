/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxFileHandler.hpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/11/26
Last Modified :
Description   : File and folder processing
Function List : 
History       : first edition ----20221125
******************************************************************************/
#pragma once

#include "JtcxMacroDefinition.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxFileHandler.hpp"
#include "JtcxCleaningDefinitions.hpp"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>


using namespace JTCX;

namespace JTCX
{
	/// @brief This handler instance is constructed in the parsing APP config function
	/// @location CleaningTaskHandler::parseCleaningConfigFromAPP(...）

	class CleaningPathHandler
	{
		public:
			std::string _map_name="NULL";   
        	std::string _plan_name="NULL";
			// geometry_msgs::Pose  _start_point;
			// geometry_msgs::Pose  _end_point;
			// int _path_size;
		public:
		CleaningPathHandler()
		{
			_logger = JtcxSingleton<JtcxLogger>::GetInstance();
		}

		~CleaningPathHandler(){}

		inline void init(std::string&map_name,std::string& plan_name)
		{
			_map_name = map_name;
			_plan_name = plan_name;
		}

		/// @brief add heading angle to starting point and return a new pose
		/// @param p1 start point
		/// @param p2 end point
		/// @return 
		geometry_msgs::Pose getNewPose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);
		
		/// @brief get pose from line 
		/// @param l "x y tx ty tz tw"
		/// @return geometry_msgs::Pose
		geometry_msgs::Pose getPoseFromline(const std::string& l);
		std::string getLineFromPose(const geometry_msgs::Pose& p);

		/// @brief get path info from file
		/// @param csv_path 
		/// @param path_info 
		void getPathInfoFromCSVFile(const std::string csv_file_location,CleaningTaskInfo &path_info);

		/// @brief call("/target_point_planner", srv) 
		/// @brief generate nav msg path and save to csv file
		/// @param from start
		/// @param to end
		bool createJoinPathAndSave(const CleaningTaskInfo& from, const CleaningTaskInfo& to, std::string &csv_file_location);
		
		/// @brief call("/auto_dock/home_to_dock",srv)
		/// @brief generate to_charging by home to dock service
		bool createToChargingPathAndSave(const CleaningTaskInfo& last_cleaning_path, const std::string &csv_file_location);

		/// @brief clip from way point and generate new path 
		bool createClipPathAndSave(const CleaningTaskInfo& task, const int& way_point, const std::string &csv_file_location);

		/// @brief save nav msg path to csv file
		/// @param path 
		/// @param csv_file 
		void saveNavPath(const nav_msgs::Path& path, const std::string& csv_file);

		/// @brief get current pose from ros param /robot_pose
		/// @param sp pose string 
		bool getCurrentPose(std::string& sp);

		private:
			JtcxLogger* _logger;
	};

}   