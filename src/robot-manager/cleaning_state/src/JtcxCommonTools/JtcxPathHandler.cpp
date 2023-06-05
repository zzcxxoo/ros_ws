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
#include "JtcxMacroDefinition.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxFileHandler.hpp"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

#include "mobile_platform_msgs/TargetPoint.h"
#include "mobile_platform_msgs/HomeToDock.h"
#include "JtcxPathHandler.hpp"

using namespace JTCX;

namespace JTCX
{
		
		geometry_msgs::Pose CleaningPathHandler::getNewPose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
		{
			auto x = p2.position.x - p1.position.x;
			auto y = p2.position.y - p1.position.y;

			geometry_msgs::Quaternion q;
			tf::quaternionTFToMsg(tf::createQuaternionFromYaw(std::atan2(y, x)), q);
			geometry_msgs::Pose po;
			po = p1;
			po.orientation = q;
			return po;
		}

		std::string CleaningPathHandler::getLineFromPose(const geometry_msgs::Pose& p){
			return fmt::format("{} {} {} {} {} {}", p.position.x, p.position.y, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
		}
		
		geometry_msgs::Pose CleaningPathHandler::getPoseFromline(const std::string& l){
			std::stringstream ss(l);
			geometry_msgs::Pose p;
			ss >> p.position.x   >> p.position.y
			>> p.orientation.x   >> p.orientation.y 
			>> p.orientation.z   >> p.orientation.w;
			p.position.z = 0;
			return p;
		}

		void CleaningPathHandler::getPathInfoFromCSVFile(const std::string csv_file_location,CleaningTaskInfo &path_info)
		{
			std::ifstream inf(csv_file_location);
			std::string line;
			std::vector<std::string> lines;
			while(std::getline(inf, line))
			{
				lines.emplace_back(std::move(line));
			}
			inf.close();

			if(lines.size() < 1.0 / 0.2)
			{
				_logger->ERROR("{} get too little points", csv_file_location);
			}
			else
			{
				path_info.path_size = lines.size();
				auto p1 = getPoseFromline(lines[0]);
				auto p2 = getPoseFromline(lines[1]);
				path_info.start = getLineFromPose(getNewPose(p1, p2));
				
				p1 = getPoseFromline(lines[path_info.path_size-2]);
				p2 = getPoseFromline(lines[path_info.path_size-1]);
				path_info.end = getLineFromPose(getNewPose(p1, p2));

				path_info.zone_id = JtcxDir::getFileStem(JtcxDir::getFatherDir(csv_file_location));
				path_info.zone_dir  = JtcxDir::getFatherDir(csv_file_location);
				path_info.map_id  = this->_map_name;
				path_info.path_file_location = csv_file_location;
    		}
		}

		bool CleaningPathHandler::createJoinPathAndSave(const CleaningTaskInfo& from, const CleaningTaskInfo& to,std::string &csv_file_location)
		{
			
			std::string path_name1 = JtcxDir::getFileStem(from.path_file_location);
			std::string path_name2 = JtcxDir::getFileStem(to.path_file_location);
			if(path_name1==path_name2)
			{
				csv_file_location =to.zone_dir + "/" + from.zone_id + "_loop.csv" ;
			}
			else
			{
				csv_file_location =to.zone_dir + "/"+ fmt::format("from_{}_to_{}.csv", path_name1, path_name2);
			}

			// if(JtcxDir::existDir(csv_file_location)) return true;

			float x1, y1, x2, y2;
			auto from_end = getPoseFromline(from.end);
			auto to_start = getPoseFromline(to.start);
			x1 = from_end.position.x;
			y1 = from_end.position.y;
			x2 = to_start.position.x;
			y2 = to_start.position.y;

			if(std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)) > 1.0){
				TargetPoint srv;
				srv.request.map_name = from.map_id;
				geometry_msgs::PoseStamped ps;
				ps.pose = from_end;
				srv.request.first_path.poses.push_back(ps);
				ps.pose = to_start;
				srv.request.second_path.poses.push_back(ps);
				
				if(ros::service::call("/target_point_planner", srv))
				{
					if(srv.response.status == 0){
						const auto& p = srv.response.planner_path;
						saveNavPath(p,csv_file_location);
						return true;
					}else{
						_logger->ERROR("call a start error code({}), msg({})", srv.response.status, srv.response.message);
						return false;
					}
				}else{
					_logger->ERROR("A* offline");
					return false;
				}
			}
			else return false;
		}

		bool CleaningPathHandler::createToChargingPathAndSave(const CleaningTaskInfo& last_cleaning_path,const std::string &csv_file_location)
		{
			HomeToDock htd;
			htd.request.start.pose.pose = getPoseFromline(last_cleaning_path.end);

			if(ros::service::call("/auto_dock/home_to_dock", htd)){
				if(htd.response.path.poses.size() < 1.0 / 0.2){
					_logger->ERROR("auto docking get too little path poses!!");
					return false;
				}
				// write to to_charging.csv
				saveNavPath(htd.response.path, csv_file_location);
				return true;
			}else{
				_logger->ERROR("auto docking offline!!");
				return false;
			} 
		}

		bool CleaningPathHandler::createClipPathAndSave(const CleaningTaskInfo& task_path_info, const int& way_point, const std::string &csv_file_location)
		{
			std::ifstream inf(task_path_info.path_file_location);
			std::string line;
			std::vector<std::string> lines;
			while(std::getline(inf, line)){
				lines.emplace_back(std::move(line));
			}
			inf.close();
			if(way_point>lines.size()) return false;

			std::ofstream out(csv_file_location);
			for(int i=way_point; i<lines.size(); i++){
				out << lines[i] << std::endl;
			}
			out.close();
			_logger->INFO("generate {} path size: {}", JtcxDir::getFileStem(csv_file_location), way_point);
			return true;
		}

		void CleaningPathHandler::saveNavPath(const nav_msgs::Path& path, const std::string& csv_file)
		{
			std::ofstream out(csv_file);
			_logger->INFO("save nav path : {}", JtcxDir::getFileStem(csv_file));
			for(auto const& pst : path.poses){
				auto const& p = pst.pose;
				auto line = fmt::format("{} {} {} {} {} {}", p.position.x, p.position.y,
							p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
				out << line << std::endl;
			}
			_logger->INFO("{} path size: {}", JtcxDir::getFileStem(csv_file), path.poses.size());
			out.close();
		}

		bool CleaningPathHandler::getCurrentPose(std::string& sp)
		{
			geometry_msgs::Pose p;
			std::vector<float> pose;
			if(ros::param::get("/robot_pose", pose) && pose.size() >= 7)
			{
				p.position.x = pose[0];
				p.position.y = pose[1];
				p.position.z = pose[2];
				p.orientation.x = pose[3];
				p.orientation.y = pose[4];
				p.orientation.z = pose[5];
				p.orientation.w = pose[6];
				sp = getLineFromPose(p);
				return true;
			}
			return false;
		}

}   
