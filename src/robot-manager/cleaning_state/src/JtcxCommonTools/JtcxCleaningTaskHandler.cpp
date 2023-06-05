/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxCleaningTaskHandler.hpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/12/04
Last Modified :
Description   : Cleaning Task Handler
Function List : 
History       : first edition ----20221204
******************************************************************************/

#include "JtcxFileHandler.hpp"
#include "nlohmann/json.hpp"
#include "JtcxCleaningTaskHandler.hpp"
#include <geometry_msgs/Pose.h>

using namespace JTCX;
using json = nlohmann::json;
using namespace std;

namespace JTCX
{
	
		CleaningTaskHandler::CleaningTaskHandler()
		{
			_time_tool = new util();
			_logger = JtcxSingleton<JtcxLogger>::GetInstance();
		}
		CleaningTaskHandler::~CleaningTaskHandler()
		{
			delete _time_tool;
		}

		void CleaningTaskHandler::parseCleaningConfigFromAPP(const std::string &msg,CleaningAppConfInfo &conf_info)
		{
			json j = json::parse(msg);
			std::string mode = "new";
			if(j.contains("mode"))	mode = j["mode"];

			if(mode == "new"){
			
				conf_info.continuous_mode = false;

				_time_tool->setStartTime();

				if (j.contains("plan_id"))  conf_info.plan_id=j["plan_id"];
				else _logger->ERROR("There is no plan name from APP config");

				if (j.contains("map_id")) conf_info.map_id=j["map_id"];
				else _logger->ERROR("There is no map name from APP config");

				//创建该地图下的路径管理器单例；
				auto PathHandler = JtcxSingleton<CleaningPathHandler>::GetInstance();
				PathHandler->init(conf_info.map_id,conf_info.plan_id);

				if (j.contains("zones"))
				{
					std::vector<CleaningTaskInfo> temp;
					std::string s_zones = j["zones"].dump();
					std::vector<json> js_zones = json::parse(s_zones).get<std::vector<json>>();
					std::string zone_location=std::string(ROBOT_MAP_DIR) + "/" + conf_info.map_id +"/zone/";

					for(const auto& zone : js_zones)
					{
						int times = zone["times"].get<int>();
						CleaningTaskInfo single_task;
						std::string csv_file_location= zone_location + zone["zone_id"].get<std::string>()+ "/" + zone["mode"].get<std::string>() + ".csv";
						_logger->TRACE("try to parse csv: {}", csv_file_location);
						PathHandler->getPathInfoFromCSVFile(csv_file_location,single_task);
						single_task.cleaning_mode = zone["mode"];
						single_task.rub_type = zone["rubtype"];
						for(int i=0;i<times;i++){
							temp.insert(temp.end(),single_task);
						}
					}
					json j_t = temp;
					conf_info.tasks = j_t.dump();
					_logger->TRACE("parse task from req: {}", conf_info.tasks);
				}
				else _logger->ERROR("There is no tasks from APP config");
			}
			else{
				conf_info.continuous_mode = true;
			}
		}
		// 定时任务！！暂不支持多区域
		void CleaningTaskHandler::parseCleaningConfigFromAPPTiming(const json& msg, CleaningAppConfInfo &conf_info)
		{
			conf_info.task_id = msg["task_id"];
			conf_info.repeat_dates = msg["repeat_dates"].get<std::vector<int>>();
			conf_info.enable = true;
			conf_info.continuous_mode = false;
			conf_info.start_time = msg["start_time"];
			json j = msg["plan"];

			conf_info.map_id = j["map_id"];
			conf_info.plan_id = j["plan_id"];

			//创建该地图下的路径管理器单例；
			auto PathHandler = JtcxSingleton<CleaningPathHandler>::GetInstance();
			CleaningTaskInfo single_task;
			std::string zone_location = std::string(ROBOT_MAP_DIR) + "/" + conf_info.map_id + "/zone/";
			std::string csv_file_location = zone_location + j["zone_id"].get<std::string>() + "/" + j["mode"].get<std::string>() + ".csv";
			_logger->TRACE("try to parse csv: {}", csv_file_location);
			PathHandler->getPathInfoFromCSVFile(csv_file_location, single_task);
			single_task.cleaning_mode = j["mode"];
			single_task.rub_type = j["rubtype"];

			std::vector<CleaningTaskInfo> temp;
			for(int i=0; i<j["times"].get<int>(); i++)	temp.emplace_back(single_task);
			conf_info.tasks = json(temp).dump();
		}

		void CleaningTaskHandler::getContextFromFile(const std::string& context_file_path,CleaningContextInfo& context)
		{
			std::lock_guard<std::recursive_mutex> lock(_context_file_mutex); 

			if(JtcxDir::existDir(context_file_path))
			{
				std::ifstream inf(context_file_path);
				json j = json::parse(inf);
				inf.close();

				std::vector<CleaningContextInfo> cc_vec = j.get<std::vector<CleaningContextInfo>>();
				//std::vector<json> cc_vec = j.get<std::vector<json>>();

				if(!cc_vec.empty()) 
				{
					context = cc_vec.back();
				}
				else
				{
					_logger->ERROR("context file is empty or invalid!!");
				}
			}
			else 
			{ _logger->ERROR("context file is not created!");}
		}
		
		void CleaningTaskHandler::getTasksFromFile(const std::string& tasks_file_path,std::vector<CleaningTaskInfo>& tasks)
		{
			if(JtcxDir::existDir(tasks_file_path))
			{
				std::ifstream inf(tasks_file_path);
				json j = json::parse(inf);
				inf.close();
				tasks = j.get<std::vector<CleaningTaskInfo>>();

				if(tasks.empty()) 
				{
					_logger->ERROR("tasks file is empty or invalid!!");
				}
			}
			else 
			{ _logger->ERROR("tasks file is not created!");}
		}
		
		void CleaningTaskHandler::getConfsFromFile(const std::string& file_path,std::vector<CleaningAppConfInfo>& confs)
		{
			if(JtcxDir::existDir(file_path))
			{
				try{
					std::ifstream inf(file_path);
					json j = json::parse(inf);
					inf.close();
					confs = j.get<std::vector<CleaningAppConfInfo>>();
				}catch(const json::exception& e){
					_logger->ERROR("confs file is empty or invalid: {}!", e.what());
				}
			}
			else 
			{
				_logger->ERROR("confs file is not created!");
				throw std::runtime_error("timing task file is not created, usually it's not possible!!");
			}
		}

		void CleaningTaskHandler::saveContext2File(CleaningContextInfo& context,const std::string& exception)
		{
			std::lock_guard<std::recursive_mutex> lock(_context_file_mutex); 
			/*不覆盖原始的断点context。现，记录断点原因并为后续做分析*/
			/*不再限制100条数量*/
			if(!JtcxDir::existDir(CLEANING_CONTEXT_JSON_PATH)) system("touch " CLEANING_CONTEXT_JSON_PATH);
			std::vector<CleaningContextInfo> cc_vec;
			try{
				std::ifstream inf(CLEANING_CONTEXT_JSON_PATH);
				json j = json::parse(inf);
				inf.close();
				cc_vec = j.get<std::vector<CleaningContextInfo>>();
			}catch(const json::exception& e){
				_logger->WARN("context file cant get anything, maybe empty: {}", e.what());
			}
			context.exception = exception;
			cc_vec.emplace_back(context);

			std::ofstream out(CLEANING_CONTEXT_JSON_PATH, std::ios::out);
			if(!out){_logger->ERROR("can not write into {} ",CLEANING_CONTEXT_JSON_PATH);}
			json j = cc_vec;
			out << j;
			out.close();
			_logger->TRACE("save context success!");
		}

		void CleaningTaskHandler::saveTasks2File(const std::string& tasks_file_path,const std::vector<CleaningTaskInfo>& tasks)
		{
			std::ofstream out;
			out.open(tasks_file_path,std::ios::out);
			if(!out){
				_logger->ERROR("can not write into {} ",tasks_file_path);
			}
			json j = tasks;
			out << j;
			out.close();
			_logger->TRACE("save tasks success!");
		}

		void CleaningTaskHandler::saveConfs2File(const std::string& file_path,const std::vector<CleaningAppConfInfo>& confs)
		{
			std::ofstream out;
			out.open(file_path,std::ios::out);//覆盖
			if(!out){
				_logger->ERROR("can not write into {} ",file_path);
			}
			json j = confs;
			out << j;
			out.close();
			_logger->TRACE("save confs success!");
		}
		
		void CleaningTaskHandler::updateContext2File(CleaningContextInfo& context, std::string exception)
		{
			//每一次task完成后覆盖记录
			std::lock_guard<std::recursive_mutex> lock(_context_file_mutex); 
			if(!JtcxDir::existDir(CLEANING_CONTEXT_JSON_PATH))  system("touch " CLEANING_CONTEXT_JSON_PATH);
			std::vector<CleaningContextInfo> cc_vec;
			try{
				std::ifstream inf(CLEANING_CONTEXT_JSON_PATH);
				json j = json::parse(inf);
				inf.close();
				cc_vec = j.get<std::vector<CleaningContextInfo>>();
			}catch(const json::exception& e){
				_logger->WARN("context file cant get anything, maybe empty: {}", e.what());
			}

			context.exception = exception;
			
			if(cc_vec.empty()) 
				cc_vec.emplace_back(context);
			else
				cc_vec.back() = context;

			std::ofstream out(CLEANING_CONTEXT_JSON_PATH, std::ios::out);
			json j2 = cc_vec;
			out << j2;
			out.close();
			_logger->TRACE("update context success!");
		}
		
		void CleaningTaskHandler::initCurrentContext(const CleaningAppConfInfo& conf_info, CleaningContextInfo& context)
		{
			if(!conf_info.continuous_mode)
			{
				context.map_id = conf_info.map_id;
				context.plan_id = conf_info.plan_id;
				//context.tasks ; 不做初始化；
				context.start_time = _time_tool->getStartTime();

				context.process  = 0;
				context.area = 0;
				context.odometry = 0;
				context.average_velocity =0;
				context.completed_task_id = 0;

				context.cleaning_time = -1;
				context.stop_time = -1;
				context.exception = JtcxSingleton<CleaningException>::GetInstance()->NORMAL;
			}
			else
			{
				getContextFromFile(std::string(CLEANING_CONTEXT_JSON_PATH),context);
			}
		}
		
		void CleaningTaskHandler::initCurrentContext(const std::string& map_id,const std::string& plan_id,CleaningContextInfo& context){

				context.map_id = map_id;
				context.plan_id = plan_id;
				context.start_time = _time_tool->getStartTime();
				context.process  = 0;
				context.area = 0;
				context.odometry = 0;
				context.average_velocity =0;
				context.completed_task_id = 0;
				context.cleaning_time = -1;
				context.stop_time = -1;
				context.exception = JtcxSingleton<CleaningException>::GetInstance()->NORMAL;
		}

		void CleaningTaskHandler::genTasksFromAPPconfig(const CleaningAppConfInfo& conf_info, std::vector<CleaningTaskInfo>& tasks)
		{
			tasks = json::parse(conf_info.tasks).get<std::vector<CleaningTaskInfo>>();
		}

		bool CleaningTaskHandler::genTaskFromCharging(const string& from_charging_csv_location,CleaningTaskInfo& task)
		{
			CleaningPathHandler* path_tool_ptr= JtcxSingleton<CleaningPathHandler>::GetInstance();
			//最终在执行器排序
			task.ID = -1;
			path_tool_ptr->getPathInfoFromCSVFile(from_charging_csv_location,task);
			task.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->FROM_CHARGING;
			task.rub_type = "small";
			return true;
		}

		bool CleaningTaskHandler::genTaskToCharging(const string& to_charging_csv_location,CleaningTaskInfo& task)
		{
			CleaningPathHandler* path_tool_ptr= JtcxSingleton<CleaningPathHandler>::GetInstance();
			//最终在执行器排序
			task.ID = -1;
			path_tool_ptr->getPathInfoFromCSVFile(to_charging_csv_location,task);
			task.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->TO_CHARGING;
			task.rub_type = "small";
			return true;
		}

		bool CleaningTaskHandler::genTaskFromJoinPath(const string& join_csv_location,CleaningTaskInfo& task)
		{
			CleaningPathHandler* path_tool_ptr= JtcxSingleton<CleaningPathHandler>::GetInstance();
			//最终在执行器排序
			task.ID = -1;
			path_tool_ptr->getPathInfoFromCSVFile(join_csv_location,task);
			task.cleaning_mode = JtcxSingleton<CleaningMode>::GetInstance()->JOIN;
			task.rub_type = "small";
			return true;
		}

		bool CleaningTaskHandler::genClipTask(const CleaningTaskInfo& task,const int& way_point, CleaningTaskInfo& task_clip)
		{
			CleaningPathHandler* path_tool_ptr= JtcxSingleton<CleaningPathHandler>::GetInstance();
			std::string clip_path_location =JtcxDir::getFatherDir(task.path_file_location)+"/"+JtcxDir::getFileStem(task.path_file_location)+"_clip.csv" ; 

			task_clip.ID                   = 0 ;
			task_clip.cleaning_mode        = task.cleaning_mode ;
			task_clip.rub_type             = task.rub_type ;
			task_clip.path_file_location   = clip_path_location;

			if(path_tool_ptr->createClipPathAndSave(task,way_point,clip_path_location)){ 
				path_tool_ptr->getPathInfoFromCSVFile(clip_path_location,task_clip);
				return true;
			}
			else{
				_logger->INFO("can not create {}",task_clip.path_file_location);
				return false;
			}
		}

}