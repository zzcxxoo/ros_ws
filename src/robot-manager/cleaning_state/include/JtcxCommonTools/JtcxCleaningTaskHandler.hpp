/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxCleaningTaskHandler.hpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/12/02
Last Modified :
Description   : Cleaning Task Handler
Function List : 
History       : first edition ----20221202
******************************************************************************/
#pragma once

#include "JtcxMacroDefinition.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxPathHandler.hpp"
#include "JtcxUtilTools.hpp"
#include "JtcxCleaningDefinitions.hpp"


using namespace JTCX;

namespace JTCX
{

	class CleaningTaskHandler
	{
		public:
		util* _time_tool;
	
		public:
		CleaningTaskHandler();
		~CleaningTaskHandler();
		/// @brief Parse information from APP configuration
		/// @param msg 
		/// @param conf_info Parsed message structure
		void parseCleaningConfigFromAPP(const std::string &msg,CleaningAppConfInfo &conf_info);
		void parseCleaningConfigFromAPPTiming(const json& msg, CleaningAppConfInfo &conf_info);

		/// @brief Get the latest context from the context file
		/// @param context_file_path 
		/// @param context  the latest context
		void getContextFromFile(const std::string &context_file_path,CleaningContextInfo& context);

		/// @brief Get all tasks from the task file
		/// @param tasks_file_path 
		/// @param tasks the last is the last completed_task_id
		void getTasksFromFile(const std::string& tasks_file_path,std::vector<CleaningTaskInfo>& tasks);

		/// @brief Get the timing cleaning configuration information from the file
		/// @param tasks_file_path 
		/// @param tasks 
		void getConfsFromFile(const std::string& file_path,std::vector<CleaningAppConfInfo>& confs);

		/// @brief Save cleaning context to file (non overwriting)
		/// @param context 
		/// @param exception 
		void saveContext2File(CleaningContextInfo& context,const std::string& exception);
		
		/// @brief Save cleaning tasks info to file (overwriting)
		/// @param tasks_file_path 
		/// @param tasks 
		void saveTasks2File(const std::string& tasks_file_path,const std::vector<CleaningTaskInfo>& tasks);

		/// @brief Save the timing cleaning configuration information to a file
		/// @param tasks_file_path 
		/// @param tasks 
		void saveConfs2File(const std::string& file_path,const std::vector<CleaningAppConfInfo>& confs);
		
		/// @brief Update cleaning context to file (overwriting)
		/// @param context_file_path 
		/// @param context 
		// void updateContext2File(const std::string &context_file_path,const CleaningContextInfo& context);
		void updateContext2File(CleaningContextInfo& context, std::string exception);

		/// @brief Initialize the cleaning context from the app configuration information
		/// @param conf_info 
		/// @param context 
		void initCurrentContext(const CleaningAppConfInfo& conf_info,CleaningContextInfo& context);
		
		/// @brief initializes a cleaning context from original
		/// @param context 
		void initCurrentContext(const std::string& map_id,const std::string& plan_id,CleaningContextInfo& context);

		/// @brief generate tasks from the app configuration information 
		/// @param conf_info 
		/// @param tasks 
		void genTasksFromAPPconfig(const CleaningAppConfInfo& conf_info, std::vector<CleaningTaskInfo>& tasks);

		/// @brief generate tasks from from_charging.csv file
		/// @param from_charging_csv_location 
		/// @param task 
		/// @return 
		bool genTaskFromCharging(const std::string& from_charging_csv_location,CleaningTaskInfo& task);
		
		/// @brief generate tasks from to_charging.csv file 
		/// @param to_charging_csv_location 
		/// @param task 
		/// @return 
		bool genTaskToCharging(const std::string& to_charging_csv_location,CleaningTaskInfo& task);
		
		/// @brief generate tasks from join path .csv file
		/// @param join_csv_location 
		/// @param task 
		/// @return 
		bool genTaskFromJoinPath(const std::string& join_csv_location,CleaningTaskInfo& task);
		
		/// @brief generate task clip 
		bool genClipTask(const CleaningTaskInfo& task,const int& way_point, CleaningTaskInfo& task_clip);

		private :
			std::recursive_mutex _context_file_mutex;
			JtcxLogger* _logger;
	};
}   