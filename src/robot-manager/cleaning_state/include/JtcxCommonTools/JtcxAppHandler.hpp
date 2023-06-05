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
#include "JtcxCleaningDefinitions.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxUtilTools.hpp"

using namespace JTCX;
namespace JTCX
{
	class CleaningAppHandler
	{
		public:
		CleaningAppHandler(){};
		~CleaningAppHandler(){}

		static std::string genReportId() noexcept;
        static bool callAgent(std::string topic, std::string srv_id, const json& msg, std::string& res);
        static void uploadStatus(const CleaningContextInfo&);
		static void uploadReport(const CleaningContextInfo&);
		
		private:
		static JtcxLogger* _logger;
	};
}   