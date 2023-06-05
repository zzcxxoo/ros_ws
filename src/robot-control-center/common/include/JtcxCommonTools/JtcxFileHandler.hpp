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

namespace JTCX
{
	class JtcxDir
	{
		public:
		~JtcxDir(){}
		
		static bool existDir(const std::string& dir);
		static void createDir(const std::string& dir);
		static void deleteDir(const std::string& dir);
		static void enumCSVFiles(std::vector<std::string>& vecFiles,const std::string& dir);
		static void enumFiles(std::vector<std::string>& vecFiles,const std::string& dir);
		static std::string getFileStem(const std::string& path);
		static std::string getFileSuffix(const std::string& path);
		static std::string getFatherDirName(const std::string& path);
	};
}   