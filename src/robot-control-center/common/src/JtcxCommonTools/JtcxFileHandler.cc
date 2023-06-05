/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxFileHandler.cpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/11/26
Last Modified :
Description   : File and folder processing
Function List : 
History       : first edition ----20221125
******************************************************************************/
#include "JtcxFileHandler.hpp"
#include <boost/filesystem.hpp>
#define bfs boost::filesystem

namespace JTCX
{
	bool JtcxDir::existDir(const std::string& dir)
	{
		return bfs::exists(bfs::path(dir));
	}

	void JtcxDir::createDir(const std::string& dir)
	{
		bfs::create_directories(bfs::path(dir));
	}

	void JtcxDir::deleteDir(const std::string& dir)
	{
		bfs::remove_all(bfs::path(dir));
	}

	void JtcxDir::enumCSVFiles(std::vector<std::string>& vecFiles,const std::string& dir)
	{

		vecFiles.clear();
		bfs::path path(dir);
		for (const auto& iter : bfs::directory_iterator(path))
		{
			if (bfs::is_directory(iter.path()))//过滤子文件夹
				continue;

			std::string extension = iter.path().extension().string();
			char csv[] = ".csv";
			if (JtcxStrCmp(extension.c_str(), csv) == 0) 
			{	
				vecFiles.push_back(iter.path().string());
			}
		}
	}

	void JtcxDir::enumFiles(std::vector<std::string>& vecFiles,const std::string& dir)
	{
		vecFiles.clear();
		bfs::path path(dir);
		for (const auto& iter : bfs::directory_iterator(path))
		{
			if (bfs::is_directory(iter.path()))//过滤子文件夹
				continue;

			std::string file = iter.path().string();
			vecFiles.push_back(file);
		}
	}

	std::string JtcxDir::getFileStem(const std::string& path)
	{
		return bfs::path(path).stem().string();
	}

	std::string JtcxDir::getFileSuffix(const std::string& path)
	{
		return bfs::path(path).extension().string();
	}

	std::string JtcxDir::getFatherDirName(const std::string& path)
	{
		return bfs::path(path).parent_path().string();
	}
}                                                                                             