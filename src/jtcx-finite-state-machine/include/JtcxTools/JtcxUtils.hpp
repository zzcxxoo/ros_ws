#pragma once

// std
#include <set>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h>

// system
#include <pwd.h>
#include <openssl/md5.h>
#include <sys/stat.h>
#include <unistd.h>

// io
#include <yaml-cpp/yaml.h>
#include <jsoncpp/json/json.h>
#include <boost/filesystem.hpp>

// time 
#include <chrono>

// third-party
#include <fmt/core.h>

// macro
#define bfs boost::filesystem

namespace JtcxUtils
{
    /**
 * @brief 获取用户名
 *
 * @return std::string
 */
std::string getUserName();

/**
 * @brief 获取指定目录下的文件名(不进行递归搜索)
 *
 * @param path 路径名
 * @return std::vector<std::string> 文件名
 */
std::vector<std::string> getFileName(const std::string &path);

/**
 * @brief 获取指定目录下指定后缀的文件名(不进行递归搜索)
 *
 * @param path 路径名
 * @param temp 文件后缀名
 * @return std::vector<std::string> 去后缀文件名
 */
std::vector<std::string> getFileName(const std::string &path,
									 const std::string &temp);

/**
 * @brief  调用system执行shell命令(串行执行)
 *
 * @param cmd   shell command
 */
void open_system(const std::string &cmd);

/**
 * @brief 调用popen执行shell命令(并行执行)
 *
 * @param cmd shell command
 * @return 返回状态
 */
int open_popen(const std::string &cmd);

/**
 * @brief 调用popen执行shell命令(并行执行
 *
 * @param cmd  shell command
 * @param out  command 输出值
 * @return 返回状态
 */
int open_popen(const std::string &cmd, std::vector<std::string> &out);
    
/**
 * @brief 
 * 
 * @param node_name : format like: /lio_samxxx, "rosnode list" and you will know
 * @return true : node is alive
 * @return false 
 */
inline bool checkNodeAlive(const std::string& node_name)
{
    std::vector<std::string> out;
    std::string cmd = fmt::format("rosnode info {} 2>&1", node_name);
    open_popen(cmd, out);

    auto it = std::find_if(out.begin(), out.end(), [](const std::string& e){
        return (e.find("ERROR") != e.npos || e.find("error") != e.npos);
    });

    return (it != out.end());
}



} // namespace JtcxUtils


