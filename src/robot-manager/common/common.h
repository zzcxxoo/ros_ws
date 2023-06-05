/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <dirent.h>
#include <jsoncpp/json/json.h>
#include <openssl/md5.h>
#include <pwd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <cstring>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <set>
#include <alibabacloud/oss/OssClient.h>
#include <xmlrpcpp/XmlRpcException.h>
#include "nlohmann/json.hpp"
#include <uuid/uuid.h>

#include <chrono>

#include <mobile_platform_msgs/SystemdService.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>

#define USER_NAME getUserName()
#define HOME_DIR "/home/" + USER_NAME
#define MAP_DIRECTORY HOME_DIR + "/.robot/data/maps/"
#define G_CONST_LOOP_COUNT -1
#define DATA_DIRECTORY HOME_DIR + "/.robot/data/"
#define LOG_DIR HOME_DIR + "/.robot/data/logger/cleaning/"

#define BF boost::filesystem
using json = nlohmann::json;

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
 * @param con  文件名前缀
 * @return std::vector<std::string> 去后缀文件名
 */
std::vector<std::string> getFileName(const std::string &path,
									 const std::string &temp,
                                     const std::string con = "");

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
 * @brief 删除指定目录下的文件夹(递归删除)
 *
 * @param path
 * @return int
 */
bool rm_dir(const std::string &path);

/**
 * @brief
 *
 * @param sourcefile
 * @param distfile
 * @return int
 */
int copy_file(const std::string &sourcefile, const std::string &distfile);

/**
 * @brief
 *
 * @param bytes_to_encode
 * @param in_len
 * @return std::string
 */
std::string base64_encode(unsigned char const *bytes_to_encode,
						  unsigned int in_len);

/**
 * @brief
 *
 * @param encoded_string
 * @return std::string
 */
std::string base64_decode(std::string const &encoded_string);

/**
 * @brief
 *
 * @param str
 * @return std::string
 */
std::string md5sum(const std::string &str);

bool YamlToJson(const YAML::Node &ynode, Json::Value &jnode);

bool is_number(const std::string &s);

std::string getEtherMac();

std::string getSelectedMap();

bool getEgoStatus(std::string);

template <class T>
void responseHelper(T& resp, int st, std::string msg){
	resp.status = st;
	resp.message = msg;
}

template<typename T>
void cropNum(T& num, T min, T max)
{
    num = std::max(min, std::min(num, max));
}

template<class LOG, class T>
inline void serviceHelper(LOG& log, T& res, std::string msg, int err)
{
    if(err == 0){
        log->info("{}", msg);
    }else{
        log->error("{}", msg);
    }
    res.status = err;
    res.message = msg;
}

template<class LOG, class T>
inline void agentHelper(LOG& log, T& res, std::string msg, int err)
{
    if(err == 0){
        log->info("{}", msg);
    }else{
        log->error("{}", msg);
    }
    res.code = err;
    res.msg = msg;
}

bool IsFileExistent(const BF::path &path);

std::vector<std::string> getAllMaps();

/**
 * @brief Get the Pose From Two Point object
 * 
 * @param p1 
 * @param p2 
 * @return geometry_msgs::Pose : return position of the pose is the same as p1 !!
 */
geometry_msgs::Pose getPoseFromTwoPoint(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2);

inline geometry_msgs::PointStamped pointToPointStamed (const geometry_msgs::Point &p) 
{
	geometry_msgs::PointStamped res;
    res.header.frame_id = "map";
    res.header.stamp = ros::Time::now();
	res.point = p;
	return res;
}

inline geometry_msgs::PoseStamped poseToPoseStamed(const geometry_msgs::Pose &p) {
	geometry_msgs::PoseStamped res;
	res.header.frame_id = "map";
    res.header.stamp = ros::Time::now();
    res.pose = p;
	return res;
}
