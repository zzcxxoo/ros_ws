#pragma once
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <uuid/uuid.h>
#include <alibabacloud/oss/OssClient.h>
#include <boost/filesystem.hpp>
#define bfs boost::filesystem

#include "nlohmann/json.hpp"
using json = nlohmann::json;

#include <mobile_platform_msgs/Agent.h>
using namespace mobile_platform_msgs;

#include <common/JtcxLogWrapper.hpp>

class OssUploadHelper
{
private:

    ros::NodeHandle nh_;
    std::string sn_code_;

    std::string prefix_local_dir_;
    std::string prefix_oss_dir_;

    std::string map_id_;

    // sts token
    const int token_timeout_ = 1800;
    // double get_token_time_{0};
	std::chrono::time_point<std::chrono::system_clock> get_token_time_;
    std::string key_secret_;
    std::string key_id_;
    std::string key_token_;

    // oss related
    std::string oss_url_;
    std::string cloud_server_;
    std::string oss_token_request_url_;
    std::string oss_upload_meta_url_;
    std::string oss_bucket_;

    // agent topic
    const std::string AGENT_GET = "/agent/provide";
    const std::string AGENT_UPLOAD = "/agent/upload";
    const std::string AGENT_RESET = "/agent/reset";
    
    OssUploadHelper(){init();}
    OssUploadHelper(const OssUploadHelper& o);
    void init();


public:
    // log
    static std::shared_ptr<JtcxLogWrapper> lg;

    static bool checkPathExist(std::string fn) noexcept
    {
        return bfs::exists(bfs::path(fn));
    }

    static std::string genUUID() noexcept
    {
        uuid_t uuid;
        char str[36];

        uuid_generate(uuid);
        uuid_unparse(uuid, str);

        return std::string(str);
    }

    static int open_popen(const std::string &cmd, std::vector<std::string> &out) {
        FILE *fp;
        const int sizebuf = 2048;
        char buff[sizebuf];
        out = std::vector<std::string>();
        fp = popen(cmd.c_str(), "r");
        if (!fp){
            lg->logger->error("open_popen couldn't start command: {}", cmd);
        } 

        while (fgets(buff, sizeof(buff), fp)) {
            std::string cur_string = "";
            cur_string += buff;
            out.push_back(cur_string.substr(0, cur_string.size()));
        }
        return pclose(fp);
    }

    static OssUploadHelper& getInstance();
    
    void setMapId(std::string m);

    void getStsToken();

    /**
     * @brief 
     * 
     * @param lc_fn : name with extension
     * @param oss_fn : name with extension
     * @return true 
     * @return false 
     */
	bool uploadOssFile(std::string lc_fn, std::string oss_fn);
   
    /**
     * @brief 
     * 
     * @param topic 
     * @param srv_id 
     * @param msg 
     * @param res 
     * @return true 
     * @return false 
     */
    bool callAgent(std::string topic, std::string srv_id, const json& msg, std::string& res);
    
    /**
     * @brief 
     * 
     * @param map_id : map unique id
     * @return true 
     * @return false 
     */
    bool uploadMapMetaInfo(std::string img_fn, std::string yaml_fn);

    /**
     * @brief upload zone meta with one specific json zone info
     * 
     * @param zone_info 
     * @return true 
     * @return false 
     */
    bool uploadZoneMetaInfo(const json& zone_info);

    /**
     * @brief upload meta with one zone id 
     * 
     * @param zone_id 
     * @return true 
     * @return false 
     */
    bool uploadZoneMetaInfo(std::string zone_id);

    /**
     * @brief : upload all zone meta info
     * 
     * @return true 
     * @return false 
     */
    bool uploadZoneMetaInfo();

    bool uploadPathMetaInfo(std::string path_id);

    bool resetZoneMetaInfo();

};
