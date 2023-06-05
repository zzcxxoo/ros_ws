#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <alibabacloud/oss/OssClient.h>
#include <openssl/md5.h>
#include "nlohmann/json.hpp"
#include <yaml-cpp/yaml.h>
#include <uuid/uuid.h>
#include <boost/filesystem.hpp>

#include "mobile_platform_msgs/Agent.h"
using json = nlohmann::json;
using namespace mobile_platform_msgs;
#define bfs boost::filesystem

int open_popen(const std::string &cmd) {
    FILE *fp;
    fp = popen(cmd.c_str(), "r");
    if (!fp) std::cerr << "Couldn't start command" << std::endl;
    return pclose(fp);
}

int open_popen(const std::string &cmd, std::vector<std::string> &out) {
    FILE *fp;
    const int sizebuf = 2048;
    char buff[sizebuf];
    out = std::vector<std::string>();
    fp = popen(cmd.c_str(), "r");
    if (!fp) std::cerr << "Couldn't start command" << std::endl;

    while (fgets(buff, sizeof(buff), fp)) {
    std::string cur_string = "";
    cur_string += buff;
    out.push_back(cur_string.substr(0, cur_string.size()));
    }
    return pclose(fp);
}

bool checkPathExist(std::string fn) noexcept
{
    return bfs::exists(bfs::path(fn));
}

std::string md5sum(const std::string &str)
{
    std::string md5;
    MD5_CTX ctx;
    MD5_Init(&ctx);
    MD5_Update(&ctx, str.c_str(), str.size());
    unsigned char digest[MD5_DIGEST_LENGTH];
    MD5_Final(digest, &ctx);
    char hex[35];
    memset(hex, 0, sizeof(hex));
    for (int i = 0; i < MD5_DIGEST_LENGTH; ++i)
    {
        sprintf(hex + i * 2, "%02x", digest[i]);
    }
    md5 = std::string(hex);
    return md5;
}

std::string genUUID() noexcept
{
    uuid_t uuid;
    char str[36];

    uuid_generate(uuid);
    uuid_unparse(uuid, str);

    return std::string(str);
}

static const std::string MAP_DIR = std::string(std::getenv("HOME")) + "/.robot/data/maps/";

class OssUploadHelper
{
private:

    ros::NodeHandle& nh_;
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

public:

    explicit OssUploadHelper(ros::NodeHandle& nh) : nh_(nh)
    {
        // get sn code by service
        Agent srv;
        srv.request.trace_id = genUUID();
        srv.request.data = "{}";
        std::string res;
        if(!callAgent(AGENT_GET, "device_meta", json{}, res)){
            throw std::runtime_error("sn code can not get!!");
        }
    
        json j = json::parse(res);
        ROS_INFO("device meta: %s", res.c_str());
        sn_code_ = j["device_sn"];
        ROS_INFO("sn code: %s", sn_code_.c_str());

        // get oss param
        if(!ros::param::get("/oss/oss_url", oss_url_) ||
            !ros::param::get("/oss/cloud_server", cloud_server_) || 
            !ros::param::get("/oss/token_request_url", oss_token_request_url_) || 
            !ros::param::get("/oss/oss_bucket", oss_bucket_) ||
            !ros::param::get("/oss/upload_meta_url", oss_upload_meta_url_))
        {
            throw ros::Exception("oss ros param is not get!!");
        }

        oss_token_request_url_ = cloud_server_ + oss_token_request_url_;
        oss_upload_meta_url_ = cloud_server_ + oss_upload_meta_url_;
        
        ROS_INFO("oss token request url: %s", oss_token_request_url_.c_str());
        ROS_INFO("oss upload meta url: %s", oss_upload_meta_url_.c_str());

        // get sts token at first time, may be throw exception
        getStsToken();

    }

    void setMapId(std::string m){
        map_id_ = m;
        prefix_local_dir_ = std::string(std::getenv("HOME")) + "/.robot/data/maps/" + m + "/";
        prefix_oss_dir_ = "vcu/" + sn_code_ + "/map/" + m + "/";
    }

    void getStsToken()
    {
		auto now = std::chrono::system_clock::now();
		auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - get_token_time_).count();
        if(diff > token_timeout_){
			get_token_time_ = now;

			ROS_WARN("get sts token for %d > %d!!", diff, token_timeout_);
            std::vector<std::string> token_response;
            open_popen(
                "curl -X 'GET' '" + oss_token_request_url_ + "' -H"
                "'accept:application/json'",
                token_response);

            if(token_response.empty()){
                throw std::runtime_error("curl can not get token!!");
            }

            auto root = json::parse(token_response[0]);
            if (root["code"].get<int>() == 0) {
                key_secret_ = root["data"]["AccessKeySecret"];
                key_id_ = root["data"]["AccessKeyId"];
                key_token_ = root["data"]["SecurityToken"];
                return;
            }
            throw std::runtime_error("sts error code: " + std::to_string(root["code"].get<int>()));
        }  
    }

    /**
     * @brief 
     * 
     * @param lc_fn : name with extension
     * @param oss_fn : name with extension
     * @return true 
     * @return false 
     */
	bool uploadOssFile(std::string lc_fn, std::string oss_fn){
        if(lc_fn.empty() || oss_fn.empty())   return false;

        getStsToken();
        
        AlibabaCloud::OSS::ClientConfiguration conf;
        AlibabaCloud::OSS::OssClient client(oss_url_, key_id_, key_secret_, key_token_, conf);

        std::time_t t = std::time(nullptr) + 1200;
        std::string oss_file = prefix_oss_dir_ + oss_fn;
        ROS_INFO("get oss file: %s", oss_file.c_str());
        auto genOutcome = client.GeneratePresignedUrl(oss_bucket_, oss_file, t, AlibabaCloud::OSS::Http::Put);

        if (!genOutcome.isSuccess()) {
            ROS_ERROR_STREAM("GeneratePresignedUrl fail"
                                    << ",code:" << genOutcome.error().Code()
                                    << ",message:" << genOutcome.error().Message()
                                    << ",requestId:" << genOutcome.error().RequestId());
            return false;
        }

        ROS_INFO_STREAM("GeneratePresignedUrl success, Gen url:" << genOutcome.result());

        std::string localfile = prefix_local_dir_ + lc_fn;
        ROS_INFO("get local file: %s", localfile.c_str());

        // ----------------- 4. upload oss by url and sign ---------------------
        auto outcome = client.PutObjectByUrl(genOutcome.result(), localfile);
        if (!outcome.isSuccess()) {
            ROS_ERROR_STREAM("PutObject fail" << ", code:" << outcome.error().Code()
                            << ", message:" << outcome.error().Message()
                            << ", requestId:" << outcome.error().RequestId());
            ROS_ERROR("upload map failed!");
            return false;
        }
        ROS_INFO("upload file %s success!!", localfile.c_str());
        return true;
    }
   
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
    bool callAgent(std::string topic, std::string srv_id, const json& msg, std::string& res)
    {
        Agent srv;
        srv.request.service_id = srv_id;
        srv.request.trace_id = genUUID();
        srv.request.data = msg.dump();
        ROS_INFO("req: %s", srv.request.data.c_str());
        if(ros::service::call(topic, srv)){
            if(srv.response.code == 0){
                res = srv.response.result;
                return true;
            }
            ROS_ERROR("agent call fail for: %s", srv.response.msg.c_str());
        }else
            ROS_ERROR("agent offline!!");
     
        return false;
    }

    /**
     * @brief 
     * 
     * @param map_id : map unique id
     * @return true 
     * @return false 
     */
    bool uploadMapMetaInfo(std::string img_fn, std::string yaml_fn){
        
        // ------- construct format -----------
        json info;
        std::vector<json> docs;
        json doc;
        doc["name"] = img_fn + ".png";
        doc["oss_url"] = prefix_oss_dir_ + img_fn + ".png";
        docs.push_back(doc);
        doc["name"] = yaml_fn + ".yaml";
        doc["oss_url"] = prefix_oss_dir_ + yaml_fn + ".yaml";
        docs.push_back(doc);

        auto map_json_fn = std::string(std::getenv("HOME")) + "/.robot/data/maps/map.json";
        std::ifstream inf(map_json_fn);
        auto j = json::parse(inf);
        inf.close();
        info["map_id"] = map_id_;
        info["map_name"] = j[map_id_];
        info["map_desc"] = "";       
        info["map_docs"] = docs;

        std::string _tmp;
        return callAgent(AGENT_UPLOAD, "map_meta", info, _tmp);
    }

    /**
     * @brief upload zone meta with one specific json zone info
     * 
     * @param zone_info 
     * @return true 
     * @return false 
     */
    bool uploadZoneMetaInfo(const json& zone_info){
        json msg, zones;
        msg["map_id"] = map_id_;
        
        std::string zone_id = zone_info["dir_name"];
        zones["zone_id"] = zone_id;
        zones["zone_name"] = zone_info["name"];
        zones["rect"] = zone_info["rect"];
        json docs;
        docs["name"] = "mask.png";
        docs["oss_url"] = prefix_oss_dir_ + "zone/" + zone_id + "/mask.png";
        zones["zone_docs"].push_back(docs);
        
        docs["name"] = "edge.csv";
        docs["oss_url"] = prefix_oss_dir_ + "zone/" + zone_id + "/edge.csv";
        zones["zone_docs"].push_back(docs);

        docs["name"] = "coverage.csv";
        docs["oss_url"] = prefix_oss_dir_ + "zone/" + zone_id + "/coverage.csv";
        zones["zone_docs"].push_back(docs);

        docs["name"] = "both.csv";
        docs["oss_url"] = prefix_oss_dir_ + "zone/" + zone_id + "/both.csv";
        zones["zone_docs"].push_back(docs);
        
        msg["zones"].push_back(zones);

        std::string _tmp;
        return callAgent(AGENT_UPLOAD, "zone_meta", msg, _tmp);
    }

    /**
     * @brief upload meta with one zone id 
     * 
     * @param zone_id 
     * @return true 
     * @return false 
     */
    bool uploadZoneMetaInfo(std::string zone_id){
        std::ifstream inf(MAP_DIR + map_id_ + "/zone/partition.json");
        json part_info = json::parse(inf)["zone_info"];
        inf.close();

        bool valid = false;
        for(const auto& e : part_info){
            if(e["dir_name"].get<std::string>() == zone_id){
                valid = uploadZoneMetaInfo(e);
                break;
            }
        }   
        return valid;
    }

    /**
     * @brief : upload all zone meta info
     * 
     * @return true 
     * @return false 
     */
    bool uploadZoneMetaInfo(){
        std::ifstream inf(MAP_DIR + map_id_ + "/zone/partition.json");
        json part_info = json::parse(inf)["zone_info"];
        inf.close();

        for(const auto& e : part_info){
            if(!uploadZoneMetaInfo(e))  return false;
        }        
        return true;
    }

    bool uploadPathMetaInfo(std::string path_id){
        json j;
        j["map_id"] = map_id_;
        j["path_id"] = path_id;
        j["path_name"] = path_id;
        json doc;
        doc["name"] = path_id + ".yaml";
        doc["oss_url"] = prefix_oss_dir_ + "path/" + path_id + ".yaml";
        j["path_docs"].push_back(doc);
     
        json jj;
        jj["map_id"] = map_id_;
        jj["path_id"] = path_id;
        jj["plan_desc"] = path_id;
        jj["times"] = 1;

        std::string _tmp;
        return callAgent(AGENT_UPLOAD, "path_meta", j, _tmp) && callAgent(AGENT_UPLOAD, "tracking", jj, _tmp);
    }

    bool resetZoneMetaInfo()
    {
        json j;
        j["map_id"] = map_id_;
        std::string _tmp;
        return callAgent(AGENT_RESET, "zone_meta", j, _tmp);
    }

};


void test_upload_map(OssUploadHelper* ptr)
{
    std::string map_id = "cba";
    ptr->setMapId(map_id);

    ROS_INFO("you can upload oss file now");

    if(ptr->uploadOssFile(map_id + ".png", map_id + ".png") && ptr->uploadOssFile(map_id + ".yaml", map_id + ".yaml")){
        ROS_INFO("upload oss file success!");
        if(ptr->uploadMapMetaInfo(map_id, map_id)){
            ROS_INFO("upload map meta info also success!");
        }
    }
    
}

void test_upload_zone(OssUploadHelper* ptr)
{
    std::string map_id = "cba";
    std::string zone_dir = MAP_DIR + map_id + "/zone/";
    ptr->setMapId(map_id);

    ROS_INFO("you can upload zone now");

    std::string zone_prefix = "zone_";

    for (int i = 0; i < 10; i++)
    {
        std::string zone_id = zone_prefix + std::to_string(i);
        auto s_zone_dir = zone_dir + zone_id + "/";

        if(!checkPathExist(s_zone_dir))    break;

        // upload mask
        std::string mask_name = "zone/" + zone_id + "/mask.png";
        std::string mask_show_name = "zone/" + zone_id + "/mask_show.png";
        std::string edge_name = "zone/" + zone_id + "/edge.csv";
        std::string coverage_name = "zone/" + zone_id + "/coverage.csv";
        std::string both_name = "zone/" + zone_id + "/both.csv";
        ROS_INFO("try to upload mask: %s", mask_name.c_str());

        std::string cmd = "touch ";
        if(!checkPathExist(s_zone_dir + "coverage.csv"))    open_popen(cmd + s_zone_dir + "coverage.csv");    
        if(!checkPathExist(s_zone_dir + "both.csv"))    open_popen(cmd + s_zone_dir + "both.csv");    

        ptr->uploadOssFile(mask_show_name, mask_name);
        ptr->uploadOssFile(edge_name, edge_name);
        ptr->uploadOssFile(coverage_name, coverage_name);
        ptr->uploadOssFile(both_name, both_name);
    }

    if(!ptr->uploadZoneMetaInfo())  ROS_ERROR("upload zone meta info fail!!");

}

void test_upload_one_zone_meta(OssUploadHelper* ptr, std::string zone_id)
{
    std::string map_id = "cba";
    ptr->setMapId(map_id);

    ROS_INFO("you can upload zone meta now");

    if(ptr->uploadZoneMetaInfo(zone_id)){
        ROS_INFO("upload success!!");
        return;
    }
    ROS_ERROR("upload fail!!");
}

void test_upload_path(OssUploadHelper* ptr, std::string path_id)
{
    std::string map_id = "cba";
    ptr->setMapId(map_id);

    std::string fn = "path/" + path_id + ".yaml";

    ROS_INFO("try to upload file: %s", fn.c_str());

    ptr->uploadOssFile(fn, fn);
    ptr->uploadPathMetaInfo(path_id);

}

void test_reset_zone(OssUploadHelper* ptr)
{
    std::string map_id = "cba";
    ptr->setMapId(map_id);
    ptr->resetZoneMetaInfo();
}

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "test_oss_node");
    ros::NodeHandle nh;

    OssUploadHelper ouh(nh);

    // test_upload_map(&ouh);
    test_upload_zone(&ouh);
    // test_upload_one_zone_meta(&ouh, "zone_0");
    // test_reset_zone(&ouh);
    test_upload_path(&ouh, "edge1330");

    return 0;
}


