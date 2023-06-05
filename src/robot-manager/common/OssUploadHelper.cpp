#include "OssUploadHelper.hpp"

static const std::string MAP_DIR = std::string(std::getenv("HOME")) + "/.robot/data/maps/";
std::shared_ptr<JtcxLogWrapper> OssUploadHelper::lg{nullptr};
#define OLG OssUploadHelper::lg->logger

void OssUploadHelper::init()
{
    // get sn code by service
    std::string res;
    json mj = json::object();
    if(!callAgent(AGENT_GET, "device_meta", mj, res)){
        throw std::runtime_error("sn code can not get!!");
    }

    json j = json::parse(res);
    sn_code_ = j["device_sn"];
    if(lg.get() == nullptr){
        throw std::runtime_error("logger is not init!!");
    }
    
    OLG->trace("sn code: {}", sn_code_);

    // get oss param
    if(!ros::param::get("/oss/oss_url", oss_url_) ||
        !ros::param::get("/oss/cloud_server", cloud_server_) || 
        !ros::param::get("/oss/token_request_url", oss_token_request_url_) || 
        !ros::param::get("/oss/oss_bucket", oss_bucket_) ||
        !ros::param::get("/oss/upload_meta_url", oss_upload_meta_url_))
    {
        OLG->error("oss ros param is not get!!");
        throw ros::Exception("oss ros param is not get!!");
    }

    oss_token_request_url_ = cloud_server_ + oss_token_request_url_;
    oss_upload_meta_url_ = cloud_server_ + oss_upload_meta_url_;
    
    OLG->trace("oss token request url: {}", oss_token_request_url_);
    OLG->trace("oss upload meta url: {}", oss_upload_meta_url_);

    // get sts token at first time, may be throw exception
    getStsToken();
}

OssUploadHelper& OssUploadHelper::getInstance()
{
    static OssUploadHelper oss;
    return oss;
}

void OssUploadHelper::setMapId(std::string m)
{
    map_id_ = m;
    prefix_local_dir_ = MAP_DIR + m + "/";
    prefix_oss_dir_ = "vcu/" + sn_code_ + "/map/" + m + "/";
}

void OssUploadHelper::getStsToken()
{
    auto now = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::seconds>(now - get_token_time_).count();
    if(diff > token_timeout_){
        get_token_time_ = now;

        OLG->info("get sts token for {} > {}!!", diff, token_timeout_);
        std::vector<std::string> token_response;
        open_popen(
            "curl -X 'GET' '" + oss_token_request_url_ + "' -H"
            "'accept:application/json'",
            token_response);

        if(token_response.empty()){
            OLG->error("curl can not get token!!");
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

bool OssUploadHelper::uploadOssFile(std::string lc_fn, std::string oss_fn)
{
    if(lc_fn.empty() || oss_fn.empty())   return false;

    getStsToken();
    
    AlibabaCloud::OSS::ClientConfiguration conf;
    AlibabaCloud::OSS::OssClient client(oss_url_, key_id_, key_secret_, key_token_, conf);

    std::time_t t = std::time(nullptr) + 1200;
    std::string oss_file = prefix_oss_dir_ + oss_fn;
    OLG->trace("get oss file: {}", oss_file);
    auto genOutcome = client.GeneratePresignedUrl(oss_bucket_, oss_file, t, AlibabaCloud::OSS::Http::Put);

    if (!genOutcome.isSuccess()) {
        OLG->error("GeneratePresignedUrl fail code: {}, message: {}, requestId: {}",
                    genOutcome.error().Code(), genOutcome.error().Message(), genOutcome.error().RequestId());
        return false;
    }

    OLG->trace("GeneratePresignedUrl success, Gen url: " + genOutcome.result());

    std::string localfile = prefix_local_dir_ + lc_fn;
    OLG->trace("get local file: {}", localfile);

    // ----------------- 4. upload oss by url and sign ---------------------
    auto outcome = client.PutObjectByUrl(genOutcome.result(), localfile);
    if (!outcome.isSuccess()) {
        OLG->error("GeneratePresignedUrl fail code: {}, message: {}, requestId: {}",
                    outcome.error().Code(), outcome.error().Message(), outcome.error().RequestId());
        OLG->error("upload map failed!");
        return false;
    }
    OLG->info("upload file {} success!!", localfile);
    return true;
}

bool OssUploadHelper::callAgent(std::string topic, std::string srv_id, const json& msg, std::string& res)
{
    Agent srv;
    srv.request.service_id = srv_id;
    srv.request.trace_id = genUUID();
    srv.request.data = msg.dump();
    OLG->trace("req: {}", srv.request.data);
    if(ros::service::call(topic, srv)){
        if(srv.response.code == 0){
            res = srv.response.result;
            return true;
        }
        OLG->error("agent call fail for: {}", srv.response.msg);
    }else
        OLG->error("agent offline!!");
    
    return false;
}

bool OssUploadHelper::uploadMapMetaInfo(std::string img_fn, std::string yaml_fn){
    
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

bool OssUploadHelper::uploadZoneMetaInfo(const json& zone_info){
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

bool OssUploadHelper::uploadZoneMetaInfo(std::string zone_id){
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

bool OssUploadHelper::uploadZoneMetaInfo(){
    std::ifstream inf(MAP_DIR + map_id_ + "/zone/partition.json");
    json part_info = json::parse(inf)["zone_info"];
    inf.close();

    for(const auto& e : part_info){
        if(!uploadZoneMetaInfo(e))  return false;
    }        
    return true;
}

bool OssUploadHelper::uploadPathMetaInfo(std::string path_id){
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

bool OssUploadHelper::resetZoneMetaInfo()
{
    json j;
    j["map_id"] = map_id_;
    std::string _tmp;
    return callAgent(AGENT_RESET, "zone_meta", j, _tmp);
}