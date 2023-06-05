//
// Created by jtcx on 8/25/22.
//

/**
 * @copyright Copyright <JT-Innovation> (c) 2022
 */

#include "cleaning_logger.h"

#include "common/common.h"
#include "mobile_platform_msgs/MappingSave.h"

void CleaningLogger::Init(ros::NodeHandle *nh_ptr) {
  nh_ptr_ = nh_ptr;
  totalPointsNum_ = -1;
  enableAll_ = false;

  loggers_.push_back(logger_);
  chassisSub_ = nh_ptr_->subscribe("/chassis", 1,
                                   &CleaningLogger::ChassisMsgHandler, this);
  purePursuitStartSub_ =
      nh_ptr_->subscribe("/pure_pursuit/tracking_path", 1,
                         &CleaningLogger::PurePursuitStartHandler, this);
  purePursuitStatusSub_ =
      nh_ptr_->subscribe("/pure_pursuit/status", 1,
                         &CleaningLogger::PurePursuitStatusHandler, this);
  purePursuitResultSub_ =
      nh_ptr_->subscribe("/pure_pursuit/tracking_result", 1,
                         &CleaningLogger::PurePursuitResultHandler, this);
  twistSub_ = nh_ptr_->subscribe("/nav/cmd_vel", 1,
                                 &CleaningLogger::TwistHandler, this);

  cleaningLoggerSrv = nh_ptr_->serviceClient<mobile_platform_msgs::MappingSave>(
      "/ui/logger/cleaning_logger");

  cleaningStatusPub =
      nh_ptr_->advertise<std_msgs::String>("/ui/logger/cleaning_status", 1);
}

void CleaningLogger::ChassisMsgHandler(const ChassisConstPtr &msg) {
  if (enableAll_) {
    logger_.mode = modeList_[msg->driving_mode];
    if (logger_.mode == "VCU_EMERGENCY_STOP" || logger_.mode == "IPC_EMERGENCY")
      logger_.exception = "Error";
  }
}

// 只会在开始计划的时候回调
// 一开始判断拿到的名字是不是正常的路径名，如果是召回名字，则不上传清洁记录，所有记录操作都disable
void CleaningLogger::PurePursuitStartHandler(
    const nav_msgs::PathConstPtr &msg) {
  enableAll_ = true;
  std::string path_name;
  if (ros::param::get("/tracking_path_name", path_name) &&
      path_name.find("jt_hom") != std::string::npos)
    enableAll_ = false;
  ros::param::get("/ui/cleaning_plan/times", cleaning_times_);
  // 如果无限循环，则不记录
  if (cleaning_times_ == -1) {
    enableAll_ = false;
    return;
  }

  logger_.start_time = ros::Time::now().toSec();
  totalPointsNum_ = msg->poses.size() * cleaning_times_;
  ROS_WARN("start cleaning: %f\n", totalPointsNum_);
  ros::param::set("/ui/debug/path_pts_num", totalPointsNum_);
  totalArea_ = 0.2 * totalPointsNum_ * 0.8;
  if (totalPointsNum_ == 0) {
    ROS_ERROR("start cleaning but num of path pose is 0!! disable cleaning!!");
    enableAll_ = false;
  }
}

// 不断接收跟踪器发布的状态
void CleaningLogger::PurePursuitStatusHandler(
    const PurePursuitStatusConstPtr &msg) {
  if (enableAll_ && msg->state == "WORKING") {
    task_index = msg->task_index;

    float wp = float(msg->waypoint);
    if (wp < 0) wp = 0;

    // 面积和里程都要关联进度,所以进度一定要合法
    float cur_process =
        1.0 * (task_index - 1) / cleaning_times_ + wp / totalPointsNum_;
    cropNum<float>(cur_process, 0.0, 1.0);

    ROS_DEBUG_NAMED("PurePursuitStatusHandler", "cur_process: %f", cur_process);
    if ((logger_.process == INT_MIN) || (cur_process > logger_.process))
      logger_.process = cur_process;

    logger_.area = logger_.process * totalArea_;
    logger_.odometry = logger_.area / 0.8;

    //    ---------------- pub cleaning status -----------------
    Json::Value jv;
    std::string path_name;
    ros::param::get("/ui/cleaning_plan/path", path_name);

    // //
    // 使用task_index来设置跟踪次数，循环不在中控做处理，否则会出现不可控错误！！！
    //  if path name ends with _times, remove it !!!!
    // size_t pos = path_name.rfind("_times");
    // if(pos != std::string::npos){
    // 	path_name = path_name.substr(0, pos);
    // }

    jv["plan_name"] = path_name;
    jv["clean_times"] = cleaning_times_;
    // jv["current_times"] = int(cur_process * tmp) + 1;
    jv["current_times"] = msg->task_index;

    jv["area"] = logger_.area;
    jv["odometry"] = logger_.odometry;
    jv["duration"] = ros::Time::now().toSec() - logger_.start_time;
    std_msgs::String msg;
    msg.data = Json::FastWriter().write(jv);
    cleaningStatusPub.publish(msg);
  }
}

// 跟踪器结束的回调
void CleaningLogger::PurePursuitResultHandler(
    const PurePursuitResultConstPtr &msg) {
  logger_.map = msg->map;
  logger_.path = msg->path;
  if (enableAll_) {
    double cur_time = ros::Time::now().toSec();
    if (msg->tracking_result == "Manual_Stop") {
      ROS_WARN("stop cleaning !");
      logger_.exception = "Error";
      logger_.cleaning_time = cur_time - logger_.start_time;
    }
    if (msg->tracking_result == "Success") {
      ROS_WARN("cleaning success!");
      logger_.cleaning_time = cur_time - logger_.start_time;
      logger_.exception = "Normal";
      logger_.process = 1.0;
    }

    if (msg->tracking_result == "Fail") {
      ROS_WARN("cleaning fail!");
      logger_.cleaning_time = cur_time - logger_.start_time;
      logger_.exception = "Error";
    }
    // 检查清洁记录的合法性
    if (CheckLoggerParamsValid()) SaveLoggerToFile();

    ResetLogger();
  }
}

void CleaningLogger::assignYamlValue(YAML::Node &config, int index) {
  config[index]["mac_addr"] = getEtherMac();
  config[index]["area"] = logger_.area;
  config[index]["average_velocity"] = logger_.average_velocity;
  config[index]["cleaning_time"] = logger_.cleaning_time;
  config[index]["exception"] = logger_.exception;
  config[index]["map"] = logger_.map;
  config[index]["mode"] = logger_.mode;
  config[index]["path"] = logger_.path;
  config[index]["process"] = logger_.process;
  config[index]["start_time"] = logger_.start_time;
  config[index]["odometry"] = logger_.odometry;
}

void CleaningLogger::SaveLoggerToFile() {
  if (!BF::exists(BF::path(LOG_DIR))) {
    ROS_DEBUG_NAMED("SaveLoggerToFile", "create file %s",
                    std::string(LOG_DIR).c_str());
    BF::create_directories(LOG_DIR);
  }

  // if (!access((LOG_DIR).c_str(), F_OK))
  // mkdir((LOG_DIR).c_str(), S_IRWXU);
  std::string LOG_FILE = LOG_DIR + "cleaning_logger.yaml";
  ROS_DEBUG_NAMED("SaveLoggerToFile", "log file is %s", LOG_FILE.c_str());

  // 1. 如果不存在，先创建
  if (!BF::exists(BF::path(LOG_FILE))) {
    std::ofstream out(LOG_FILE);
    if (!out.is_open()) {
      ROS_ERROR("fail to create cleaning_logger.yaml");
      out.close();
      return;
    }
    out.close();
  }

  // 2. 已经存在了, 先读！！
  YAML::Node config = YAML::LoadFile(LOG_FILE);
  ROS_WARN("config size: %d", config.size());
  assignYamlValue(config, config.size());

  std::ofstream out(LOG_FILE);
  if (config.size() > 100) {
    YAML::Node ds_config;
    int idx = 0;
    // 只保留最近的100个
    for (int i = config.size() - 100; i < config.size(); i++) {
      ds_config[idx++] = config[i];
    }
    out << ds_config;
  } else {
    out << config;
  }
  out.close();

  // pub logger at this time
  Json::Value jv;
  if (YamlToJson(config[config.size() - 1], jv)) {
    std::string tmp = Json::FastWriter().write(jv);
    mobile_platform_msgs::MappingSave ms;
    ms.request.filename = tmp;
    if (cleaningLoggerSrv.call(ms)) {
      ROS_DEBUG_NAMED("SaveLoggerToFile",
                      "SaveLoggerToFile: code(%d), status(%s)",
                      (int)ms.response.status, ms.response.message.c_str());
    } else {
      ROS_ERROR("SaveLoggerToFile: mqtt log event may be not online!!");
    }
  }
}

void CleaningLogger::TwistHandler(const geometry_msgs::TwistConstPtr &msg) {
  if (enableAll_) {
    if (abs(msg->linear.x) > 0.05) twistQueue_.push_back(msg->linear.x);

    float twistSum = 0;
    for (auto e : twistQueue_) {
      twistSum += e;
    }
    logger_.average_velocity = twistSum / twistQueue_.size();
  }
}

bool CleaningLogger::CheckLoggerParamsValid() {
  if (!enableAll_) return false;

  if (logger_.map.empty() || logger_.path.empty()) {
    ROS_ERROR("map or path name is invalid!!");
    return false;
  }

  if (logger_.area < 0.8) {
    ROS_ERROR("area(%f) is not valid, meaning that process is not valid too!!");
    return false;
  }

  if (logger_.cleaning_time < 10 || logger_.start_time == 0) {
    ROS_WARN("cleaning time(%f) is too short!!", logger_.cleaning_time);
    return false;
  }

  if (logger_.exception.empty()) {
    logger_.exception = "Normal";
  }

  ROS_WARN("CheckLoggerParamValid normal!!!");
  return true;
}

void CleaningLogger::ResetLogger() {
  logger_.map = "";
  logger_.mode = "";
  logger_.path = "";
  logger_.exception = "";
  logger_.process = 0;
  logger_.area = 0;
  logger_.average_velocity = 0;
  logger_.cleaning_time = 0;
  logger_.start_time = 0;
  logger_.odometry = 0;

  twistQueue_.clear();
  enableAll_ = false;

  task_index = 0;
}
