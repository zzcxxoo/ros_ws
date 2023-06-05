#include "param_manager.h"
#include <common/OssUploadHelper.hpp>

static JtcxLogWrapper lr("param_manager", LOG_LEVEL::DEBUG);
JtcxLogWrapper* jlw_for_param_manager = &lr;

int main(int argc, char **argv) {
  // init log in advance
  lr.initLogFile("/var/log/jtcx/robot-manager", "param_manager.log");
  OssUploadHelper::lg = std::make_shared<JtcxLogWrapper>(lr);

  ros::init(argc, argv, "ui_param");
  ros::NodeHandle n;
  ParamManager parammanager;
  parammanager.Init(&n);
  ros::spin();
  return 0;
}