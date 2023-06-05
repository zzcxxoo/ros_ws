/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "job_manager.h"
#include "map_manager.h"
#include <common/OssUploadHelper.hpp>

static JtcxLogWrapper lr("check_services", LOG_LEVEL::DEBUG);
JtcxLogWrapper* jlw_for_job_manager = &lr;
JtcxLogWrapper* jlw_for_map_manager = &lr;

int main(int argc, char **argv) {
	// init log in advance!!
  lr.initLogFile("/var/log/jtcx/robot-manager", "check_services.log");
  OssUploadHelper::lg = std::make_shared<JtcxLogWrapper>(lr);
  
  ros::init(argc, argv, "service_and_map_manager");
  ros::NodeHandle n;
  JobManager jobmanager;
  MapManager mapmanager;
  jobmanager.Init(&n);
  mapmanager.Init(&n);
  mapmanager.setJobmanagerPtr(&jobmanager);
  ros::spin();
  return 0;
}
