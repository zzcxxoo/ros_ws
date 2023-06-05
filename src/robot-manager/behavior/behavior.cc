/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "waypoint_manager.h"
#include "status_report.h"
#include "common/JtcxLogWrapper.hpp"

static JtcxLogWrapper lr("behavior", LOG_LEVEL::DEBUG);
JtcxLogWrapper* jlw_for_status_report = &lr;

int main(int argc, char** argv) {
	lr.initLogFile("/var/log/jtcx/robot-manager", "behavior.log");
	
	ros::init(argc, argv, "behave");
	ros::NodeHandle n;

	WaypointManager manager;
	WaypointManager::s_logger = std::make_shared<JtcxLogWrapper>(lr);
	manager.Init(&n);
	
	StatusReport sr;

	ros::spin();
	return 0;
}