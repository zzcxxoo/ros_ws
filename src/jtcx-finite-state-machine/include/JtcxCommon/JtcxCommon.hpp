#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <future>

#include "JtcxDifines.hpp"
#include "JtcxConcurrenceQueue.hpp"
#include "JtcxInternalEventQueueManager.hpp"
#include "JtcxExternalEventQueueManager.hpp"
#include "JtcxExternalEventsFactory.hpp"

/*
    Some tools for communicating with the state machine used in internal and external event processing。
*/


namespace JTCX {

/// @brief
/// @param abnormalType
void pushAbnormalState(JTCX::RobotAbnormalType abnormalType) {
	JTCX::InternalEventQueueManager::getEventManager().pushAbnormalState(abnormalType);
}

/// @brief This interface emergency mode has the fastest corresponding speed,
/// @brief so use this API with great caution.
void launchEmergentFlag() {
	JTCX::InternalEventQueueManager::getEventManager().launchEmergentFlag();
}

/// @brief register low event code
/// @param low_event
void pushPenddingEvent(JTCX::LowLevelExternalEvents low_event) {
	JTCX::ExternalEventQueueManager::getEventManager().pushPenddingLowEvent(low_event);
}

/// @brief register low event code
/// @param mid_event
void pushPenddingEvent(JTCX::MidLevelExternalEvents mid_event) {
	JTCX::ExternalEventQueueManager::getEventManager().pushPenddingMidEvent(mid_event);
}

/// @brief register
/// @param high_event
void pushPenddingEvent(JTCX::HighLevelExternalEvents high_event) {
	JTCX::ExternalEventQueueManager::getEventManager().pushPenddingHighEvent(high_event);
}

/// @brief inform external event queue manager finished or not;
void informEventFinished(bool finished_flag) {
	JTCX::ExternalEventQueueManager::getEventManager().setEventFinishedFlag(finished_flag);
}

/// @return ControlMessage
JTCX::ControlMessage getRobotCurrentInfor() {
	return JTCX::InternalEventQueueManager::getEventManager().getRobotCurrentInfor();
}

}