//
// Created by jtcx on 10/19/22.
//
#pragma once

#include "JtcxExternalLowFrequencyEventReceiver.hpp"

/*** construct a singleton schema object  ***/
std::unique_ptr<ExternalLowFrequencyEventReceiver>
		ExternalLowFrequencyEventReceiver::_pExternalLowFrequencyEventReceiver
		(new ExternalLowFrequencyEventReceiver());

ExternalLowFrequencyEventReceiver::ExternalLowFrequencyEventReceiver() {
	std::make_unique<std::thread>(ExternalLowFrequencyEventReceiver::RosHandler);
	_ui_start_mapping =
			_nh.advertiseService("ui/mapping/start", &ExternalLowFrequencyEventReceiver::HandleMappingStart, this);
	_ui_stop_mapping =
			_nh.advertiseService("ui/mapping/stop", &ExternalLowFrequencyEventReceiver::HandleMappingStop, this);
}

bool ExternalLowFrequencyEventReceiver::HandleMappingStart(SystemdService::Request &req,
														   SystemdService::Response &res) {
	_start_mapping_req = req;

	JTCX::ExternalEventQueueManager::getEventManager().pushPenddingHighEvent(JTCX::HighLevelExternalEvents::H0001);

	auto future = _is_mapping_start_completed.get_future();

	while (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
		/// return overtime situation

		JTCX::ExternalEventQueueManager::getEventManager().setEventFinishedFlag(true);

		res.message = "handle mapping start function overtime!";

		res.status = -1;

		return true;
	}

	JTCX::ExternalEventQueueManager::getEventManager().setEventFinishedFlag(true);

	res = _start_mapping_res;

	return true;
}

bool ExternalLowFrequencyEventReceiver::HandleMappingStop(SystemdService::Request &req,
														  SystemdService::Response &res) {

	_stop_mapping_req = req;

	JTCX::ExternalEventQueueManager::getEventManager().pushPenddingHighEvent(JTCX::HighLevelExternalEvents::H0002);

	auto future = _is_mapping_stop_completed.get_future();

	while (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
		/// return overtime situation

		JTCX::ExternalEventQueueManager::getEventManager().setEventFinishedFlag(true);

		res.message = "handle mapping stop function overtime!";

		res.status = -1;

		return true;
	}

	JTCX::ExternalEventQueueManager::getEventManager().setEventFinishedFlag(true);

	const SystemdService::Response &response = res;

	ExternalLowFrequencyEventReceiver::getinstance()->SetStopMappingRes(res);

	ExternalLowFrequencyEventReceiver::getinstance()->GetIsMappingSaveCompleted().set_value(true);

	res = _stop_mapping_res;

	return true;

}

bool ExternalLowFrequencyEventReceiver::HandleMappingExtend() {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleMappingSave(MappingSave::Request &req, MappingSave::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleStartNavigation(SystemdService::Request &req,
															  SystemdService::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleStopNavigation(SystemdService::Request &req,
															 SystemdService::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleRelocalize() {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleStartPathrecord(SystemdService::Request &req,
															  SystemdService::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandlePathPlanningStartPursuit(PatrolStart::Request &req,
																	   PatrolStart::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandlePathPlanningStopPursuit(PatrolStop::Request &req,
																	  PatrolStop::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleConvexPloygonGenerate(ConvexPolygon::Request &req,
																	ConvexPolygon::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandlePassPointPlanner(PassingPoint::Request &req,
															   PassingPoint::Response &res) {
	return true;
}
bool ExternalLowFrequencyEventReceiver::HandleTargetPointPlanner(TargetPoint::Request &req,
																 PassingPoint::Response &res) {
	return true;
}
