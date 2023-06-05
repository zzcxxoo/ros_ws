//
// Created by jtcx on 10/19/22.
//

#include "JtcxExternalHighFrequencyEventReceiver.hpp"

bool ExternalHighFrequencyEventReceiver::HandleInitialPoseSave(InitPoseSave::Request &req,
															   InitPoseSave::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleInitialPoseLoad(InitPoseLoad::Request &req,
															   InitPoseLoad::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleChargingSave(ChargingSave::Request &req, ChargingSave::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleChargingLoad(ChargingLoad::Request &req, ChargingLoad::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleVirtualLineSave(VirtualLineSave::Request &req,
															   VirtualLineSave::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleVirtualLineLoad(VirtualLineLoad::Request &req,
															   VirtualLineLoad::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandlePathLoad(PathPlanningLoad::Request &req,
														PathPlanningLoad::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandlePathSave(PathPlanningSave::Request &req,
														PathPlanningSave::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandlePathDelete(PathPlanningDelete::Request &req,
														  PathPlanningDelete::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleGetPathList(PathPlanningList::Request &req,
														   PathPlanningList::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleMappingDelete(MappingDelete::Request &req,
															 MappingDelete::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleMappingSelect(MappingSelect::Request &req,
															 MappingSelect::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleMappingEdit(MappingEdit::Request &req, MappingEdit::Response &res) {
	return true;
}
bool ExternalHighFrequencyEventReceiver::HandleGetValidMaps(MappingList::Request &req, MappingList::Response &res) {
	return true;
}
