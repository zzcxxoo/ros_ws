#pragma once

#include "JtcxRobotStatesBase.hpp"

/*** singleton for mapping external interface  ***/

class ExternalMappingInterface {

 public:

  ExternalMappingInterface &getinstance() {
	ExternalMappingInterface e;
	return e;
  }

 private:

};

class RobotMappingState : public RobotStateBase {
 public:
  ~RobotMappingState();

};
