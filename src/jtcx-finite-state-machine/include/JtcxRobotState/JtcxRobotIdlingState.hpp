#pragma once

#include "JtcxRobotStatesBase.hpp"

class RobotIdlingState:public RobotStateBase
{
    public:
        RobotIdlingState();
        ~RobotIdlingState();

        virtual void checkHealthState() override;

};