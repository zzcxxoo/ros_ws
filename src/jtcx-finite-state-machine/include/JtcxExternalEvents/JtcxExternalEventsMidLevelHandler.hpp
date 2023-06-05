#pragma once

#include "JtcxExternalEventsBase.hpp"


class M00001:public ExternalEventsBase
{
    public:
        ~M00001(){};
    bool handleEvent();
};
void EXTERNAL_EVENT_REGISTER(M00001);