#pragma once

#include "JtcxExternalEventsBase.hpp"


class L00001:public ExternalEventsBase
{
    public:
        ~L00001(){};
    bool handleEvent();
};
void EXTERNAL_EVENT_REGISTER(L00001);
