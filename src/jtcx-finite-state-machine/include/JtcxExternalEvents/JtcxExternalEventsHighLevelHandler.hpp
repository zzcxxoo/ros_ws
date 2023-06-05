#pragma once

#include "JtcxExternalEventsBase.hpp"


class H00001:public ExternalEventsBase
{
    public:
        ~H00001(){};
        bool handleEvent() override;
};
void EXTERNAL_EVENT_REGISTER(H00001);