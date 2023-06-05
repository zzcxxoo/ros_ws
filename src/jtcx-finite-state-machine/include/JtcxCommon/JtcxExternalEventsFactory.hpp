#pragma once

/******************************************************************************
 Copyright (C), 2022-2032, JTCX Inc.
******************************************************************************
File Name     : ExternalEventsFactory
Version       : v1.0.0
Author        : wang jian qiang 
Created       : 2022/10/15
Last Modified :
Description   : Factory
Function List :
History       : 20221015_v1.0.0
******************************************************************************/

#include "JtcxCommon.hpp"
#include "JtcxExternalEventsBase.hpp"

class ExternalEventsBase;

/// @brief register macro definition
#define EXTERNAL_EVENT_REGISTER(eventCode)                   \
        ExternalEventsBase* createEvent##eventCode()    \
        { return new eventCode;}                              \
        EventReflector reflector##eventCode(#eventCode,createEvent##eventCode)\

/// @brief event constructor
typedef ExternalEventsBase* (*EventConstructor)();

class ExternalEventsFactory
{

        public:
            ~ExternalEventsFactory();

            static ExternalEventsFactory& getInstance();

            /// @brief register
            /// @param eventCode  event code
            /// @param objc  event code handler
            void Register(std::string eventCode, EventConstructor objc);

            /// @brief get event handler
            /// @param eventCode  event code 
            /// @return 
            ExternalEventsBase* getEventHandler(std::string eventCode);

            /// @brief low enum to class
            std::map<int,std::string> G_enum_class_reflector_low_events;

            /// @brief mid enum to class
            std::map<int,std::string> G_enum_class_reflector_mid_events;
            
            /// @brief high enum to class
            std::map<int,std::string> G_enum_class_reflector_high_events;

        private:

            std::map<std::string,EventConstructor> _EventHandlers;

            ExternalEventsFactory();
            ExternalEventsFactory(const ExternalEventsFactory& );
            ExternalEventsFactory& operator=(const ExternalEventsFactory& )
            {return *this;}

};

ExternalEventsFactory& ExternalEventsFactory::getInstance()
{
    static ExternalEventsFactory factory;
    return factory;
}

void ExternalEventsFactory::Register(std::string eventCode, EventConstructor objc)
{
    if(objc)
    {
        _EventHandlers.insert(std::make_pair(eventCode,objc));
    }

}

ExternalEventsBase* ExternalEventsFactory::getEventHandler(std::string eventCode)
{
    std::map<std::string,EventConstructor> ::const_iterator iter=_EventHandlers.find(eventCode);

    if(iter !=_EventHandlers.end())
    {
        return iter->second();
    }
}

class EventReflector
{
    public:
    EventReflector(std::string eventCode,EventConstructor objc)
    {
        ExternalEventsFactory::getInstance().Register(eventCode,objc);
        
    }
    virtual ~EventReflector(){}
};