#pragma once


class ExternalEventsBase
{
    public:

        virtual ~ExternalEventsBase(){};
        
        /// @brief Handle parameterless events
        virtual bool handleEvent();
};