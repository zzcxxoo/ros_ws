/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxLogWrapper.hpp
Version       : 固件v1.1.1rc
Author        : Guanyu/Jianqiang 
Created       : 2022/11/26
Last Modified :
Description   : log class
Function List : 
History       : first edition ----20221128
******************************************************************************/
#pragma once
#include <spdlog/spdlog.h>
#include "spdlog/sinks/rotating_file_sink.h"
#include "JtcxMacroDefinition.hpp"

namespace JTCX
{
    class JtcxLogger
    {
        public:
            JtcxLogger(const std::string& stem_name,const std::string& log_file_name);

            /// @brief trace
            template<typename ...ARGS>
            void TRACE(ARGS ...args)
            {
                G_logger->trace(args ...);
            }

            /// @brief debug
            template<typename ...ARGS>
            void DEBUG(ARGS ...args)
            {
                G_logger->debug(args ...);
            }
            
            /// @brief info
            template<typename ...ARGS>
            void INFO(ARGS ...args)
            {
                G_logger->info(args ...);
            }

            /// @brief warn
            template<typename ...ARGS>
            void WARN(ARGS ...args)
            {
                G_logger->warn(args ...);  
            }

            /// @brief error
            template<typename ...ARGS>
            void ERROR(ARGS ...args)
            {
                G_logger->error(args ...); 
            }

            /// @brief critical
            template<typename ...ARGS>
            void CRITICAL(ARGS ...args)
            {
                G_logger->critical(args ...);
            }

            ~JtcxLogger();

        private:
            std::shared_ptr<spdlog::logger> G_logger;
    };
}