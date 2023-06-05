#include "JtcxLogWrapper.hpp"
#include "JtcxFileHandler.hpp"

using namespace JTCX;
using namespace spdlog;


namespace JTCX
{
    JtcxLogger::JtcxLogger(const std::string& stem_name,const std::string& log_file_name) 
    {
        if(!JtcxDir::existDir(ROBOT_LOG_DIR))
        {
            JtcxDir::createDir(ROBOT_LOG_DIR);
        }
        std::string logfile_path = std::string(ROBOT_LOG_DIR) + "/"+ log_file_name;
        
        G_logger = spdlog::rotating_logger_mt(stem_name, logfile_path , JTCX_MAX_LOG_SIZE, 1);
        G_logger->set_level(spdlog::level::debug);
        G_logger->flush_on(spdlog::level::info);
    }
    JtcxLogger::~JtcxLogger()
    {
        if(G_logger)
        {
            spdlog::drop_all();
        }
    }
}