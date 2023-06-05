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
            system("sudo mkdir -p " ROBOT_LOG_DIR);
        }
        std::string logfile_path = std::string(ROBOT_LOG_DIR) + "/"+ log_file_name;
        if(!JtcxDir::existDir(logfile_path)){
            
            std::string cmd = fmt::format("sudo touch {0} && sudo chmod 666 {0}", logfile_path);
            system(cmd.c_str());
        }

    #ifdef LOG_CONSOLE 
        G_logger = spdlog::default_logger();
    #else
        G_logger = spdlog::rotating_logger_mt(stem_name, logfile_path , JTCX_MAX_LOG_SIZE, 1);
    #endif
        G_logger->set_level(spdlog::level::trace);
        G_logger->flush_on(spdlog::level::trace);
    }
    JtcxLogger::~JtcxLogger()
    {
        if(G_logger)
        {
            spdlog::drop_all();
        }
    }
}