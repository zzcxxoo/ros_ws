#include <spdlog/spdlog.h>
#include <boost/filesystem.hpp>
#include "spdlog/sinks/rotating_file_sink.h"

#define bf boost::filesystem

class JtcxLogWrapper
{
private:
    std::string _name;
    
public:
    std::shared_ptr<spdlog::logger> logger;
     
    explicit JtcxLogWrapper(const std::string& name) : _name(name){}
    
    void initLogFile(const std::string& dir, const std::string& file, size_t mb){
        if(!bf::exists(dir)){
            bf::create_directories(bf::path(dir));
        }
        logger = spdlog::rotating_logger_mt(_name, dir + "/" + file , mb * 1024 * 1024, 1);
    }

    ~JtcxLogWrapper(){}

};