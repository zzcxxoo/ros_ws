#pragma once

#include <spdlog/spdlog.h>

#include <boost/filesystem.hpp>
#include <ctime>

#include "spdlog/fmt/bundled/ranges.h"
#include "spdlog/sinks/rotating_file_sink.h"

#define bfs boost::filesystem

enum class LOG_LEVEL { DEPLOY, TEST, DEBUG };

#define LOG_WITH_GAP(log, level, gap, msg) \
  \   
static double __lt = std::time(nullptr);   \
  double __tmp = std::time(nullptr);       \
  if (__tmp - __lt > gap) {                \
    __lt = __tmp;                          \
    log->level("{}", msg);                 \
  }

class JtcxLogWrapper {
 private:
  std::string _name;
  LOG_LEVEL _level;

 public:
  std::shared_ptr<spdlog::logger> logger;

  explicit JtcxLogWrapper(const std::string& name,
                          LOG_LEVEL l = LOG_LEVEL::DEPLOY)
      : _name(name), _level(l) {
    logger = spdlog::default_logger();
    initLog();
  }

  void initLog() { setLogLevel(_level); }

  void initLogFile(const std::string& dir, const std::string& file,
                   size_t mb = 20) {
    if (!bfs::exists(dir)) {
      std::string cmd = "sudo mkdir -p " + dir;
      system(cmd.c_str());
    }
    auto lf = dir + "/" + file;
    if (!bfs::exists(lf)) {
      std::string cmd = "sudo touch " + lf;
      system(cmd.c_str());
      cmd = "sudo chmod 666 " + lf;
      system(cmd.c_str());
    }

    logger = spdlog::rotating_logger_mt(_name, lf, mb * 1024 * 1024, 1);
    initLog();
  }

  void setLogLevel(LOG_LEVEL l) {
    switch (l) {
      case LOG_LEVEL::DEBUG: {
        logger->set_level(spdlog::level::trace);
        logger->flush_on(spdlog::level::trace);
        break;
      }
      case LOG_LEVEL::TEST: {
        logger->set_level(spdlog::level::info);
        logger->flush_on(spdlog::level::info);
        break;
      }
      default: {
        logger->flush_on(spdlog::level::warn);
        break;
      }
    }
  }

  ~JtcxLogWrapper() { spdlog::drop(_name); }
};