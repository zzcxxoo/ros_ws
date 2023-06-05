/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxMacroDefinition.hpp
Version       : 固件v1.1.1rc
Author        : Jianqiang 
Created       : 2022/11/22
Last Modified :
Description   : Necessary macro definitions
Function List :
History       : first edition ----20221125
******************************************************************************/
#pragma once

#define JTCX_MAX_LOG_SIZE (10485760)   // 10 * 1024 * 1024 bytes
#define JTCX_MAX_RECORD_TRACK_NUM (100) //最大录制轨迹数量100条
#define JTCX_CAR_WIDTH (1.2) //车辆宽度
#define JTCX_PATHPOINT_GAP_DIST (0.3) //轨迹点间距

#define ROBOT_VERSION "1.3.0"

///DIR
#define ROBOT_HOME_DIR                           "/home/jtcx"
#define ROBOT_CENTRAL_CONTROL_ROOT_DIR           "/home/jtcx/.robot/"
#define ROBOT_LOG_DIR                            "/var/log/jtcx/robot-manager"   //日志路径
#define ROBOT_MAP_DIR                            "/home/jtcx/.robot/data/maps"  //地图路径
#define WEB_CURRENT_REAL_TIME_SAVED_MAP_DIR      "/home/jtcx/.robot/data/maps/current"

///web app related
#define WEB_SELECTED_MAP_YAML_PATH               "/home/jtcx/.robot/config/selected_map.yaml"//选择地图后的地图名，web刷新后的导航页面加载的地图
#define WEB_MAPFILE_UPDATE_PATH                  "/home/jtcx/.robot/config/mapfile_update" //periodic_save_map.sh有使用
///cleaning task related
#define CLEANING_PATH                            "/home/jtcx/.robot/data/cleaning-" ROBOT_VERSION
#define CLEANING_CONTEXT_JSON_PATH               CLEANING_PATH "/context.json" //清洁记录/上下文
#define CLEANING_TASKS_JSON_PATH                 CLEANING_PATH "/tasks.json" //清洁任务列表
#define CLEANING_TIMING_CONTEXT_JSON_PATH        CLEANING_PATH "/timing_tasks.json" //清洁定时任务
///params related
#define PARAMS_AMCL_YAML_PATH                    "/home/jtcx/.robot/config/amcl_params.yaml"
///OSS upload
// #define MAP_ID_2_NAME_JSON　　　　                "layer.json"//语义层标注文件
// #define COVERAGE_PATH_NAME                       "coverage.csv"//覆盖路径
// #define EDGE_CSV_NAME                            "edge.csv"//沿边路径 
// #define FROM_CHARGING_PATH                       "from_charging.csv"//默认出航路径

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <map>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <assert.h>
#include <atomic>
#include <stdio.h>
#include <cassert>
#include <cstdarg>
#include <fstream>
#include <sys/time.h>  
#include <unistd.h>
#include <ratio>   
#include <ctime>
#include <algorithm>
// #include <boost/preprocessor.hpp>

// #define JTCX_SWITCH_THREAD_PRE                                             \
//     std::function<void(void)> *pFunc = new std::function<void(void)>;      \
//     *pFunc = [&](void) {                                                   \
// #define JTCX_SWITCH_THREAD_ING                                             \
//         }                                                                  \           

// 打印到控制台还是文件
// #define LOG_CONSOLE
#define JtcxStrnCpy strncpy
#define JtcxStrCmp strcmp
#include <string.h>

#define JTCX_DEPRECATED __attribute__((deprecated))
#define JTCX_EXP_API __attribute__((visibility("default")))

#define JTCX_BEGIN_DECLS extern "C" {
#define JTCX_END_DECLS }

/// @brief Using an in class function as a protection function, 
/// @brief please call this macro in the in class constructor.
/// @note fun_ptr: 一个无返回值的函数地址 void fun_ptr(int sig);
#include <signal.h>
#define PROTECTION_BRFORE_CRASH(fun_ptr)                                   \
signal(SIGINT,  fun_ptr);                                                  \
signal(SIGTERM, fun_ptr);                                                  \
signal(SIGBUS,  fun_ptr);                                                  \
signal(SIGFPE,  fun_ptr);                                                  \
signal(SIGTSTP, fun_ptr);                                                  \

// #define I_LOG_T(...)  SPDLOG_LOGGER_CALL(G_logger, spdlog::level::trace, __VA_ARGS__); 
// #define I_LOG_D(...)  SPDLOG_LOGGER_CALL(G_logger, spdlog::level::debug, __VA_ARGS__); 
// #define I_LOG_I(...)  SPDLOG_LOGGER_CALL(G_logger, spdlog::level::info, __VA_ARGS__); 
// #define I_LOG_W(...)  SPDLOG_LOGGER_CALL(G_logger, spdlog::level::warn, __VA_ARGS__); 
// #define I_LOG_E(...)  SPDLOG_LOGGER_CALL(G_logger, spdlog::level::err, __VA_ARGS__); 
// #define I_LOG_C(...)  SPDLOG_LOGGER_CALL(G_logger, spdlog::level::critical, __VA_ARGS__);


// #define declearSingleRawMember(MemberType)                                                         \
//     MemberType m##MemberType = nullptr;                                                            \
//     std::recursive_mutex m##MemberType##Mutex;                                                     \
//                                                                                                    \
//     void set##MemberType(MemberType member) {                                                      \
//         std::lock_guard<std::recursive_mutex> lock(m##MemberType##Mutex);                          \
//         m##MemberType = member;                                                                    \
//     }                                                                                              \
//                                                                                                    \
//     MemberType get##MemberType() {                                                                 \
//         std::lock_guard<std::recursive_mutex> lock(m##MemberType##Mutex);                          \
//         return m##MemberType;                                                                      \
//     }                                                                                              \


/// @brief Single instance mode of template class
template <typename T> 
class JtcxSingleton {
  public:
    template <typename... Args> 
    static T *CreateInstance(Args &&... args) {
        if (m_pInstance == nullptr) {
            m_pInstance = new T(std::forward<Args>(args)...);
        }
        return m_pInstance;
    }

    static T *GetInstance() { return m_pInstance; }

    static void DestroyInstance() {
        if (m_pInstance) {
            delete m_pInstance;
            m_pInstance = nullptr;
        }
    }

  private:
    JtcxSingleton(void) = default;
    virtual ~JtcxSingleton(void) = default;

  private:
    static T *m_pInstance;
};

template <class T> 
T *JtcxSingleton<T>::m_pInstance = nullptr;
