#include "JtcxFileHandler.hpp"
#include "JtcxLogWrapper.hpp"
#include "JtcxExceptionHandler.hpp"

using namespace JTCX;
using namespace std;

//必须全局初始化
/// @brief Global instantiation of single instance mode JtcxLogger
/// @brief using JtcxSingleton<JtcxLogger>::GetInstance() to get the instance
auto jt_log = JtcxSingleton<JtcxLogger>::CreateInstance("control_centor","wjqcapf2011.log");

/// @brief Exception handling instance object, which can listen to exceptions 
/// @brief thrown by all threads and handle them orderly
auto jt_exception_handler=JtcxSingleton<JtcxExceptionHandler>::CreateInstance();


int main()
{ 

    //文件夹和文件工具使用示例1
    cout<<JtcxDir::existDir("/home/wangjq/projects")<<endl;

    //文件夹和文件工具使用示例2
    JtcxDir::createDir("/home/wangjq/projects/demo");
    JtcxDir::deleteDir("/home/wangjq/projects/demo");

    //文件夹和文件工具使用示例3
    vector<string> names;
    JtcxDir::enumCSVFiles(names,"/home/wangjq/projects/central-control-folder/data/maps/abc/zone");
    for(auto a:names)
    {
        cout<<a<<endl;
    }

    //文件夹和文件工具使用示例4
    vector<string> files;
    JtcxDir::enumFiles(files,"/home/wangjq/projects/central-control-folder/data/maps/abc/zone");
    for(auto a:files)
    {
        cout<<a<<endl;
    }

    //文件夹和文件工具使用示例5
    string stem =JtcxDir::getFileStem("/home/wangjq/projects/central-control-folder/data/maps/abc/zone/innerborder_1667289101178.csv");
    cout<<stem<<endl;

    //文件夹和文件工具使用示例6
    string suffix =JtcxDir::getFileSuffix("/home/wangjq/projects/central-control-folder/data/maps/abc/zone/innerborder_1667289101178.csv");
    cout<<suffix<<endl;

    //文件夹和文件工具使用示例7
    string father =JtcxDir::getFatherDirName("/home/wangjq/projects/central-control-folder/data/maps/abc/zone/innerborder_1667289101178.csv");
    cout<<father<<endl;

    //日志工具使用示例
    jt_log->DEBUG("2");
    jt_log->INFO("1"); 
    jt_log->WARN("{}","xiaoxiao");
    jt_log->ERROR("{}",1234);
    jt_log->CRITICAL("{},{}",12345,5678);

    //异常捕获与处理工具使用示例
    //启动异常监听与处理线程
    std::thread th1(&JtcxExceptionHandler::ExceptionLoopListener,jt_exception_handler);

    JTCX_EXCEPTION_TRY                      //在可能存在异常的代码片段前插入此宏
								
    if(!JtcxDir::existDir("projects"))       //运行你的代码
    {
        JTCX_EXCEPTION_THROW(ERROR3,"文件夹 projects 不存在");//将异常描述与对应的错误码使用此宏抛出
    }
    JTCX_EXCEPTION_CATCH_BEGIN              //插入此宏(作用：捕捉异常并上报给监听线程)

    std::cout<<"已上报监听线程"<<std::endl;    //你的后处理代码

    JTCX_EXCEPTION_CATCH_END                //插入此宏(上报结束)


    //系统命令执行并记录示例代码
    std::string error_info;//捕捉标准错误输出
    bool success = JtcxCheckTools::ExecuteSystemCMD("rosnode info /pure_pursuit 2>&1",error_info);
    if(!success) 
    {
		jt_log->ERROR("run \'rosnode info /pure_pursuit\' {}",error_info);
    }

    th1.join();
    return 1;
}
