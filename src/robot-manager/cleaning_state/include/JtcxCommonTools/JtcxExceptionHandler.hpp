/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxExceptionHandler.hpp
Version       : 固件v1.1.1rc
Author        : JianQiang 
Created       : 2022/11/27
Last Modified :
Description   : exception tools
Function List : 
History       : first edition ----20221125
******************************************************************************/
#pragma once
#include "JtcxMacroDefinition.hpp"
#include "JtcxErrorCodes.hpp"
#include "JtcxLogWrapper.hpp"


///@note using instructions
/*
	JTCX_EXCEPTION_TRY                       //在可能存在异常的代码片段前插入此宏
	...										 //运行你的代码
	...
    if(!JtcxDir::existDir("projects"))
    {
        JTCX_EXCEPTION_THROW(M10001,"文件夹 projects 不存在");//将异常描述与对应的错误码使用此宏抛出
    }
    JTCX_EXCEPTION_CATCH_BEGIN				//插入此宏用以捕捉异常并上报给监听线程
		...                                  
    	std::cout<<"已上报"<<std::endl;      //你的后处理代码

    JTCX_EXCEPTION_CATCH_END				//插入此宏上报结束
*/

namespace JTCX
{

	class JtcxExceptionCatch
	{
		public:
			/// @brief 
			/// @param code error code
			/// @param s  Information recorded in the log
			JtcxExceptionCatch(JTCX::JTCXErrorCode code, const std::string &s);
			
			/// @brief Information printed on the screen
			void operator()();
			
			~JtcxExceptionCatch();

		public:
			std::string _msg;
			JTCX::JTCXErrorCode _error_code;

		private :
			JtcxLogger* _logger = nullptr;
	};

	class JtcxExceptionHandler 
	{
		public:

			///@brief This atomic pointer can catch error exceptions in multiple threads 
			///@brief and throw them to the loop listener thread to handle exceptions
			static std::atomic<JtcxExceptionCatch *> _atomic_exception_ptr;
			
			/// @brief Exception Processor
			JtcxExceptionHandler();
			
			/// @brief This function is a built-in loop listening function and must be started in a separate thread
			void ExceptionLoopListener();

			/// Error code 1 handler
			void fun1();

			/// Error code 2 handler
			void fun2();


			~JtcxExceptionHandler();
		private :
			JtcxLogger* _logger = nullptr;
	};


	//Exception handling, which can support exception throwing between multiple threads
	#define JTCX_EXCEPTION_TRY                                                     \
    try                                                                            \
    {                                                                              \
                                                                                   
	#define JTCX_EXCEPTION_THROW(ErrorCode,ErrorMsg)                               \
    throw new JtcxExceptionCatch(ErrorCode,ErrorMsg);                              \
    }                                                                              \
                                                                                   
	#define JTCX_EXCEPTION_CATCH_BEGIN                                              \
    catch (JtcxExceptionCatch* e)                                                   \
    {                                                                               \
    /*atomic<T>*/JtcxExceptionHandler::_atomic_exception_ptr.store(e);              \
                                                                                   
	#define JTCX_EXCEPTION_CATCH_END                                                \
    }                                                                               \
	

	class JtcxCheckTools
	{
		public:
			JtcxCheckTools();

			/// @brief check battery status
			/// @return 
			static bool CHECK_BATTERY_STATUS(); 
			
			/// @brief check pursuit status
			/// @return 
			static bool CHECK_PURSUIT_STATUS();
			
			/// @brief check chassis status
			/// @return 
			static bool CHECK_CHASSIS_STATUS(); 
			
			/// @brief 
			/// @return 
			static bool CHECK_OTHER_STATUS(); 

			/// @brief catch system command standard error output redirection and judge whether it is effective
			/// @param cmd  system command
			/// @param error_infos error info record into logger
			static bool ExecuteSystemCMD(const std::string &cmd,std::string &error_info);

			~JtcxCheckTools();

		private :
			static JtcxLogger* _logger;
	};
}