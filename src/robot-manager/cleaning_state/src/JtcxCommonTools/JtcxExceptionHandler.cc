/******************************************************************************
 @copyright Copyright <JT-Innovation> (c) 2022
******************************************************************************
File Name     : JtcxExceptionHandler.cc
Version       : 固件v1.1.1rc
Author        : JianQiang 
Created       : 2022/11/28
Last Modified :
Description   : exception tools
Function List : 
History       : first edition ----20221129
******************************************************************************/
#include "JtcxExceptionHandler.hpp"

namespace JTCX
{
		
		JtcxExceptionCatch::JtcxExceptionCatch(JTCX::JTCXErrorCode code, const std::string &s)
			: _error_code(code)
			, _msg(s)
		{
			if(code != NO_ERROR)
			{
				_logger=JtcxSingleton<JtcxLogger>::GetInstance();
				auto text = "Error code:" + std::to_string(_error_code) + ". Error message:" + _msg;
				_logger->ERROR("{}",text);
			}
		}

		void JtcxExceptionCatch::operator()()
		{
			auto text = "Error code:" + std::to_string(_error_code) + "\r\nError message:" + _msg;
			std::cout << text << std::endl;
		}

		JtcxExceptionCatch::~JtcxExceptionCatch()
		{
			if(_logger)
			{
				delete _logger;
				_logger=nullptr;
			}
		}

		std::atomic<JtcxExceptionCatch *> JtcxExceptionHandler::_atomic_exception_ptr(new JtcxExceptionCatch(NO_ERROR,"normal"));

		JtcxExceptionHandler::JtcxExceptionHandler()
		{
			_logger=JtcxSingleton<JtcxLogger>::GetInstance();
		}
		JtcxExceptionHandler::~JtcxExceptionHandler()
		{
			if(_logger)
			{
				delete _logger;
				_logger=nullptr;
			}
		}
		void JtcxExceptionHandler::ExceptionLoopListener()
		{
			bool battery_error_handled=false;
			bool pursuit_error_handled=false;
			bool chassis_error_handled=false;

			while(1){
				try{
					//exception come from other modules
					JtcxExceptionCatch* temp=_atomic_exception_ptr.load();
					if (temp->_error_code != NO_ERROR) {
						throw(temp);
					}
					// exception come from JtcxCheckTools
					// The judgment order cannot be wrong,or the logger is easy to overflow
					if (!battery_error_handled && !JtcxCheckTools::CHECK_BATTERY_STATUS())
					{
						temp->_error_code = BATTERY_ERROR;
						temp->_msg  = "电池电量不足！";
						_logger->ERROR("{}",temp->_msg);
						throw(temp);
					}
					if(!pursuit_error_handled && !JtcxCheckTools::CHECK_PURSUIT_STATUS())
					{
						temp->_error_code = PURSUIT_ERROR;
						temp->_msg  = "跟踪器运行状态错误！";
						_logger->ERROR("{}",temp->_msg);
						throw(temp);
					}

					if(!chassis_error_handled && !JtcxCheckTools::CHECK_CHASSIS_STATUS())
					{
						temp->_error_code = CHASSIS_ERROR;
						temp->_msg  = "传感器故障！";
						_logger->ERROR("{}",temp->_msg);
						throw(temp);
					}
				}

				catch (JtcxExceptionCatch* e){
					switch(e->_error_code){
						case ERROR3:{
							fun1();
							break;
						}
						case ERROR4:{
							fun2();
							break;
						}
						case BATTERY_ERROR:{
							/*handler*/;
							battery_error_handled=true;break;
						}
						case PURSUIT_ERROR:{
							/*handler*/;
							pursuit_error_handled=true;break;
						}
						case CHASSIS_ERROR:{
							/*handler*/;
							chassis_error_handled=true;break;
						}
					}
					_logger->INFO("{} handled successfully",e->_msg);
					_atomic_exception_ptr.store(new JtcxExceptionCatch(NO_ERROR,"normal"));//务必重置!!!!
				}
			}
		}

		void JtcxExceptionHandler::fun1()
		{std::cout<<"Error code:ERROR3 is handling"<<std::endl;}

		void JtcxExceptionHandler::fun2()
		{std::cout<<"error code:ERROR4 is handling"<<std::endl;}


		JtcxLogger* JtcxCheckTools::_logger = JtcxSingleton<JtcxLogger>::GetInstance();

		JtcxCheckTools::JtcxCheckTools(){}

		JtcxCheckTools::~JtcxCheckTools(){}

		bool JtcxCheckTools::ExecuteSystemCMD(const std::string &cmd,std::string &error_info)
		{ 

			FILE *fp;
			const int sizebuf = 2048;
			char buff[sizebuf];
			fp = popen(cmd.c_str(), "r");
			std::vector<std::string> out;
			if (!fp)    
				_logger->ERROR("Couldn't start command{}",cmd);

			while (fgets(buff, sizeof(buff), fp)) {
				std::string cur_string = "";
				cur_string += buff;
				out.push_back(cur_string.substr(0, cur_string.size()));
			}
			pclose(fp);

			auto it = std::find_if(out.begin(), out.end(), [](const std::string& e){
            return (e.find("ERROR") != e.npos) || (e.find("cannot") != e.npos);});
   
            if(it != out.end())
            {   
			   error_info = *it;
			   return false;
			}
			return true;
		}

		bool JtcxCheckTools::CHECK_BATTERY_STATUS()
		{
			//ros
			return false;
		} 
		bool JtcxCheckTools::CHECK_PURSUIT_STATUS()
		{
			std::string error_info;
            bool success = ExecuteSystemCMD("rosnode info /pure_pursuit 2>&1",error_info);
			if(!success) 
			{
				// std::cout<<out.size()<<std::endl;
				_logger->ERROR("run \'rosnode info /pure_pursuit\' {}",error_info);
				// _logger->ERROR(out[1]);
				return false;
			}
			return true;

			//后处理
			// PurePursuitResult ppr;
			// ppr.tracking_result = cleaningException::PP_EXIT;
			// cleaningResultHandler(boost::make_shared<PurePursuitResult>(ppr));
		
		}
		bool JtcxCheckTools::CHECK_CHASSIS_STATUS()
		{
			//ros
			return false;
		}
}