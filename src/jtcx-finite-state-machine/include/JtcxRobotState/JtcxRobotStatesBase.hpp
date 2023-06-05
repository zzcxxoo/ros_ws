#pragma once

#include "JtcxCommon.hpp"
#include "JtcxInternalEventQueueManager.hpp"
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <memory>

class RobotStateBase
{
    public:

        RobotStateBase(){
            // 1. construct ros thread for ros related function
            _nh.setCallbackQueue(&_ros_queue);
            _ros_thread.reset(new std::thread(RobotStateBase::rosHandler, this));
        }

        virtual ~RobotStateBase(){
            _nh.shutdown();
        }

        /// @brief 
        /// @param abnormalType 
        void pushAbnormalState(JTCX::RobotAbnormalType abnormalType)
        {
            JTCX::InternalEventQueueManager::getEventManager().pushAbnormalState(abnormalType);
        }

        /**
         * @brief  thread handler: checking health state in a separate thread 
         * 
         */
        virtual void checkHealthState() = 0;

        /**
         * @brief ros handler, feel free to modify it! but usually no need to!!
         * 
         * @param rsb 
         */
        void rosHandler() {
            ros::Rate r(5);
            while(_nh.ok()){
                _ros_queue.callAvailable();
                r.sleep();
            }
        }

    protected:
        std::unique_ptr<std::thread> _ros_thread;  ///< ros thread execute ros.spin() in case that block main thread        
        std::unique_ptr<std::thread> _check_health_state_thread_ptr;    ///< pointer of thread with handler named checkHealthState
        ros::NodeHandle _nh;   ///< ros public handler
        ros::NodeHandle _pnh;  ///< ros private handler

    private:
        ros::CallbackQueue _ros_queue;  ///< cb registered in _nh should be push in the queue

    private:
        RobotStateBase(const RobotStateBase& ){};
        RobotStateBase& operator=(const RobotStateBase& )
        {return *this;}

};