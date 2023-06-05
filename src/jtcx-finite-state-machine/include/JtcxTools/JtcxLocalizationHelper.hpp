#include "JtcxUtils.hpp"
#include "JtcxInternalEventQueueManager.hpp"
#include <ros/ros.h>
#include <functional>

#include "mobile_platform_msgs/ScanMatchingStatus.h"

/**
 * @brief localiztion related helper class 
 * 
 */
class JtcxLocalizationHelper
{

public:
    // localization algorithm error code
    enum class ErrorCode{
        NODE_DIED,
        LOST,
        LOSTING,
        NORMAL
    };

private:
    // ros related 
    ros::NodeHandle* _nh_ptr;
    ros::Subscriber _loc_sub;
    ros::WallTimer _localization_node_timer;    
    
    ErrorCode _state{ErrorCode::NODE_DIED};     ///< localization state

    const float _loc_threshold{0.7}; ///< localization threshold
    const float _lost_time_threshold{1.0};
    ros::WallTime _lost_start_timestamp;
    
public:

    /**
     * @brief Construct a new Jtcx Localization Helper object
     * 
     * @param nh 
     */
    explicit JtcxLocalizationHelper(ros::NodeHandle* nh) : _nh_ptr(nh)
    {
        _localization_node_timer = _nh_ptr->createWallTimer(ros::WallDuration(1.0), &JtcxLocalizationHelper::checkNodeAliveHandler, this);
        _loc_sub = _nh_ptr->subscribe("/status", 1, &JtcxLocalizationHelper::localizationInfoHandler, this);

    }

    /**
     * @brief Destroy the Jtcx Localization Helper object
     * 
     */
    ~JtcxLocalizationHelper();

    /**
     * @brief 
     * 
     * @param msg : rule to check one specific msg is a lost info or not 
     * @return true 
     * @return false 
     */
    bool lostOrNot(const mobile_platform_msgs::ScanMatchingStatus& msg){
        return !(msg.has_converged && (msg.inlier_fraction > _loc_threshold));
    }
    
    /**
     * @brief : check wether localization is lost exactly 
     * 
     * @param msg 
     */
    void localizationInfoHandler(const mobile_platform_msgs::ScanMatchingStatusConstPtr& msg)
    {
        // 1. if normal and lost at first time --> losting
        // 2. if losting
        // 2.1 if loc good --> normal, reset
        // 2.2 if loc bad --> stay
        // 2.3 if loc bad to threshold time --> lost
        // 3. if lost and get good --> normal
        // 4. if node died 

        if(!lostOrNot(*msg)){
            switchState(ErrorCode::NORMAL);
            return;
        }

        if(_state != ErrorCode::LOSTING && lostOrNot(*msg))
        {
            switchState(ErrorCode::LOSTING);
            _lost_start_timestamp = ros::WallTime::now();
            return;
        }

        if(_state == ErrorCode::LOSTING){
            if((ros::WallTime::now() - _lost_start_timestamp) > ros::WallDuration(_lost_time_threshold)){
                switchState(ErrorCode::LOST);
            }
        }
    }

    /**
     * @brief 
     * 
     * @param e 
     */
    void checkNodeAliveHandler(const ros::WallTimerEvent& e)
    {
        if(!JtcxUtils::checkNodeAlive("/hdl_localization_node"))
        {
            switchState(ErrorCode::NODE_DIED);
        }
    }

    /**
     * @brief Get the Current State object
     * 
     */
    ErrorCode getCurrentState(){return _state;}

    /**
     * @brief a wrapper for switching state and deal with exception
     *          localization related stuff no need to be worried when the object is included in a class
     * @param s : state to switch
     */
    void switchState(ErrorCode s)
    {
        _state = s;
        if(s == ErrorCode::LOST || s == ErrorCode::NODE_DIED){
            JTCX::InternalEventQueueManager::getEventManager().pushAbnormalState(JTCX::RobotAbnormalType::emergency);
        }
    }
};
