/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "head_state_update.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateUpdate::Init() {
     if (is_initialized_) {
    ROS_INFO("Update Initialized Already");
  } else {
    ROS_INFO("Update Initializing Now");
    // do init
    sleep(2);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateUpdate::Action() {
      ROS_INFO("Update Action");
}

void HeadStateUpdate::Reset() {
      is_initialized_ = false;
  ROS_INFO("Update Reset");
}
}  // namespace state_machine
