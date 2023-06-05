/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "head_state_configuration.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateConfiguraion::Init() {
      if (is_initialized_) {
    ROS_INFO("Configuration Initialized Already");
  } else {
    ROS_INFO("Configuration Initializing Now");
    // do init
    sleep(2);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateConfiguraion::Action() {
      ROS_INFO("Configuration Action");
}

void HeadStateConfiguraion::Reset() {
      is_initialized_ = false;
  ROS_INFO("Configuration Reset");
}
}  // namespace state_machine
