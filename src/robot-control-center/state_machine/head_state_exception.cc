/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "head_state_exception.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateException::Init() {
      if (is_initialized_) {
    ROS_INFO("Exception Initialized Already");
  } else {
    ROS_INFO("Exception Initializing Now");
    // do init
    sleep(2);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateException::Action() {
      ROS_INFO("Exception Action");
}

void HeadStateException::Reset() {
      is_initialized_ = false;
  ROS_INFO("Exception Reset");
}
}  // namespace state_machine
