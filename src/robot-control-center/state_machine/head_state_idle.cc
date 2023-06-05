/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "head_state_idle.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateIdle::Init() {
  if (is_initialized_) {
    ROS_INFO("Idle Initialized Already");
  } else {
    ROS_INFO("Idle Initializing Now");
    // do init
    sleep(2);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateIdle::Action() {
  ROS_INFO("Idle Action");
}

void HeadStateIdle::Reset() {
  is_initialized_ = false;
  ROS_INFO("Idle Reset");
}
}  // namespace state_machine
