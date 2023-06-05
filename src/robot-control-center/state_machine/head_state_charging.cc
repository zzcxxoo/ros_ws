/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "head_state_charging.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateCharging::Init() {
  if (is_initialized_) {
    ROS_INFO("Charging Initialized Already");
  } else {
    ROS_INFO("Charging Initializing Now");
    // do init
    sleep(2);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateCharging::Action() { ROS_INFO("Charging Action"); }

void HeadStateCharging::Reset() {
  is_initialized_ = false;
  ROS_INFO("Charging Reset");
}
}  // namespace state_machine
