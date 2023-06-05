/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "head_state_sleep.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateSleep::Init() {
  if (is_initialized_) {
    ROS_INFO("Sleep Initialized Already");
  } else {
    ROS_INFO("Sleep Initializing Now");
    sleep(1);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateSleep::Action() { ROS_INFO("Sleep Action"); }

void HeadStateSleep::Reset() {
  is_initialized_ = false;
  ROS_INFO("Sleep Reset");
}

}  // namespace state_machine
