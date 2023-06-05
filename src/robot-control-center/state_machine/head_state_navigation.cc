/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "head_state_navigation.h"

#include <ros/ros.h>

#include "proxy/data_proxy.h"
#include "state_machine.h"

using proxy::DataProxy;

namespace state_machine {
void HeadStateNavigation::Init() {
  if (is_initialized_) {
    ROS_INFO("Navigation Initialized Already");
  } else {
    ROS_INFO("Navigation Initializing Now");
    // do init
    sleep(3);
  }

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetSecondState(State::ON);
  is_initialized_ = true;
}

void HeadStateNavigation::Action() {
  ROS_INFO("Navigation Action");
  if (nav_factory_ == nullptr || nav_factory_->IsFactoryEmpty()) {
    StateMachine* fsm = StateMachine::GetInstance();
    State state;
    state.set_first(State::IDLE);
    fsm->ChangeTo(state);
  } else {
    // TODO: 决策模块
    nav_factory_->Start();
  }
}

void HeadStateNavigation::Reset() {
  is_initialized_ = false;
  // nav_factory_.reset();
  ROS_INFO("Navigation Reset");
}

}  // namespace state_machine
