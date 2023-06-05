/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <ros/forwards.h>

#include <memory>

#include "head_state_charging.h"
#include "head_state_configuration.h"
#include "head_state_exception.h"
#include "head_state_idle.h"
#include "head_state_navigation.h"
#include "head_state_sleep.h"
#include "head_state_update.h"
#include "robot-control-center/proto/state.pb.h"

using state_machine::data_type::State;

namespace state_machine {

class StateMachine {
 public:
  static StateMachine* GetInstance() {
    static StateMachine singleton;
    return &singleton;
  }

  void Init();
  void Update(const ros::TimerEvent&);
  bool ChangeTo(const State&);
  void ConstructNavigationFactory(std::unique_ptr<BaseTaskFactory>&);

 private:
  BaseHeadState** current_state_ptr_;

  BaseHeadState* idle_state_;
  BaseHeadState* sleep_state_;
  BaseHeadState* navigation_state_;
  BaseHeadState* configuration_state_;
  BaseHeadState* charging_state_;
  BaseHeadState* update_state_;
  BaseHeadState* exception_state_;
};

}  // namespace state_machine
