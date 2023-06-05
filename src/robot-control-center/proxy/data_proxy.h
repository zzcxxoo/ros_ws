/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include "robot-control-center/proto/state.pb.h"

using state_machine::data_type::State;

namespace proxy {
class DataProxy {
 public:
  static DataProxy* GetInstance() {
    static DataProxy singleton;
    return &singleton;
  }

  void UpdatePreviousState() {
    previous_state_.set_first(current_state_.first());
    previous_state_.set_second(current_state_.second());
    previous_state_.set_task(current_state_.task());
  }
  void SetFirstState(const State::HeadStateType& type) {
    current_state_.set_first(type);
  }
  void SetSecondState(const State::SecondStateType& type) {
    current_state_.set_second(type);
  }
  void SetTaskType(const State::TaskType& type) {
    current_state_.set_task(type);
  }
  const State& GetCurrentState() { return current_state_; }

 private:
  State current_state_;
  State previous_state_;
};
}  // namespace proxy
