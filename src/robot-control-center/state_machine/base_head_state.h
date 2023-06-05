/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <iostream>

namespace state_machine {
class BaseHeadState {
 public:
  virtual bool IsInitialized() {
    return is_initialized_;
  }
  virtual void Init() = 0;
  virtual void Action() = 0;
  virtual void Reset() = 0;

 protected:
  bool is_initialized_ = false;
};
}  // namespace state_machine
