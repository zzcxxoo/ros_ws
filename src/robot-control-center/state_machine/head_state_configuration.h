/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include "base_head_state.h"

namespace state_machine {
class HeadStateConfiguraion : public BaseHeadState {
 public:
  void Init() override;
  void Action() override;
  void Reset() override;
};
}  // namespace state_machine
