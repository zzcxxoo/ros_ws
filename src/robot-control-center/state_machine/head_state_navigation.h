/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <memory>

#include "base_head_state.h"
#include "task_factory/base_task_factory.h"

using task_factory::BaseTaskFactory;

namespace state_machine {
class HeadStateNavigation : public BaseHeadState {
 public:
  void Init() override;
  void Action() override;
  void Reset() override;

  std::unique_ptr<BaseTaskFactory> nav_factory_;
};

}  // namespace state_machine
