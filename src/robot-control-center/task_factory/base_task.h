/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include "robot-control-center/proto/state.pb.h"

using state_machine::data_type::State;

namespace task_factory {
class BaseTask {
 public:
  virtual void Init() { is_task_initialized = true; }
  virtual bool IsTaskFinished() { return is_task_finished_; }
  virtual bool IsTaskInitialized() { return is_task_initialized; }
  virtual void Execute() = 0;

  const State::TaskType& GetTaskType() { return task_type_; }

  BaseTask() { std::cout << "base task constructor called" << std::endl; }
  ~BaseTask() { std::cout << "base task destructor called" << std::endl; }

 protected:
  bool is_execution_done_ = false;
  bool is_task_finished_ = false;
  bool is_task_initialized = false;
  State::TaskType task_type_ = State::NONE;
};
}  // namespace task_factory
