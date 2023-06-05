/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <list>
#include <memory>
#include <utility>
#include <vector>

#include "base_task.h"
#include "main_tasks.h"

namespace task_factory {
typedef std::list<std::pair<int, std::shared_ptr<BaseTask>>> MainTaskList;
typedef void (*AuxFuncPtr)();
typedef std::list<std::pair<int, std::vector<AuxFuncPtr>>> AuxTaskList;

class BaseTaskFactory {
 public:
  MainTaskList& SetMainTaskList();
  AuxTaskList& SetAuxTaskList();

  virtual void Start();

  // 暂停当前任务
  virtual void Pause() {}

  // 取消所有任务
  virtual void Stop() {}

  // 删除任务

  // 增加任务

  bool IsFactoryEmpty();

 protected:
  MainTaskList main_task_list_;
  AuxTaskList aux_task_list_;
};
}  // namespace task_factory
