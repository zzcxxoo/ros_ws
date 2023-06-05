/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "base_task_factory.h"

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace task_factory {
void BaseTaskFactory::Start() {
  if (!IsFactoryEmpty()) {
    auto current_task_ptr = main_task_list_.begin()->second;
    if (!current_task_ptr->IsTaskInitialized()) {
      DataProxy* proxy = DataProxy::GetInstance();
      proxy->SetTaskType(current_task_ptr->GetTaskType());
      current_task_ptr->Init();
    }
    if (!current_task_ptr->IsTaskFinished()) {
      current_task_ptr->Execute();
      auto current_aux_task = aux_task_list_.begin();
      if (current_aux_task->first == main_task_list_.begin()->first) {
        for (auto aux : current_aux_task->second) {
          aux();
        }
        aux_task_list_.pop_front();
      }
    } else {
      main_task_list_.pop_front();
    }

    std::cout << "Current Task is "
              << State::TaskType_Name(current_task_ptr->GetTaskType())
              << ", ID is " << main_task_list_.begin()->first << ", "
              << main_task_list_.size() - 1 << " tasks left" << std::endl;
  } else {
    std::cout << "Task Factory is EMPTY" << std::endl;
  }
}

bool BaseTaskFactory::IsFactoryEmpty() {
  if (main_task_list_.size() == 0)
    return true;
  else
    return false;
}

MainTaskList& BaseTaskFactory::SetMainTaskList() { return main_task_list_; }

AuxTaskList& BaseTaskFactory::SetAuxTaskList() { return aux_task_list_; }

}  // namespace task_factory
