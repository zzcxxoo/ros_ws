#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <iostream>
#include <memory>

#include "auxiliay_tasks.h"
#include "base_task_factory.h"
#include "main_tasks.h"

using namespace task_factory;
using namespace std;

class StateMachine {
 public:
  bool ConstructFactory(unique_ptr<BaseTaskFactory>& fac) {
    factory_ = move(fac);
  }
  unique_ptr<BaseTaskFactory> factory_;
};

class NavigationFactory : public BaseTaskFactory {};

int main(int argc, char* argv[]) {
  std::cout << "task factory test_fsm" << std::endl;
  ros::init(argc, argv, "task_factory_test_fsm");
  ros::NodeHandle n;
  StateMachine test_fsm;

  // 收到event（外部或内部），创建任务工厂，放入状态机，等待执行
  {
    std::unique_ptr<BaseTaskFactory> test_factory(new BaseTaskFactory());
    vector<AuxFuncPtr> aux_task;
    // add 1st main task
    test_factory->SetMainTaskList().emplace_back(
        make_pair(1, make_shared<CurbFollowTask>()));
    // add 2nd main task and aux task
    test_factory->SetMainTaskList().emplace_back(
        make_pair(5, make_shared<CoveragePlanningTask>()));
    aux_task.emplace_back(&MainBrush);
    aux_task.emplace_back(&SideBrush);
    aux_task.emplace_back(&Fan);
    test_factory->SetAuxTaskList().emplace_back(make_pair(5, aux_task));
    aux_task.clear();
    // add 3rd main task and aux task
    test_factory->SetMainTaskList().emplace_back(
        make_pair(8, make_shared<AvoidingTask>()));
    aux_task.emplace_back(&Loudspeaker);
    test_factory->SetAuxTaskList().emplace_back(make_pair(8, aux_task));
    // 任务工厂创建好后，放入状态机中
    test_fsm.ConstructFactory(test_factory);
  }
  // 打印工厂内的主任务清单和辅助任务清单
  int num = test_fsm.factory_->SetMainTaskList().size();
  ROS_WARN("******* %d TASKS IN THE FACTORY *********", num);
  for (MainTaskList::iterator it = test_fsm.factory_->SetMainTaskList().begin();
       it != test_fsm.factory_->SetMainTaskList().end(); ++it) {
    cout << "Task ID " << it->first << ": "
         << State::TaskType_Name(it->second->GetTaskType()) << endl;
    for (AuxTaskList::iterator ait =
             test_fsm.factory_->SetAuxTaskList().begin();
         ait != test_fsm.factory_->SetAuxTaskList().end(); ++ait) {
      if (ait->first == it->first) {
        for (auto t : ait->second) {
          t();
        }
        break;
      }
    }
  }

  // 收到event（外部或内部），创建任务工厂，放入状态机，等待执行
  {
    std::unique_ptr<BaseTaskFactory> new_factory(new NavigationFactory());
    vector<AuxFuncPtr> aux_task;
    // add 1st main task and aux task
    new_factory->SetMainTaskList().emplace_back(
        make_pair(10, make_shared<CoveragePlanningTask>()));
    aux_task.emplace_back(&Loudspeaker);
    aux_task.emplace_back(&MainBrush);
    aux_task.emplace_back(&Fan);
    new_factory->SetAuxTaskList().emplace_back(make_pair(10, aux_task));
    aux_task.clear();
    // add 2nd main task
    new_factory->SetMainTaskList().emplace_back(
        make_pair(20, make_shared<AvoidingTask>()));
    // add 3rd main task
    new_factory->SetMainTaskList().emplace_back(
        make_pair(15, make_shared<CurbFollowTask>()));
    aux_task.emplace_back(&SideBrush);
    aux_task.emplace_back(&Loudspeaker);
    new_factory->SetAuxTaskList().emplace_back(make_pair(15, aux_task));
    // 任务工厂创建好后，放入状态机中
    test_fsm.ConstructFactory(new_factory);
  }
  // 打印工厂内的主任务清单和辅助任务清单
  num = test_fsm.factory_->SetMainTaskList().size();
  ROS_WARN("******* %d TASKS IN THE FACTORY *********", num);
  for (MainTaskList::iterator it = test_fsm.factory_->SetMainTaskList().begin();
       it != test_fsm.factory_->SetMainTaskList().end(); ++it) {
    cout << "Task ID " << it->first << ": "
         << State::TaskType_Name(it->second->GetTaskType()) << endl;
    for (AuxTaskList::iterator ait =
             test_fsm.factory_->SetAuxTaskList().begin();
         ait != test_fsm.factory_->SetAuxTaskList().end(); ++ait) {
      if (ait->first == it->first) {
        for (auto t : ait->second) {
          t();
        }
        break;
      }
    }
  }

  // 状态机按序执行任务工厂的所有任务，直到任务工厂为空
  // while (!test_fsm.factory_->IsFactoryEmpty()) {
  //   test_fsm.factory_->Start();
  //   sleep(1);
  //   ros::spinOnce();
  // }

  // 任务工厂执行完所有任务后，重置工厂，等待新的任务
  test_fsm.factory_.reset();
  std::cout << "exit main" << std::endl;
}
