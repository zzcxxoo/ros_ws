#include "state_machine.h"

#include <ros/ros.h>
#include <ros/timer.h>
#include <std_msgs/UInt8.h>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "proxy/data_proxy.h"
#include "task_factory/auxiliay_tasks.h"
#include "task_factory/base_task_factory.h"
#include "task_factory/main_tasks.h"

using proxy::DataProxy;
using state_machine::StateMachine;
using namespace task_factory;
using namespace std;

void MainEventCallback(std_msgs::UInt8 msg) {
  std::cout << "event call back " << std::endl;
  State state;
  if (msg.data == 0) {
    state.set_first(State::IDLE);
  } else if (msg.data == 1) {
    state.set_first(State::SLEEP);
  } else if (msg.data == 2) {
    state.set_first(State::NAVIGATION);
  } else if (msg.data == 3) {
    state.set_first(State::CONFIGURATION);
  } else if (msg.data == 4) {
    state.set_first(State::CHARGING);
  } else if (msg.data == 5) {
    state.set_first(State::EXCEPTION);
  } else if (msg.data == 6) {
    state.set_first(State::UPDATE);
  }
  StateMachine* sta = StateMachine::GetInstance();
  if (sta->ChangeTo(state)) {
    ROS_ERROR("State changed SUCCESS");
  } else {
    ROS_ERROR("State Change FAIL");
  }

  StateMachine* fsm = StateMachine::GetInstance();
  DataProxy* proxy = DataProxy::GetInstance();

  ROS_WARN("Current State: %s, %s",
           proxy->GetCurrentState()
               .HeadStateType_Name(proxy->GetCurrentState().first())
               .c_str(),
           proxy->GetCurrentState()
               .SecondStateType_Name(proxy->GetCurrentState().second())
               .c_str());
}

void NavigationTaskCallback(std_msgs::UInt8 msg) {
  unique_ptr<BaseTaskFactory> nav_factory(new BaseTaskFactory());
  vector<AuxFuncPtr> aux_task;
  // add 1st main task
  nav_factory->SetMainTaskList().emplace_back(
      make_pair(1, make_shared<CurbFollowTask>()));
  // add 2nd main task and aux task
  nav_factory->SetMainTaskList().emplace_back(
      make_pair(13, make_shared<CoveragePlanningTask>()));
  aux_task.emplace_back(&MainBrush);
  aux_task.emplace_back(&Loudspeaker);
  aux_task.emplace_back(&Fan);
  aux_task.emplace_back(&SideBrush);
  nav_factory->SetAuxTaskList().emplace_back(make_pair(13, aux_task));
  aux_task.clear();
  // add 3rd main task and aux task
  nav_factory->SetMainTaskList().emplace_back(
      make_pair(7, make_shared<AvoidingTask>()));
  aux_task.emplace_back(&MainBrush);
  aux_task.emplace_back(&Fan);
  nav_factory->SetAuxTaskList().emplace_back(7, aux_task);
  aux_task.clear();
  // add 4th main task and aux task
  nav_factory->SetMainTaskList().emplace_back(
      make_pair(9, make_shared<CoveragePlanningTask>()));
  aux_task.emplace_back(&Loudspeaker);
  nav_factory->SetAuxTaskList().emplace_back(9, aux_task);
  aux_task.clear();
  // 任务工厂创建好后，放入状态机
  StateMachine* fsm = StateMachine::GetInstance();
  fsm->ConstructNavigationFactory(nav_factory);
}

int main(int argc, char* argv[]) {
  std::cout << "state machine test" << std::endl;
  ros::init(argc, argv, "state_machine_test");
  ros::NodeHandle n;

  StateMachine* state_machine = StateMachine::GetInstance();
  state_machine->Init();

  ros::Subscriber event_sub = n.subscribe("/event", 1, MainEventCallback);
  ros::Subscriber nav_task_sub =n.subscribe("/navigation_task", 1, NavigationTaskCallback);
  ros::Timer fsm =n.createTimer(ros::Duration(3), &StateMachine::Update, state_machine);

  ros::spin();
  return true;
}
