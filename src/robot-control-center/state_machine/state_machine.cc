/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "state_machine/state_machine.h"

#include <ros/ros.h>
#include <utility>

#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace state_machine {
void StateMachine::Init() {
  idle_state_ = new HeadStateIdle();
  sleep_state_ = new HeadStateSleep();
  navigation_state_ = new HeadStateNavigation();
  configuration_state_ = new HeadStateConfiguraion();
  charging_state_ = new HeadStateCharging();
  update_state_ = new HeadStateUpdate();
  exception_state_ = new HeadStateException();

  DataProxy* proxy = DataProxy::GetInstance();
  proxy->SetFirstState(State::IDLE);
  proxy->SetSecondState(State::INITIALIZING);
  proxy->SetTaskType(State::NONE);
  current_state_ptr_ = &idle_state_;
}

void StateMachine::Update(const ros::TimerEvent&) {
  if (!((*current_state_ptr_)->IsInitialized())) {
    (*current_state_ptr_)->Init();
  }
  (*current_state_ptr_)->Action();
}

bool StateMachine::ChangeTo(const State& state) {
  // 要切换状态与当前状态一致，若是则拒绝，拒绝原因为为已经处在当前状态
  DataProxy* proxy = DataProxy::GetInstance();
  if (state.first() == proxy->GetCurrentState().first()) {
    return false;
  }

  // 检查是否满足状态切换规则，若是，则切换状态，更新状态值和状态指针；若否则拒绝态切换表
  // TODO()：true为状态切换规则查询函数，该函数返回bool值
  if (true) {
    proxy->UpdatePreviousState();
    (*current_state_ptr_)->Reset();
    switch (state.first()) {
      case State::IDLE:
        current_state_ptr_ = &idle_state_;
        proxy->SetFirstState(State::IDLE);
        break;
      case State::SLEEP:
        current_state_ptr_ = &sleep_state_;
        proxy->SetFirstState(State::SLEEP);
        break;
      case State::NAVIGATION:
        current_state_ptr_ = &navigation_state_;
        proxy->SetFirstState(State::NAVIGATION);
        break;
      case State::CONFIGURATION:
        current_state_ptr_ = &configuration_state_;
        proxy->SetFirstState(State::CONFIGURATION);
        break;
      case State::CHARGING:
        current_state_ptr_ = &charging_state_;
        proxy->SetFirstState(State::CHARGING);
        break;
      case State::EXCEPTION:
        current_state_ptr_ = &exception_state_;
        proxy->SetFirstState(State::EXCEPTION);
        break;
      case State::UPDATE:
        current_state_ptr_ = &update_state_;
        proxy->SetFirstState(State::UPDATE);
      default:
        break;
    }
    proxy->SetSecondState(State::INITIALIZING);
    return true;
  } else {
    return false;
  }
}

void StateMachine::ConstructNavigationFactory(
    std::unique_ptr<BaseTaskFactory>& fac) {
  if (fac != nullptr) {
    HeadStateNavigation* p =dynamic_cast<HeadStateNavigation*>(navigation_state_);
    p->nav_factory_ = move(fac);
  }
}

}  // namespace state_machine