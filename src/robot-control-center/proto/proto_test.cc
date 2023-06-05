#include <iostream>

#include "robot-control-center/proto/state.pb.h"

using state_machine::data_type::State;

int main() {
  std::cout << "enter proto test" << std::endl;
  State state;
  state.set_first(State::IDLE);


  std::cout << state.first() << std::endl;
  std::cout << state.HeadStateType_Name(state.first()) << std::endl;
  std::cout << state.second() << std::endl;
  std::cout << state.SecondStateType_Name(state.second()) << std::endl;
  std::cout << state.has_second() << std::endl;
  std::cout << State::TaskType_Name(State::CURB_FOLLOW) << std::endl;

  return 0;
}