#include <iostream>
#include <list>
#include <memory>
#include <vector>

void MainBrushControl() {
  std::cout << "this is MainBrushControl Func" << std::endl;
}
void SideBrushControl() {
  std::cout << "this is SideBrushControl Func" << std::endl;
}

typedef void (*AuxFuncPtr)();
typedef std::list<std::pair<int, std::vector<AuxFuncPtr>>> AuxTaskPair;
// typedef std::list<std::pair<int, std::unique_ptr<AuxFuncPtr>>> AuxTaskPair;

int main() {
  std::cout << "aux task test main" << std::endl;
  // AuxTaskPair aux_task_list;

  std::list<AuxFuncPtr> func_list;
  AuxTaskPair aux_task_list;
  {
    std::vector<AuxFuncPtr> task;
    task.emplace_back(&MainBrushControl);
    task.emplace_back(&MainBrushControl);
    task.emplace_back(&MainBrushControl);
    aux_task_list.emplace_back(std::make_pair(1, task));
    task.clear();
    task.emplace_back(&SideBrushControl);
    task.emplace_back(&SideBrushControl);
    task.emplace_back(&MainBrushControl);
    task.emplace_back(&MainBrushControl);
    aux_task_list.emplace_back(std::make_pair(18, task));
  }


  for (AuxTaskPair::iterator it = aux_task_list.begin();
       it != aux_task_list.end(); ++it) {
    std::cout << "execute task " << it->first << std::endl;
    for (auto task : it->second) {
      task();
    }
  }

  return 0;
}