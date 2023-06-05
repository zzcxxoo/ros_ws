/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */

#include "main_tasks.h"
#include "state_machine/state_machine.h"
#include "proxy/data_proxy.h"

using proxy::DataProxy;

namespace task_factory {
CurbFollowTask::CurbFollowTask() {
  ros::NodeHandle nh;
  curb_sub_ =nh.subscribe("/cub_follow", 1, &CurbFollowTask::CurbCallback, this);
  task_type_ = State::CURB_FOLLOW;
  ROS_WARN("CurbFollowTask Constructed");
}

void CurbFollowTask::Execute() {
  if (!is_execution_done_) {
    ROS_INFO("Curb Follow Task is executing !!!");
    is_execution_done_ = true;
  } else {
    ROS_INFO("Curb Follow Task already executed");
  }
}

void CurbFollowTask::CurbCallback(std_msgs::Bool msg) {
  DataProxy* proxy = DataProxy::GetInstance();
  if (proxy->GetCurrentState().task() == task_type_) {
    if (msg.data) {
      ROS_INFO("Curb Follow Task: Finish CMD Received");
      is_task_finished_ = true;
    }
  } else {
    ROS_WARN("Current Task is not CurbFollow, Finish Task CMD rejected!");
  }
}

CoveragePlanningTask::CoveragePlanningTask() {
  ros::NodeHandle nh;
  cov_sub_ = nh.subscribe("/coverage_planning", 1,
                          &CoveragePlanningTask::CoverageCallback, this);
  task_type_ = State::COVERAGE_PLAN;
  ROS_WARN("Coverage Planning Task Constructed");
}

void CoveragePlanningTask::Execute() {
  if (!is_execution_done_) {
    ROS_INFO("Coverage Planning Task is executing !!!");
    is_execution_done_ = true;
  } else {
    ROS_INFO("Coverage Planning Task already executed");
  }
}

void CoveragePlanningTask::CoverageCallback(std_msgs::Bool msg) {
  DataProxy* proxy = DataProxy::GetInstance();
  if (proxy->GetCurrentState().task() == task_type_) {
    if (msg.data) {
      ROS_INFO("Coverage Planning Task: Finish CMD Received");
      is_task_finished_ = true;
    }
  } else {
    ROS_WARN("Current Task is not CoveagePlanning, Finish Task CMD rejected!");
  }
}

AvoidingTask::AvoidingTask() {
  ros::NodeHandle nh;
  avoid_sub_ = nh.subscribe("/avoiding", 1, &AvoidingTask::AvoidCallback, this);
  task_type_ = State::AVOID;
  ROS_WARN("Avoiding Task Constructed");
}

void AvoidingTask::Execute() {
  if (!is_execution_done_) {
    ROS_INFO("Avoiding Task is executing !!!");
    is_execution_done_ = true;
  } else {
    ROS_INFO("Avoiding Task already exectued");
  }
}

void AvoidingTask::AvoidCallback(std_msgs::Bool msg) {
  DataProxy* proxy = DataProxy::GetInstance();
  if (proxy->GetCurrentState().task() == task_type_) {
    if (msg.data) {
      ROS_INFO("Avoiding Task: Finish CMD Received");
      is_task_finished_ = true;
    }
  } else {
    ROS_WARN("Current Task is not CoveagePlanning, Finish Task CMD rejected!");
  }
}

}  // namespace task_factory
