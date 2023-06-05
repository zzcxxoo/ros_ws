/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include "base_task.h"

namespace task_factory {
class CurbFollowTask : public BaseTask {
 public:
  CurbFollowTask();
  void Execute() override;
  ~CurbFollowTask() { ROS_ERROR("Curb Follow Task destructor called"); }

 private:
  ros::Subscriber curb_sub_;
  void CurbCallback(std_msgs::Bool msg);
};

class CoveragePlanningTask : public BaseTask {
 public:
  CoveragePlanningTask();
  void Execute() override;
  ~CoveragePlanningTask() {
    ROS_ERROR("Coverage Planning Task desctructor called");
  }

 private:
  ros::Subscriber cov_sub_;
  void CoverageCallback(std_msgs::Bool msg);
};

class AvoidingTask : public BaseTask {
 public:
  AvoidingTask();
  void Execute() override;
  ~AvoidingTask() { ROS_ERROR("Avoiding Task destructor called"); }

 private:
  ros::Subscriber avoid_sub_;
  void AvoidCallback(std_msgs::Bool msg);
};

}  // namespace task_factory
