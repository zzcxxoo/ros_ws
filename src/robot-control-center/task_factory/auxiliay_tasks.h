/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#pragma once

#include <iostream>
#include <ros/ros.h>

namespace task_factory {
void MainBrush() {
    ROS_WARN("main brush open");
}

void SideBrush() {
    ROS_WARN("side brush open");
}
void Fan() {
    ROS_WARN("fan open");
}
void Loudspeaker() {
    ROS_WARN("loudspeaker open");
}
void Light() {
    ROS_WARN("light open");
}

};  // namespace task_factory
