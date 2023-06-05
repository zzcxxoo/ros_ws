/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include <ros/ros.h>

#include <filesystem>

#include "common/common.h"
#include "mobile_platform_msgs/SystemdService.h"

using namespace std;
using mobile_platform_msgs::SystemdService;

bool handle_systemd_service(SystemdService::Request &req,
                            SystemdService::Response &res) {
  res.message = req.service_name + " received";
  ROS_INFO("HI, %s got, thanks", req.service_name.c_str());
  return true;
}

void test_convert_yaml_to_json() {
  auto yf = "/home/jtcx/9tian_ws/src/robot-manager/test/test_yaml.yaml";
  std::cout << yf << endl;

  YAML::Node config = YAML::LoadFile(yf);
  Json::Value jnode;
  Json::Reader reader;
  YamlToJson(config, jnode);

  cout << Json::FastWriter().write(jnode) << endl;
}

void test_check_digit() {
  string a1 = "10.23c";
  string a2 = "10.232";

  cout << is_number(a1) << endl;
  cout << is_number(a2) << endl;
  cout << is_number("12..3") << endl;
  cout << is_number("-12..3") << endl;
  cout << is_number("+12.332") << endl;
  // if (std::isdigit(a2)) {
  //   cout << a2 << " is digit" << endl;
  // }
}

int main(int argc, char **argv) {
  // ros::init(argc, argv, "simple_example");
  // ros::NodeHandle n;

  // SystemdService srv;
  // ros::ServiceClient client =
  //     n.serviceClient<SystemdService>("/ui/charging/start");
  // if (client.call(srv)) {
  //   ROS_INFO("%s", srv.response.message.c_str());
  // } else {
  //   ROS_INFO("call server defeat");
  // }
  // ros::spin();

  test_convert_yaml_to_json();
  // test_check_digit();
  return 0;
}