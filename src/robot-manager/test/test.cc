
// #include <yaml-cpp/yaml.h>

#include <time.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "common/common.h"

using namespace std;
using json = nlohmann::json;

void test_json() {
  json data;

  data["mac_addr"] = "jt_code";
  data["age"] = 15;

  cout << data.dump() << endl;
}

int main() {
  // vector<string> output;
  // int returncode = open_popen("systemctl status mapping | grep Active",
  // output);

  // for (auto str : output) {
  //   printf("%s\n", str.c_str());
  // }

  // printf("%d", returncode);
  // std::ofstream out("config.yaml", ios::out | ios::app);

  // YAML::Node config;

  // if (config.IsNull()) cout << "yes!";

  // config["map"] = 354;

  // config["sac"] = 123;

  // out << std::endl << config;

  // config.remove(config[0]);

  // out.close();

  // std::ifstream in("macaddress.txt");
  // std::string mac;
  // in >> mac;
  // std::cout << mac;
  // in.close();

  // std::string mac = "1223:123:132132:12313";

  // std::for_each(mac.begin(), mac.end(), [](auto &str) {
  //   if (str == ':') str = '-';
  // });

  // std::cout << mac << std::endl;

  // std::cout << md5sum(mac) << std::endl;

  // std::fstream in;
  // in.open("jt_lego.png", std::ios::binary);
  // std::string file = "";
  // std::string res = "";
  // while (getline(in, file)) {
  //   res += file;
  // }
  // std::cout << res << std::endl;

  // int time_last;
  // time_last = time(NULL);
  // std::cout << time_last << std::endl;

  // in.close();

//  std::string mac_addr = "13234";
//  std::string mapName = "jt_lego";
//  std::string wpfile = "wrap";
//  std::string oss_file_url = "123213132w132";
//  std::string md5_sum = "shasdgscdjadsoajodisjoa";
//  int unix_time = time(NULL);
//
//  std::ifstream in("/usr/local/agent/macaddress");
//
//  std::string format =
//      "curl -X 'POST' 'http://1.117.217.164:18080/vcu/upload_map' -H "
//      "'accept:application/"
//      "json' -H 'Content-Type:application/"
//      "json' -d '{ \"mac_addr\": \"%s\", \"map_info\" "
//      ":{\"docs\":[\"%s\",\"%s\"],\"map_dir\":\"%s\",\"oss_"
//      "url\""
//      ":\"%s\"},\"sign\":\"%s\",\"unix\":%d}'";
//  char meta_info[512];
//  std::sprintf(meta_info, format.c_str(), mac_addr.c_str(), mapName.c_str(),
//               wpfile.c_str(), wpfile.c_str(), oss_file_url.c_str(),
//               md5_sum.c_str(), unix_time);
//
//  std::vector<std::string> meta_response;
//
//  open_popen(std::string(meta_info), meta_response);
//
//  std::cout << meta_response[0] << std::endl;

  // test_json();
  auto j = json::parse(
	  "{\"map_name\":\"jt_demo_1th\",\"property\":\"charging_point\",\"type\":\"point\",\"message\":{\"id\":0,\"name\":\"test0319\",\"data\":[-17.4,-11.5,90]}}");

  return 0;
}