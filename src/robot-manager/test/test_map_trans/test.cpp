#include "MapImageTrans.hpp"
#include <iostream>
#include <vector>
#include <pwd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#define USER_NAME getUserName()
#define HOME_DIR "/home/" + USER_NAME
#define MAP_DIRECTORY HOME_DIR + "/.robot/data/maps/"

std::string getUserName() {
  uid_t userid;
  struct passwd *pwd;
  userid = getuid();
  pwd = getpwuid(userid);
  return pwd->pw_name;
}

int main(int argc, char **argv) {

  std::vector<float> data{8.08972184773971, 24.15302297508977};

  MapImageTrans mit;
  mit.setYaml(MAP_DIRECTORY + "cba/cba.yaml");
  std::vector<float> tv(2, 0);
  tv = mit.mapToImage(std::vector<float>{data[0], data[1]});
  for (int i = 0; i < 2; i++) {
	data[i] = tv[i];

  }

  printf("map to image points is %f,%f\n", data[0], data[1]);

  tv = mit.imageToMap(std::vector<float>{data[0], data[1]});

  for (int i = 0; i < 2; i++) {
	data[i] = tv[i];

  }
  printf("image to map points is %f,%f\n", data[0], data[1]);

  return 0;
}