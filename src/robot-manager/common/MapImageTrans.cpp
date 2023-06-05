#include "MapImageTrans.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

void MapImageTrans::setYaml(std::string yn) {

  YAML::Node nd = YAML::LoadFile(yn);

  _fn = nd["image"].as<std::string>();
  _rel = nd["resolution"].as<float>();

  auto img = cv::imread(_fn, cv::IMREAD_GRAYSCALE);

  _h = img.rows;
  _w = img.cols;

  _ori = nd["origin"].as<std::vector<float>>();

  float theta = _ori[2];
  _cth = cos(theta);
  _sth = sin(theta);
  
}

std::vector<float> MapImageTrans::imageToMap(const std::vector<float> &src) {
  std::vector<float> dst(2, 0);
  // Upside down
  std::vector<float> s{src[0], _h - 1 - src[1]};
  // Rotate and translate
  dst[0] = _rel * (_cth * s[0] - _sth * s[1]) + _ori[0];
  dst[1] = _rel * (_sth * s[0] + _cth * s[1]) + _ori[1];

  return dst;
}

std::vector<float> MapImageTrans::mapToImage(const std::vector<float> &src) {
  std::vector<float> dst(2, 0);
  cv::Point2d src_point;

  // Rotate and translate
  // cv::Mat P_dst = _R.t() * (P_src - _t);
  std::vector<float> s{src[0] - _ori[0], src[1] - _ori[1]};
  dst[0] = (_cth * s[0] + _sth * s[1]) / _rel;
  dst[1] = (-_sth * s[0] + _cth * s[1]) / _rel;

  // Upside down
  dst[0] = cropNum((int)round(dst[0]), 0, _w - 1);
  dst[1] = cropNum(_h - 1 - (int)round(dst[1]), 0, _h - 1);

  return dst;
}
