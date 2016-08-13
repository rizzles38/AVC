#pragma once

#include <string>

#include <ros/ros.h>

namespace simulator {

class World {
public:
  explicit World(ros::NodeHandle& node);

private:
  void load(const std::string& file_name);

  ros::NodeHandle& node_;
};

} // namespace simulator
