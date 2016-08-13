#include <simulator/World.h>

#include <string>

#include <yaml-cpp/yaml.h>

namespace simulator {

World::World(ros::NodeHandle& node)
  : node_(node) {

  std::string world_file;
  ros::NodeHandle private_node("~");
  if (private_node.getParam("world_file", world_file)) {
    ROS_INFO("Loading world file '%s'.", world_file.c_str());
    load(world_file);
  } else {
    ROS_WARN("No world file defined.");
  }
}

void World::load(const std::string& file_name) {
  YAML::Node file = YAML::LoadFile(file_name);
  if (!file["world"]) {
    ROS_ERROR("Expected top-level 'world' element in world YAML.");
    return;
  }

  YAML::Node world = file["world"];
  if (!world.IsSequence()) {
    ROS_ERROR("Expected top-level 'world' element to be a sequence of boxes.");
    return;
  }

  for (std::size_t i = 0; i < world.size(); ++i) {
    if (!world[i].IsSequence()) {
      ROS_ERROR("Expected box %d to be a sequence of 2D points!", i);
      return;
    }

    auto box = world[i];
    for (std::size_t i = 0; i < box.size(); ++i)

  }
}

} // namespace simulator
