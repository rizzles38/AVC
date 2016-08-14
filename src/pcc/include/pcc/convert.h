#pragma once

#include <string>

#include <pcc/types.h>
#include <nav_msgs/Path.h>

namespace pcc {

nav_msgs::Path convertToPath(const Curve& curve, std::string frame_id = "map");

// TODO: convertToPath that takes a position curve and a velocity profile curve
// and combines them into a time-specified path, with the velocity on the z-axis

} // namespace pcc
