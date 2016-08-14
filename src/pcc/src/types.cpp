#include <pcc/types.h>

#include <algorithm>
#include <iterator>
#include <vector>

namespace pcc {

Curve::Curve(const std::vector<ControlPoint>& control_points, int levels)
  : tangents_(control_points.size(), Direction(0.0, 0.0)),
    normals_(control_points.size(), Direction(0.0, 0.0)),
    curvatures_(control_points.size(), Curvature(0.0)),
    is_control_points_(control_points.size(), true) {
  // TODO: Actually implement the PCC refinement algorithm here, right now
  // just copy the control points to test the pipeline.
  for (auto iter = control_points.cbegin(); iter != control_points.cend(); ++iter) {
    positions_.push_back(iter->position());
  }
}

} // namespace pcc
