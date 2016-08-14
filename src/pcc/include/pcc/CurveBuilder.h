#pragma once

#include <cassert>
#include <vector>

#include <pcc/types.h>

namespace pcc {

class CurveBuilder {
public:
  using ConstIterator = std::vector<ControlPoint>::const_iterator;

  // Appends the given control point to the end of the curve.
  void append(ControlPoint control_point);

  // Inserts the given control point before the given location in the list of
  // control points.
  void insert(ConstIterator where, ControlPoint control_point);

  int size() const { return control_points_.size(); }

  ConstIterator begin() const {
    return control_points_.cbegin();
  }

  ConstIterator end() const {
    return control_points_.cend();
  }

  const ControlPoint& operator[](int index) {
    assert(0 <= index && index < size());
    return control_points_[index];
  }

  // Returns an iterator pointing at the control point closest in Euclidean
  // distance to the given position, or end() if there is no such control point.
  ConstIterator select(const Position& position);

  // Refines the control points into piecewise line segments, computing all the
  // additional curve data. The levels argument controls how many levels of
  // subdivision refinement are applied.
  Curve build(int levels) const {
    return Curve(control_points_, levels);
  }

private:
  std::vector<ControlPoint> control_points_;
};

} // namespace pcc
