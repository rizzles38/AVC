#pragma once

#include <cassert>
#include <vector>

#include <boost/optional.hpp>
#include <Eigen/Dense>

namespace pcc {

using Position = Eigen::Vector2d;
using Direction = Eigen::Vector2d;
using Curvature = double;

// A control point in a PCC curve that has a position an optional tangent
// constraint.
class ControlPoint {
public:
  explicit ControlPoint(Position position)
    : position_(position), tangent_(boost::none) {}

  ControlPoint(Position position, Direction tangent)
    : position_(position), tangent_(tangent) {}

  Position position() const { return position_; }
  void setPosition(Position position) { position_ = position; }

  boost::optional<Direction> tangent() const { return tangent_; }
  void setTangent(Direction tangent) { tangent_ = tangent; }
  void clearTangent() { tangent_ = boost::none; }

private:
  Position position_;
  boost::optional<Direction> tangent_;
};

// Represents all the refined curve data for a single point in the PCC.
struct CurvePoint {
  CurvePoint(Position position, Direction tangent, Direction normal, Curvature curvature)
    : position(position), tangent(tangent), normal(normal), curvature(curvature) {}

  Position position;
  Direction tangent;
  Direction normal;
  Curvature curvature;
};

// A refined PCC curve made up of line segments. Each curve point has position,
// tangent, normal, and curvature information accessible by its index. Curves
// cannot be constructed directly. They are built with a CurveBuilder. If you
// want to make control point modifications and quickly retesselate the curve,
// hang on to the CurveBuilder and rebuild it.
class Curve {
public:
  int size() const { return positions_.size(); }

  Position position(int index) const {
    assert(0 <= index && index < size());
    return positions_[index];
  }

  const std::vector<Position>& positions() const {
    return positions_;
  }

  Direction tangent(int index) const {
    assert(0 <= index && index < size());
    return tangents_[index];
  }

  const std::vector<Direction>& tangents() const {
    return tangents_;
  }

  Direction normal(int index) const {
    assert(0 <= index && index < size());
    return normals_[index];
  }

  const std::vector<Direction>& normals() const {
    return normals_;
  }

  Curvature curvature(int index) const {
    assert(0 <= index && index < size());
    return curvatures_[index];
  }

  const std::vector<Curvature>& curvatures() const {
    return curvatures_;
  }

  CurvePoint operator[](int index) const {
    return CurvePoint(position(index), tangent(index), normal(index), curvature(index));
  }

private:
  Curve(const std::vector<ControlPoint>& control_points, int levels);

  std::vector<Position> positions_;
  std::vector<Direction> tangents_;
  std::vector<Direction> normals_;
  std::vector<Curvature> curvatures_;

  friend class CurveBuilder;
};

} // namespace pcc
