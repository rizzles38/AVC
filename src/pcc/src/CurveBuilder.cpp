#include <pcc/CurveBuilder.h>

namespace pcc {

void CurveBuilder::append(ControlPoint control_point) {
  control_points_.push_back(control_point);
}

void CurveBuilder::insert(CurveBuilder::ConstIterator where, ControlPoint control_point) {
  control_points_.insert(where, control_point);
}

CurveBuilder::ConstIterator CurveBuilder::select(const Position& position) {
  // TODO
  return end();
}

} // namespace pcc
