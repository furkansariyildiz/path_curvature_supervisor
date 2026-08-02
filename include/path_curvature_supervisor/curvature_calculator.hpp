#ifndef PATH_CURVATURE_SUPERVISOR__CURVATURE_CALCULATOR_HPP_
#define PATH_CURVATURE_SUPERVISOR__CURVATURE_CALCULATOR_HPP_

#include <vector>

#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// Computes signed curvature for every point of an already-resampled,
// arc-length-annotated path. Stateless.
//
// Sign convention: positive curvature == left turn, negative == right
// turn, near-zero == straight. This matches the standard mathematical
// convention for a path traversed in order of increasing arc length in a
// right-handed x-y plane.
class CurvatureCalculator
{
public:
  CurvatureCalculator() = default;

  // Fills in `path[i].curvature` for every point. `path[i].heading` must
  // already be populated when `method == HeadingDerivative`.
  //
  // `calculation_distance` is an arc-length half-window (in meters): for an
  // interior point, the neighbors used are the closest points at
  // approximately `-calculation_distance` and `+calculation_distance` arc
  // length away. Near the path boundaries, where a full window is not
  // available, the boundary point's curvature is copied from its nearest
  // interior neighbor instead of being computed from a degenerate window.
  //
  // `calculation_distance` must be > 0.
  static void calculate(std::vector<PathPoint>& path, CurvatureMethod method,
                        double calculation_distance);

private:
  static void calculateThreePoint(std::vector<PathPoint>& path, double calculation_distance);
  static void calculateHeadingDerivative(std::vector<PathPoint>& path, double calculation_distance);
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__CURVATURE_CALCULATOR_HPP_
