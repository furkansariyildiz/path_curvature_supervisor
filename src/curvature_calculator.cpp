#include "path_curvature_supervisor/curvature_calculator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace path_curvature_supervisor
{

namespace
{

constexpr double kCurvatureDenominatorEpsilon = 1e-9;

// Returns the index within [lo, hi] whose arc_length is closest to target.
std::size_t closestIndexToArcLength(const std::vector<PathPoint>& path, double target,
                                    std::size_t lo, std::size_t hi)
{
  const auto begin = path.begin() + static_cast<std::ptrdiff_t>(lo);
  const auto end = path.begin() + static_cast<std::ptrdiff_t>(hi) + 1;

  const auto it = std::lower_bound(
      begin, end, target,
      [](const PathPoint& point, double value) { return point.arc_length < value; });

  const std::size_t idx = static_cast<std::size_t>(it - path.begin());

  if (idx <= lo)
  {
    return lo;
  }
  if (idx > hi)
  {
    return hi;
  }

  const double diff_at_idx = std::abs(path[idx].arc_length - target);
  const double diff_at_prev = std::abs(path[idx - 1].arc_length - target);
  return (diff_at_prev <= diff_at_idx) ? (idx - 1) : idx;
}

}  // namespace

void CurvatureCalculator::calculate(std::vector<PathPoint>& path, CurvatureMethod method,
                                    double calculation_distance)
{
  if (calculation_distance <= 0.0)
  {
    throw std::invalid_argument("CurvatureCalculator: calculation_distance must be > 0");
  }

  const std::size_t n = path.size();
  if (n == 0)
  {
    return;
  }
  if (n < 3)
  {
    // A path with fewer than 3 points has no well-defined interior point to
    // measure turning at; it is geometrically a single straight segment.
    for (auto& point : path)
    {
      point.curvature = 0.0;
    }
    return;
  }

  switch (method)
  {
    case CurvatureMethod::ThreePoint:
      calculateThreePoint(path, calculation_distance);
      break;
    case CurvatureMethod::HeadingDerivative:
      calculateHeadingDerivative(path, calculation_distance);
      break;
  }

  // Boundary points have no full window available; extrapolate from the
  // nearest interior point rather than computing from a degenerate window.
  path.front().curvature = path[1].curvature;
  path.back().curvature = path[n - 2].curvature;
}

void CurvatureCalculator::calculateThreePoint(std::vector<PathPoint>& path,
                                              double calculation_distance)
{
  const std::size_t n = path.size();

  for (std::size_t i = 1; i + 1 < n; ++i)
  {
    const double s_i = path[i].arc_length;
    const std::size_t j0 = closestIndexToArcLength(path, s_i - calculation_distance, 0, i - 1);
    const std::size_t j2 =
        closestIndexToArcLength(path, s_i + calculation_distance, i + 1, n - 1);

    const double x0 = path[j0].x;
    const double y0 = path[j0].y;
    const double x1 = path[i].x;
    const double y1 = path[i].y;
    const double x2 = path[j2].x;
    const double y2 = path[j2].y;

    const double cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    const double d01 = std::hypot(x1 - x0, y1 - y0);
    const double d12 = std::hypot(x2 - x1, y2 - y1);
    const double d02 = std::hypot(x2 - x0, y2 - y0);
    const double denominator = d01 * d12 * d02;

    path[i].curvature = (denominator > kCurvatureDenominatorEpsilon)
                             ? (2.0 * cross / denominator)
                             : 0.0;
  }
}

void CurvatureCalculator::calculateHeadingDerivative(std::vector<PathPoint>& path,
                                                     double calculation_distance)
{
  const std::size_t n = path.size();

  for (std::size_t i = 1; i + 1 < n; ++i)
  {
    const double s_i = path[i].arc_length;
    const std::size_t j0 = closestIndexToArcLength(path, s_i - calculation_distance, 0, i - 1);
    const std::size_t j2 =
        closestIndexToArcLength(path, s_i + calculation_distance, i + 1, n - 1);

    const double ds = path[j2].arc_length - path[j0].arc_length;
    if (ds > kCurvatureDenominatorEpsilon)
    {
      const double dpsi = normalizeAngle(path[j2].heading - path[j0].heading);
      path[i].curvature = dpsi / ds;
    }
    else
    {
      path[i].curvature = 0.0;
    }
  }
}

}  // namespace path_curvature_supervisor
