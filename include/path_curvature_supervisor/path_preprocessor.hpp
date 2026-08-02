#ifndef PATH_CURVATURE_SUPERVISOR__PATH_PREPROCESSOR_HPP_
#define PATH_CURVATURE_SUPERVISOR__PATH_PREPROCESSOR_HPP_

#include <vector>

#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// Turns a raw, possibly noisy/irregularly-spaced global path into a
// uniformly-resampled sequence of `PathPoint`s with arc length and heading
// filled in. Stateless: every method depends only on its arguments.
//
// Not thread-safe to call concurrently on the *same* output buffers, but
// separate calls with separate arguments have no shared state.
class PathPreprocessor
{
public:
  PathPreprocessor() = default;

  // Throws std::invalid_argument if any point has a NaN or infinite
  // coordinate.
  static void validatePath(const std::vector<Point2D>& path);

  // Removes consecutive points closer than `minimum_distance` to the last
  // kept point. The first point is always kept. `minimum_distance` must be
  // >= 0.
  static std::vector<Point2D> removeDuplicatePoints(const std::vector<Point2D>& path,
                                                     double minimum_distance);

  // Resamples `path` at uniform arc-length intervals of `sample_distance`
  // using linear interpolation. The first and last input points are always
  // preserved exactly (as the first and last output samples). Returns an
  // empty vector for an empty input, and a single point for a single-point
  // input. `sample_distance` must be > 0.
  //
  // Only x, y and arc_length are populated; heading and curvature are left
  // at their default value of 0.0 and must be filled in by later stages.
  static std::vector<PathPoint> resample(const std::vector<Point2D>& path, double sample_distance);

  // Fills in `PathPoint::heading` for every point using central difference
  // in the interior and forward/backward difference at the endpoints.
  // No-op for empty input; leaves heading at 0.0 for a single-point path.
  static void computeHeadings(std::vector<PathPoint>& path);

private:
  static std::vector<double> computeCumulativeArcLength(const std::vector<Point2D>& path);
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__PATH_PREPROCESSOR_HPP_
