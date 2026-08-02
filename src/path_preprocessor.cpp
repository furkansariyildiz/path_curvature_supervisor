#include "path_curvature_supervisor/path_preprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace path_curvature_supervisor
{

void PathPreprocessor::validatePath(const std::vector<Point2D>& path)
{
  for (const auto& point : path)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y))
    {
      throw std::invalid_argument(
          "PathPreprocessor: path contains a NaN or infinite coordinate");
    }
  }
}

std::vector<Point2D> PathPreprocessor::removeDuplicatePoints(const std::vector<Point2D>& path,
                                                               double minimum_distance)
{
  if (minimum_distance < 0.0)
  {
    throw std::invalid_argument("PathPreprocessor: minimum_distance must be >= 0");
  }

  std::vector<Point2D> result;
  if (path.empty())
  {
    return result;
  }

  result.reserve(path.size());
  result.push_back(path.front());

  for (std::size_t i = 1; i < path.size(); ++i)
  {
    const Point2D& last_kept = result.back();
    const double dx = path[i].x - last_kept.x;
    const double dy = path[i].y - last_kept.y;
    const double distance = std::hypot(dx, dy);
    if (distance >= minimum_distance)
    {
      result.push_back(path[i]);
    }
  }

  return result;
}

std::vector<double> PathPreprocessor::computeCumulativeArcLength(const std::vector<Point2D>& path)
{
  std::vector<double> arc_length(path.size(), 0.0);
  for (std::size_t i = 1; i < path.size(); ++i)
  {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    arc_length[i] = arc_length[i - 1] + std::hypot(dx, dy);
  }
  return arc_length;
}

std::vector<PathPoint> PathPreprocessor::resample(const std::vector<Point2D>& path,
                                                    double sample_distance)
{
  if (sample_distance <= 0.0)
  {
    throw std::invalid_argument("PathPreprocessor: sample_distance must be > 0");
  }

  std::vector<PathPoint> result;
  if (path.empty())
  {
    return result;
  }

  if (path.size() == 1)
  {
    PathPoint point;
    point.x = path.front().x;
    point.y = path.front().y;
    point.arc_length = 0.0;
    result.push_back(point);
    return result;
  }

  const std::vector<double> arc_length = computeCumulativeArcLength(path);
  const double total_length = arc_length.back();

  if (total_length <= 0.0)
  {
    // All input points collapsed onto the same location.
    PathPoint point;
    point.x = path.front().x;
    point.y = path.front().y;
    point.arc_length = 0.0;
    result.push_back(point);
    return result;
  }

  result.reserve(static_cast<std::size_t>(total_length / sample_distance) + 2);

  std::size_t segment = 0;
  constexpr double kEpsilon = 1e-9;

  for (double s = 0.0; s < total_length - kEpsilon; s += sample_distance)
  {
    while (segment + 1 < arc_length.size() && arc_length[segment + 1] < s)
    {
      ++segment;
    }

    const double segment_start = arc_length[segment];
    const double segment_end = arc_length[segment + 1];
    const double segment_length = segment_end - segment_start;
    const double t = (segment_length > kEpsilon) ? (s - segment_start) / segment_length : 0.0;

    PathPoint point;
    point.x = path[segment].x + t * (path[segment + 1].x - path[segment].x);
    point.y = path[segment].y + t * (path[segment + 1].y - path[segment].y);
    point.arc_length = s;
    result.push_back(point);
  }

  // Always emit the exact final point so the tail of the path is never lost.
  PathPoint last_point;
  last_point.x = path.back().x;
  last_point.y = path.back().y;
  last_point.arc_length = total_length;
  result.push_back(last_point);

  return result;
}

void PathPreprocessor::computeHeadings(std::vector<PathPoint>& path)
{
  const std::size_t n = path.size();
  if (n < 2)
  {
    return;
  }

  path.front().heading =
      std::atan2(path[1].y - path[0].y, path[1].x - path[0].x);
  path.back().heading =
      std::atan2(path[n - 1].y - path[n - 2].y, path[n - 1].x - path[n - 2].x);

  for (std::size_t i = 1; i + 1 < n; ++i)
  {
    path[i].heading = std::atan2(path[i + 1].y - path[i - 1].y, path[i + 1].x - path[i - 1].x);
  }
}

}  // namespace path_curvature_supervisor
