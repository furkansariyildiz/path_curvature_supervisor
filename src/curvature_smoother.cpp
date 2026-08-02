#include "path_curvature_supervisor/curvature_smoother.hpp"

#include <algorithm>
#include <stdexcept>

namespace path_curvature_supervisor
{

void CurvatureSmoother::smooth(std::vector<PathPoint>& path, CurvatureSmoothingMode mode,
                               double smoothing_distance)
{
  if (smoothing_distance <= 0.0)
  {
    throw std::invalid_argument("CurvatureSmoother: smoothing_distance must be > 0");
  }

  if (path.size() < 2)
  {
    return;
  }

  switch (mode)
  {
    case CurvatureSmoothingMode::SignedMovingAverage:
      smoothSignedMovingAverage(path, smoothing_distance);
      break;
    case CurvatureSmoothingMode::Median:
      smoothMedian(path, smoothing_distance);
      break;
  }
}

void CurvatureSmoother::smoothSignedMovingAverage(std::vector<PathPoint>& path,
                                                   double smoothing_distance)
{
  const std::size_t n = path.size();
  std::vector<double> smoothed(n, 0.0);

  std::size_t lo = 0;
  std::size_t hi = 0;
  double sum = path[0].curvature;

  for (std::size_t i = 0; i < n; ++i)
  {
    const double s_i = path[i].arc_length;

    while (s_i - path[lo].arc_length > smoothing_distance)
    {
      sum -= path[lo].curvature;
      ++lo;
    }
    while (hi + 1 < n && path[hi + 1].arc_length - s_i <= smoothing_distance)
    {
      ++hi;
      sum += path[hi].curvature;
    }

    const std::size_t count = hi - lo + 1;
    smoothed[i] = sum / static_cast<double>(count);
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    path[i].curvature = smoothed[i];
  }
}

void CurvatureSmoother::smoothMedian(std::vector<PathPoint>& path, double smoothing_distance)
{
  const std::size_t n = path.size();
  std::vector<double> smoothed(n, 0.0);

  std::size_t lo = 0;
  std::size_t hi = 0;

  for (std::size_t i = 0; i < n; ++i)
  {
    const double s_i = path[i].arc_length;

    while (s_i - path[lo].arc_length > smoothing_distance)
    {
      ++lo;
    }
    while (hi + 1 < n && path[hi + 1].arc_length - s_i <= smoothing_distance)
    {
      ++hi;
    }

    std::vector<double> curvatures;
    curvatures.reserve(hi - lo + 1);
    for (std::size_t k = lo; k <= hi; ++k)
    {
      curvatures.push_back(path[k].curvature);
    }

    const std::size_t mid = curvatures.size() / 2;
    std::nth_element(curvatures.begin(), curvatures.begin() + static_cast<std::ptrdiff_t>(mid),
                     curvatures.end());
    if (curvatures.size() % 2 == 1)
    {
      smoothed[i] = curvatures[mid];
    }
    else
    {
      const double upper = curvatures[mid];
      const double lower =
          *std::max_element(curvatures.begin(), curvatures.begin() + static_cast<std::ptrdiff_t>(mid));
      smoothed[i] = 0.5 * (upper + lower);
    }
  }

  for (std::size_t i = 0; i < n; ++i)
  {
    path[i].curvature = smoothed[i];
  }
}

}  // namespace path_curvature_supervisor
