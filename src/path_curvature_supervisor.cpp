#include "path_curvature_supervisor/path_curvature_supervisor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "path_curvature_supervisor/curvature_calculator.hpp"
#include "path_curvature_supervisor/curvature_smoother.hpp"
#include "path_curvature_supervisor/path_preprocessor.hpp"

namespace path_curvature_supervisor
{

PathCurvatureSupervisor::PathCurvatureSupervisor(const SupervisorConfig& config)
  : config_(config), controller_supervisor_(config_), curve_region_detector_(config_)
{
  config_.validate();
}

void PathCurvatureSupervisor::setPath(const std::vector<Point2D>& path)
{
  PathPreprocessor::validatePath(path);

  const std::vector<Point2D> deduplicated =
      PathPreprocessor::removeDuplicatePoints(path, config_.resampling_distance * 0.5);

  processed_path_ = PathPreprocessor::resample(deduplicated, config_.resampling_distance);
  PathPreprocessor::computeHeadings(processed_path_);

  CurvatureCalculator::calculate(processed_path_, config_.curvature_method,
                                 config_.curvature_calculation_distance);
  CurvatureSmoother::smooth(processed_path_, config_.curvature_smoothing_mode,
                            config_.curvature_smoothing_distance);

  curve_regions_ = curve_region_detector_.detect(processed_path_);

  has_path_ = !processed_path_.empty();
  last_known_index_.reset();
}

bool PathCurvatureSupervisor::hasPath() const noexcept
{
  return has_path_;
}

const std::vector<PathPoint>& PathCurvatureSupervisor::processedPath() const noexcept
{
  return processed_path_;
}

const std::vector<CurveRegion>& PathCurvatureSupervisor::curveRegions() const noexcept
{
  return curve_regions_;
}

PreviewMetrics PathCurvatureSupervisor::computePreviewMetrics(double current_arc_length,
                                                               double vehicle_velocity) const
{
  const double speed = std::abs(vehicle_velocity);
  const double preview_distance =
      std::clamp(config_.minimum_preview_distance + speed * config_.preview_time,
                config_.minimum_preview_distance, config_.maximum_preview_distance);

  const double window_end_arc_length = current_arc_length + preview_distance;

  const auto start_it = std::lower_bound(
      processed_path_.begin(), processed_path_.end(), current_arc_length,
      [](const PathPoint& point, double value) { return point.arc_length < value; });
  std::size_t start_index = static_cast<std::size_t>(start_it - processed_path_.begin());
  if (start_index >= processed_path_.size())
  {
    start_index = processed_path_.size() - 1;
  }

  const auto end_it = std::upper_bound(
      processed_path_.begin(), processed_path_.end(), window_end_arc_length,
      [](double value, const PathPoint& point) { return value < point.arc_length; });
  std::size_t end_index = start_index;
  if (end_it != processed_path_.begin())
  {
    end_index = static_cast<std::size_t>((end_it - 1) - processed_path_.begin());
  }
  if (end_index < start_index)
  {
    end_index = start_index;
  }

  double max_abs = 0.0;
  double sum_abs = 0.0;
  double sum_sq = 0.0;
  double max_signed_at_max_abs = 0.0;

  for (std::size_t i = start_index; i <= end_index; ++i)
  {
    const double curvature = processed_path_[i].curvature;
    const double abs_curvature = std::abs(curvature);
    sum_abs += abs_curvature;
    sum_sq += curvature * curvature;
    if (abs_curvature > max_abs)
    {
      max_abs = abs_curvature;
      max_signed_at_max_abs = curvature;
    }
  }

  const std::size_t count = end_index - start_index + 1;

  PreviewMetrics metrics;
  metrics.preview_distance = preview_distance;
  metrics.maximum_absolute_curvature = max_abs;
  metrics.average_absolute_curvature = sum_abs / static_cast<double>(count);
  metrics.rms_curvature = std::sqrt(sum_sq / static_cast<double>(count));
  metrics.maximum_signed_curvature = max_signed_at_max_abs;
  metrics.dominant_turn_direction =
      (max_abs > 0.0) ? ((max_signed_at_max_abs > 0.0) ? 1.0 : -1.0) : 0.0;

  return metrics;
}

SupervisorOutput PathCurvatureSupervisor::update(std::size_t nearest_path_index,
                                                 double vehicle_velocity,
                                                 double current_time_seconds)
{
  if (!has_path_)
  {
    throw std::logic_error("PathCurvatureSupervisor: setPath() must be called before update()");
  }
  if (nearest_path_index >= processed_path_.size())
  {
    throw std::out_of_range("PathCurvatureSupervisor: nearest_path_index out of range");
  }

  const double current_arc_length = processed_path_[nearest_path_index].arc_length;
  const PreviewMetrics metrics = computePreviewMetrics(current_arc_length, vehicle_velocity);

  return controller_supervisor_.update(metrics, vehicle_velocity, current_arc_length,
                                       current_time_seconds);
}

SupervisorOutput PathCurvatureSupervisor::update(const Point2D& vehicle_position,
                                                 double vehicle_velocity,
                                                 double current_time_seconds)
{
  if (!has_path_)
  {
    throw std::logic_error("PathCurvatureSupervisor: setPath() must be called before update()");
  }

  const std::size_t nearest_index = findNearestIndex(vehicle_position);
  return update(nearest_index, vehicle_velocity, current_time_seconds);
}

std::size_t PathCurvatureSupervisor::findNearestIndex(const Point2D& position)
{
  std::size_t lo = 0;
  std::size_t hi = processed_path_.size() - 1;

  if (config_.nearest_point_search_window > 0 && last_known_index_.has_value())
  {
    const std::size_t window = config_.nearest_point_search_window;
    const std::size_t last = *last_known_index_;
    lo = (last > window) ? (last - window) : 0;
    hi = std::min(processed_path_.size() - 1, last + window);
  }

  std::size_t best_index = lo;
  double best_distance_sq = std::numeric_limits<double>::max();

  for (std::size_t i = lo; i <= hi; ++i)
  {
    const double dx = processed_path_[i].x - position.x;
    const double dy = processed_path_[i].y - position.y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq)
    {
      best_distance_sq = distance_sq;
      best_index = i;
    }
  }

  last_known_index_ = best_index;
  return best_index;
}

void PathCurvatureSupervisor::resetSupervisorState()
{
  controller_supervisor_.reset();
  last_known_index_.reset();
}

}  // namespace path_curvature_supervisor
