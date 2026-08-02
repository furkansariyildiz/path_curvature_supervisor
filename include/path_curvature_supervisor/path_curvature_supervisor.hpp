#ifndef PATH_CURVATURE_SUPERVISOR__PATH_CURVATURE_SUPERVISOR_HPP_
#define PATH_CURVATURE_SUPERVISOR__PATH_CURVATURE_SUPERVISOR_HPP_

#include <optional>
#include <vector>

#include "path_curvature_supervisor/config.hpp"
#include "path_curvature_supervisor/controller_supervisor.hpp"
#include "path_curvature_supervisor/curve_region_detector.hpp"
#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// High-level facade: owns the whole pipeline (preprocessing, curvature,
// smoothing, region extraction, controller state machine) and exposes a
// small, stable API.
//
// Thread safety: `setPath()`, `update()` and `resetSupervisorState()` all
// mutate internal state and must not be called concurrently with each
// other or with themselves on the same instance. `processedPath()` and
// `curveRegions()` are read-only and safe to call concurrently with each
// other, but not concurrently with `setPath()`.
class PathCurvatureSupervisor
{
public:
  explicit PathCurvatureSupervisor(const SupervisorConfig& config);

  // Runs the full offline pipeline once: validation, deduplication,
  // resampling, arc length, heading, curvature, smoothing and region
  // extraction. O(N log N) in the input size. Does not reset the
  // controller state machine -- call `resetSupervisorState()` explicitly
  // if a path replacement should also force a fresh controller start.
  //
  // Throws std::invalid_argument if `path` contains a NaN/infinite
  // coordinate.
  void setPath(const std::vector<Point2D>& path);

  bool hasPath() const noexcept;

  const std::vector<PathPoint>& processedPath() const noexcept;

  const std::vector<CurveRegion>& curveRegions() const noexcept;

  // Advances the supervisor using a caller-supplied nearest-path-index.
  // Throws std::logic_error if no path has been set, and std::out_of_range
  // if `nearest_path_index` is not a valid index into `processedPath()`.
  SupervisorOutput update(std::size_t nearest_path_index, double vehicle_velocity,
                          double current_time_seconds);

  // Advances the supervisor by first locating the nearest processed path
  // point to `vehicle_position` (linear search, optionally windowed
  // around the last known index per `SupervisorConfig::
  // nearest_point_search_window` -- this is also the seam where a spatial
  // index such as a KD-tree could later replace the linear scan without
  // changing this API). Throws std::logic_error if no path has been set.
  SupervisorOutput update(const Point2D& vehicle_position, double vehicle_velocity,
                          double current_time_seconds);

  // Resets the controller state machine and the nearest-point search
  // cache. Does not clear the processed path.
  void resetSupervisorState();

private:
  SupervisorConfig config_;
  ControllerSupervisor controller_supervisor_;
  CurveRegionDetector curve_region_detector_;

  std::vector<PathPoint> processed_path_;
  std::vector<CurveRegion> curve_regions_;
  bool has_path_{false};

  std::optional<std::size_t> last_known_index_;

  std::size_t findNearestIndex(const Point2D& position);
  PreviewMetrics computePreviewMetrics(double current_arc_length, double vehicle_velocity) const;
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__PATH_CURVATURE_SUPERVISOR_HPP_
