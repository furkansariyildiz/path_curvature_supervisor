#ifndef PATH_CURVATURE_SUPERVISOR__CONTROLLER_SUPERVISOR_HPP_
#define PATH_CURVATURE_SUPERVISOR__CONTROLLER_SUPERVISOR_HPP_

#include "path_curvature_supervisor/config.hpp"
#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// Summary of the curvature situation in the vehicle's upcoming preview
// window, computed by `PathCurvatureSupervisor` from the processed path.
// Kept as a separate, path-agnostic struct so `ControllerSupervisor` has no
// dependency on path representation at all -- its only job is the
// controller-selection state machine.
struct PreviewMetrics
{
  double preview_distance{0.0};
  double maximum_absolute_curvature{0.0};
  double average_absolute_curvature{0.0};
  double rms_curvature{0.0};
  double maximum_signed_curvature{0.0};

  // +1.0 = dominant turn is to the left, -1.0 = right, 0.0 = none/straight.
  double dominant_turn_direction{0.0};
};

// Stateful controller-mode state machine with hysteresis and minimum dwell
// time. NOT thread-safe: `update()` mutates internal state and must not be
// called concurrently with another `update()` or `reset()` on the same
// instance.
class ControllerSupervisor
{
public:
  explicit ControllerSupervisor(const SupervisorConfig& config);

  // Advances the state machine with the latest preview metrics and returns
  // the full supervisor output for this cycle. `current_arc_length` and
  // `metrics.preview_distance` are copied through into the output purely
  // for the caller's convenience/diagnostics.
  SupervisorOutput update(const PreviewMetrics& metrics, double vehicle_velocity,
                          double current_arc_length, double current_time_seconds);

  // Resets to the initial state (Stanley, no history). Does not affect the
  // config.
  void reset();

  ControllerMode currentMode() const noexcept;

private:
  const SupervisorConfig& config_;

  ControllerMode current_mode_{ControllerMode::Stanley};
  ControllerMode previous_mode_{ControllerMode::Stanley};
  bool initialized_{false};
  double last_mode_change_time_{0.0};

  ControllerMode decideTargetMode(const PreviewMetrics& metrics,
                                  double predicted_lateral_acceleration) const;
  static int tierOf(ControllerMode mode) noexcept;
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__CONTROLLER_SUPERVISOR_HPP_
