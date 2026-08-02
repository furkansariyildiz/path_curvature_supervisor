#include "path_curvature_supervisor/controller_supervisor.hpp"

#include <algorithm>
#include <cmath>

namespace path_curvature_supervisor
{

ControllerSupervisor::ControllerSupervisor(const SupervisorConfig& config) : config_(config) {}

int ControllerSupervisor::tierOf(ControllerMode mode) noexcept
{
  switch (mode)
  {
    case ControllerMode::PID:
      return 0;
    case ControllerMode::Stanley:
      return 1;
    case ControllerMode::PurePursuit:
      return 2;
    case ControllerMode::MPC:
      return 3;
  }
  return 0;
}

ControllerMode ControllerSupervisor::decideTargetMode(const PreviewMetrics& metrics,
                                                      double predicted_lateral_acceleration) const
{
  // A single noisy curvature sample is already heavily damped by
  // CurvatureSmoother before it ever reaches this point. As a second,
  // independent safeguard, a curvature-based upgrade additionally requires
  // the *average* preview curvature to reach a fraction of the enter
  // threshold, so a brief, isolated spike in an otherwise straight preview
  // window cannot by itself trigger an escalation. A genuinely high
  // lateral-acceleration situation (high v^2*kappa) can still trigger MPC
  // even if peak curvature alone would not have -- see
  // README.md "Preview curvature analysis".
  const bool average_confirms_upgrade =
      metrics.average_absolute_curvature >=
      config_.mpc_enter_curvature_threshold * config_.average_curvature_confirmation_ratio;

  const bool mpc_enter = (metrics.maximum_absolute_curvature >= config_.mpc_enter_curvature_threshold &&
                          average_confirms_upgrade) ||
                         (predicted_lateral_acceleration >= config_.mpc_lateral_acceleration_enter_threshold);

  const bool mpc_exit = (metrics.maximum_absolute_curvature < config_.mpc_exit_curvature_threshold) &&
                        (predicted_lateral_acceleration < config_.mpc_lateral_acceleration_exit_threshold);

  const bool pure_pursuit_enter =
      metrics.maximum_absolute_curvature >= config_.pure_pursuit_enter_curvature_threshold;
  const bool pure_pursuit_exit =
      metrics.maximum_absolute_curvature < config_.pure_pursuit_exit_curvature_threshold;

  // PID (see ControllerMode) has no cross-track-error correction, so it is
  // only ever the target on essentially straight path segments.
  const bool stanley_enter =
      metrics.maximum_absolute_curvature >= config_.stanley_enter_curvature_threshold;
  const bool stanley_exit =
      metrics.maximum_absolute_curvature < config_.stanley_exit_curvature_threshold;

  switch (current_mode_)
  {
    case ControllerMode::MPC:
      if (!mpc_exit)
      {
        return ControllerMode::MPC;
      }
      if (!pure_pursuit_exit)
      {
        return ControllerMode::PurePursuit;
      }
      return stanley_exit ? ControllerMode::PID : ControllerMode::Stanley;

    case ControllerMode::PurePursuit:
      if (mpc_enter)
      {
        return ControllerMode::MPC;
      }
      if (!pure_pursuit_exit)
      {
        return ControllerMode::PurePursuit;
      }
      return stanley_exit ? ControllerMode::PID : ControllerMode::Stanley;

    case ControllerMode::Stanley:
      if (mpc_enter)
      {
        return ControllerMode::MPC;
      }
      if (pure_pursuit_enter)
      {
        return ControllerMode::PurePursuit;
      }
      return stanley_exit ? ControllerMode::PID : ControllerMode::Stanley;

    case ControllerMode::PID:
      // A single preview window can, in principle, already show enough
      // curvature/lateral-acceleration to justify jumping straight to
      // Pure Pursuit or MPC (e.g. right after a fresh, more demanding
      // path is set); it need not step through every tier in order.
      if (mpc_enter)
      {
        return ControllerMode::MPC;
      }
      if (pure_pursuit_enter)
      {
        return ControllerMode::PurePursuit;
      }
      return stanley_enter ? ControllerMode::Stanley : ControllerMode::PID;
  }

  return current_mode_;
}

SupervisorOutput ControllerSupervisor::update(const PreviewMetrics& metrics,
                                              double vehicle_velocity, double current_arc_length,
                                              double current_time_seconds)
{
  const double speed = std::abs(vehicle_velocity);
  const double predicted_lateral_acceleration = speed * speed * metrics.maximum_absolute_curvature;

  const ControllerMode target_mode = decideTargetMode(metrics, predicted_lateral_acceleration);

  bool changed = false;

  if (!initialized_)
  {
    // No real controller has been running yet: adopt the correct starting
    // mode immediately (e.g. the path may begin inside a sharp curve)
    // without treating this as a "transition" or waiting on dwell time.
    current_mode_ = target_mode;
    previous_mode_ = target_mode;
    last_mode_change_time_ = current_time_seconds;
    initialized_ = true;
  }
  else if (target_mode != current_mode_)
  {
    const bool is_upgrade = tierOf(target_mode) > tierOf(current_mode_);
    const double dwell_elapsed = current_time_seconds - last_mode_change_time_;
    const bool dwell_satisfied = dwell_elapsed >= config_.minimum_controller_dwell_time;

    if (dwell_satisfied || (is_upgrade && config_.bypass_dwell_time_for_upgrade))
    {
      previous_mode_ = current_mode_;
      current_mode_ = target_mode;
      last_mode_change_time_ = current_time_seconds;
      changed = true;
    }
  }

  const double time_since_change = std::max(0.0, current_time_seconds - last_mode_change_time_);
  const double transition_progress =
      std::clamp(time_since_change / config_.controller_blend_duration, 0.0, 1.0);

  SupervisorOutput output;
  output.selected_controller = current_mode_;
  output.previous_controller = previous_mode_;
  output.current_arc_length = current_arc_length;
  output.preview_distance = metrics.preview_distance;
  output.maximum_preview_curvature = metrics.maximum_absolute_curvature;
  output.average_preview_curvature = metrics.average_absolute_curvature;
  output.predicted_lateral_acceleration = predicted_lateral_acceleration;
  output.controller_changed = changed;
  output.time_since_controller_change = time_since_change;
  output.transition_progress = transition_progress;

  return output;
}

void ControllerSupervisor::reset()
{
  current_mode_ = ControllerMode::PID;
  previous_mode_ = ControllerMode::PID;
  initialized_ = false;
  last_mode_change_time_ = 0.0;
}

ControllerMode ControllerSupervisor::currentMode() const noexcept
{
  return current_mode_;
}

}  // namespace path_curvature_supervisor
