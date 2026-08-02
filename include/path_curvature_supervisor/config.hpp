#ifndef PATH_CURVATURE_SUPERVISOR__CONFIG_HPP_
#define PATH_CURVATURE_SUPERVISOR__CONFIG_HPP_

#include <stdexcept>
#include <string>

#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// Central, tunable configuration for every stage of the pipeline.
//
// All fields carry sensible defaults so a `SupervisorConfig{}` is usable
// out of the box, but every real deployment should retune the curvature
// and dwell-time thresholds against the target vehicle, see README.md
// section "Configuration parameters".
struct SupervisorConfig
{
  // --- Preprocessing -------------------------------------------------
  double resampling_distance{0.25};

  // --- Curvature calculation / smoothing ------------------------------
  double curvature_calculation_distance{2.0};
  double curvature_smoothing_distance{2.0};

  CurvatureMethod curvature_method{CurvatureMethod::ThreePoint};
  CurvatureSmoothingMode curvature_smoothing_mode{CurvatureSmoothingMode::SignedMovingAverage};

  // --- Zone classification -------------------------------------------
  double straight_curvature_threshold{0.003};
  double sharp_curvature_threshold{0.015};

  // --- Controller hysteresis thresholds --------------------------------
  double mpc_enter_curvature_threshold{0.015};
  double mpc_exit_curvature_threshold{0.010};

  double pure_pursuit_enter_curvature_threshold{0.005};
  double pure_pursuit_exit_curvature_threshold{0.003};

  double mpc_lateral_acceleration_enter_threshold{1.5};
  double mpc_lateral_acceleration_exit_threshold{1.0};

  // Enter/exit thresholds for the bottom tier transition, PID <-> Stanley.
  // PID here is a bare heading-error regulator with no cross-track-error
  // correction (see ControllerMode), so it should only be active on
  // essentially straight path segments -- these defaults are intentionally
  // much smaller than pure_pursuit_enter/exit_curvature_threshold.
  double stanley_enter_curvature_threshold{0.0008};
  double stanley_exit_curvature_threshold{0.0004};

  // Fraction (0-1] of `mpc_enter_curvature_threshold` that the *average*
  // preview curvature must also reach before a curvature-based upgrade is
  // confirmed. Guards against a single noisy curvature spike (already rare
  // after smoothing) triggering an unnecessary escalation. See README.md
  // section "Preview curvature analysis".
  double average_curvature_confirmation_ratio{0.4};

  // --- Preview distance -------------------------------------------------
  double minimum_preview_distance{5.0};
  double maximum_preview_distance{30.0};
  double preview_time{1.5};

  // --- Curve region extraction -------------------------------------------
  double minimum_region_length{5.0};
  double maximum_straight_gap_to_merge{3.0};

  // Arc length [m] a curve region's reported start is pulled backwards by,
  // and the amount its reported end is pushed forward by. Lets a downstream
  // consumer "see" the curve in `curveRegions()` slightly before/after the
  // raw geometric boundary. This is independent of, and complementary to,
  // the runtime speed-dependent preview distance used by `update()`.
  double region_entry_margin{5.0};
  double region_exit_margin{2.0};

  // --- Controller dwell time / transition -------------------------------
  double minimum_controller_dwell_time{1.0};

  // If true, a transition to a *more capable* controller (Stanley ->
  // PurePursuit -> MPC) is allowed to bypass the minimum dwell time. A
  // transition to a less demanding controller always respects dwell time.
  // See README.md section "Minimum dwell time".
  bool bypass_dwell_time_for_upgrade{true};

  // Duration [s] over which `SupervisorOutput::transition_progress` ramps
  // from 0 to 1 after a controller change.
  double controller_blend_duration{1.0};

  // --- Nearest-point search ----------------------------------------------
  // 0 = unbounded linear search over the whole path. > 0 = only search
  // within +/- this many resampled points of the last known index. See
  // README.md section "Performance".
  std::size_t nearest_point_search_window{0};

  void validate() const
  {
    requirePositive(resampling_distance, "resampling_distance");
    requirePositive(curvature_calculation_distance, "curvature_calculation_distance");
    requirePositive(curvature_smoothing_distance, "curvature_smoothing_distance");

    requireNonNegative(straight_curvature_threshold, "straight_curvature_threshold");
    requireNonNegative(sharp_curvature_threshold, "sharp_curvature_threshold");
    requireOrdered(straight_curvature_threshold, sharp_curvature_threshold,
                   "straight_curvature_threshold", "sharp_curvature_threshold");

    requireNonNegative(mpc_exit_curvature_threshold, "mpc_exit_curvature_threshold");
    requireNonNegative(mpc_enter_curvature_threshold, "mpc_enter_curvature_threshold");
    requireOrdered(mpc_exit_curvature_threshold, mpc_enter_curvature_threshold,
                   "mpc_exit_curvature_threshold", "mpc_enter_curvature_threshold");

    requireNonNegative(stanley_exit_curvature_threshold, "stanley_exit_curvature_threshold");
    requireNonNegative(stanley_enter_curvature_threshold, "stanley_enter_curvature_threshold");
    requireOrdered(stanley_exit_curvature_threshold, stanley_enter_curvature_threshold,
                   "stanley_exit_curvature_threshold", "stanley_enter_curvature_threshold");

    requireNonNegative(pure_pursuit_exit_curvature_threshold, "pure_pursuit_exit_curvature_threshold");
    requireNonNegative(pure_pursuit_enter_curvature_threshold, "pure_pursuit_enter_curvature_threshold");
    requireOrdered(pure_pursuit_exit_curvature_threshold, pure_pursuit_enter_curvature_threshold,
                   "pure_pursuit_exit_curvature_threshold", "pure_pursuit_enter_curvature_threshold");

    requireNonNegative(mpc_lateral_acceleration_exit_threshold, "mpc_lateral_acceleration_exit_threshold");
    requireNonNegative(mpc_lateral_acceleration_enter_threshold, "mpc_lateral_acceleration_enter_threshold");
    requireOrdered(mpc_lateral_acceleration_exit_threshold, mpc_lateral_acceleration_enter_threshold,
                   "mpc_lateral_acceleration_exit_threshold", "mpc_lateral_acceleration_enter_threshold");

    if (average_curvature_confirmation_ratio <= 0.0 || average_curvature_confirmation_ratio > 1.0)
    {
      throw std::invalid_argument(
          "SupervisorConfig: average_curvature_confirmation_ratio must be in (0, 1]");
    }

    requirePositive(minimum_preview_distance, "minimum_preview_distance");
    requirePositive(maximum_preview_distance, "maximum_preview_distance");
    requireOrdered(minimum_preview_distance, maximum_preview_distance,
                   "minimum_preview_distance", "maximum_preview_distance");
    requireNonNegative(preview_time, "preview_time");

    requireNonNegative(minimum_region_length, "minimum_region_length");
    requireNonNegative(maximum_straight_gap_to_merge, "maximum_straight_gap_to_merge");
    requireNonNegative(region_entry_margin, "region_entry_margin");
    requireNonNegative(region_exit_margin, "region_exit_margin");

    requireNonNegative(minimum_controller_dwell_time, "minimum_controller_dwell_time");
    requirePositive(controller_blend_duration, "controller_blend_duration");
  }

private:
  static void requirePositive(double value, const std::string& name)
  {
    if (!(value > 0.0))
    {
      throw std::invalid_argument("SupervisorConfig: " + name + " must be > 0");
    }
  }

  static void requireNonNegative(double value, const std::string& name)
  {
    if (!(value >= 0.0))
    {
      throw std::invalid_argument("SupervisorConfig: " + name + " must be >= 0");
    }
  }

  static void requireOrdered(double lower, double upper, const std::string& lower_name,
                             const std::string& upper_name)
  {
    if (!(lower <= upper))
    {
      throw std::invalid_argument("SupervisorConfig: " + lower_name + " must be <= " + upper_name);
    }
  }
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__CONFIG_HPP_
