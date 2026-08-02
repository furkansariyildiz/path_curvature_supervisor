#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "path_curvature_supervisor/controller_supervisor.hpp"
#include "path_curvature_supervisor/path_curvature_supervisor.hpp"

namespace path_curvature_supervisor
{
namespace
{

PreviewMetrics makeMetrics(double absolute_curvature)
{
  PreviewMetrics metrics;
  metrics.preview_distance = 20.0;
  metrics.maximum_absolute_curvature = absolute_curvature;
  metrics.average_absolute_curvature = absolute_curvature;
  metrics.rms_curvature = absolute_curvature;
  metrics.maximum_signed_curvature = absolute_curvature;
  metrics.dominant_turn_direction = (absolute_curvature > 0.0) ? 1.0 : 0.0;
  return metrics;
}

TEST(ControllerSupervisor, HysteresisPreventsOscillationAroundMpcThreshold)
{
  const SupervisorConfig config;  // defaults: mpc_enter=0.015, mpc_exit=0.010
  ControllerSupervisor supervisor(config);

  // First call initializes the state machine; 0.014 is enough to enter
  // PurePursuit (>= pure_pursuit_enter_curvature_threshold = 0.005) but not
  // MPC (< mpc_enter_curvature_threshold = 0.015).
  const SupervisorOutput first = supervisor.update(makeMetrics(0.014), 5.0, 0.0, 0.0);
  EXPECT_EQ(first.selected_controller, ControllerMode::PurePursuit);

  // 0.016 crosses the MPC-enter threshold: escalate.
  const SupervisorOutput second = supervisor.update(makeMetrics(0.016), 5.0, 0.0, 0.1);
  EXPECT_EQ(second.selected_controller, ControllerMode::MPC);
  EXPECT_TRUE(second.controller_changed);

  // 0.014 no longer triggers a downgrade: it is below mpc_enter (0.015) but
  // still above mpc_exit (0.010), so hysteresis keeps MPC active.
  const SupervisorOutput third = supervisor.update(makeMetrics(0.014), 5.0, 0.0, 0.2);
  EXPECT_EQ(third.selected_controller, ControllerMode::MPC);
  EXPECT_FALSE(third.controller_changed);

  const SupervisorOutput fourth = supervisor.update(makeMetrics(0.016), 5.0, 0.0, 0.3);
  EXPECT_EQ(fourth.selected_controller, ControllerMode::MPC);
  EXPECT_FALSE(fourth.controller_changed);
}

TEST(ControllerSupervisor, MinimumDwellTimeBlocksDowngradeUntilElapsed)
{
  const SupervisorConfig config;  // minimum_controller_dwell_time defaults to 1.0 s
  ControllerSupervisor supervisor(config);

  // Initialize directly into MPC.
  const SupervisorOutput init = supervisor.update(makeMetrics(0.02), 5.0, 0.0, 0.0);
  ASSERT_EQ(init.selected_controller, ControllerMode::MPC);

  // A very low curvature would normally downgrade all the way to Stanley,
  // but only 0.1 s of dwell time has elapsed.
  const SupervisorOutput too_soon = supervisor.update(makeMetrics(0.001), 5.0, 0.0, 0.1);
  EXPECT_EQ(too_soon.selected_controller, ControllerMode::MPC);
  EXPECT_FALSE(too_soon.controller_changed);

  // Past the 1.0 s dwell time, the downgrade is allowed to happen.
  const SupervisorOutput after_dwell = supervisor.update(makeMetrics(0.001), 5.0, 0.0, 1.1);
  EXPECT_EQ(after_dwell.selected_controller, ControllerMode::Stanley);
  EXPECT_TRUE(after_dwell.controller_changed);
}

TEST(ControllerSupervisor, UpgradeBypassesDwellTimeByDefault)
{
  const SupervisorConfig config;
  ControllerSupervisor supervisor(config);

  const SupervisorOutput init = supervisor.update(makeMetrics(0.001), 5.0, 0.0, 0.0);
  ASSERT_EQ(init.selected_controller, ControllerMode::Stanley);

  // Only 50 ms later, but an *upgrade* to MPC should not have to wait for
  // the 1 s dwell time -- a sharp curve must be handled immediately.
  const SupervisorOutput escalate = supervisor.update(makeMetrics(0.02), 5.0, 0.0, 0.05);
  EXPECT_EQ(escalate.selected_controller, ControllerMode::MPC);
  EXPECT_TRUE(escalate.controller_changed);
}

TEST(ControllerSupervisor, TransitionProgressRampsFromZeroToOne)
{
  SupervisorConfig config;
  config.controller_blend_duration = 1.0;
  ControllerSupervisor supervisor(config);

  supervisor.update(makeMetrics(0.001), 5.0, 0.0, 0.0);
  const SupervisorOutput changed = supervisor.update(makeMetrics(0.02), 5.0, 0.0, 0.0);
  EXPECT_NEAR(changed.transition_progress, 0.0, 1e-9);

  const SupervisorOutput halfway = supervisor.update(makeMetrics(0.02), 5.0, 0.0, 0.5);
  EXPECT_NEAR(halfway.transition_progress, 0.5, 1e-9);

  const SupervisorOutput settled = supervisor.update(makeMetrics(0.02), 5.0, 0.0, 2.0);
  EXPECT_NEAR(settled.transition_progress, 1.0, 1e-9);
}

TEST(ControllerSupervisor, ResetReturnsToInitialPidState)
{
  const SupervisorConfig config;
  ControllerSupervisor supervisor(config);

  supervisor.update(makeMetrics(0.02), 5.0, 0.0, 0.0);
  ASSERT_EQ(supervisor.currentMode(), ControllerMode::MPC);

  supervisor.reset();
  EXPECT_EQ(supervisor.currentMode(), ControllerMode::PID);

  // After reset, the next update re-initializes without dwell delay.
  const SupervisorOutput reinit = supervisor.update(makeMetrics(0.02), 5.0, 0.0, 100.0);
  EXPECT_EQ(reinit.selected_controller, ControllerMode::MPC);
  EXPECT_FALSE(reinit.controller_changed);
}

TEST(ControllerSupervisor, PidIsSelectedOnEssentiallyStraightPath)
{
  const SupervisorConfig config;  // stanley_enter_curvature_threshold defaults to 0.0008
  ControllerSupervisor supervisor(config);

  const SupervisorOutput output = supervisor.update(makeMetrics(0.0001), 5.0, 0.0, 0.0);
  EXPECT_EQ(output.selected_controller, ControllerMode::PID);
}

TEST(ControllerSupervisor, HysteresisPreventsOscillationAroundPidStanleyThreshold)
{
  const SupervisorConfig config;  // stanley_enter=0.0008, stanley_exit=0.0004
  ControllerSupervisor supervisor(config);

  const SupervisorOutput init = supervisor.update(makeMetrics(0.0001), 5.0, 0.0, 0.0);
  ASSERT_EQ(init.selected_controller, ControllerMode::PID);

  // Crosses stanley_enter_curvature_threshold: escalate to Stanley.
  const SupervisorOutput escalate = supervisor.update(makeMetrics(0.0009), 5.0, 0.0, 0.1);
  EXPECT_EQ(escalate.selected_controller, ControllerMode::Stanley);
  EXPECT_TRUE(escalate.controller_changed);

  // Drops below stanley_enter but stays above stanley_exit (0.0004):
  // hysteresis keeps Stanley active rather than dropping back to PID.
  const SupervisorOutput hold = supervisor.update(makeMetrics(0.0006), 5.0, 0.0, 1.2);
  EXPECT_EQ(hold.selected_controller, ControllerMode::Stanley);
  EXPECT_FALSE(hold.controller_changed);
}

TEST(PathCurvatureSupervisorPreview, SelectsMpcBeforeReachingUpcomingSharpCurve)
{
  SupervisorConfig config;
  config.resampling_distance = 0.25;
  config.curvature_calculation_distance = 2.0;
  config.curvature_smoothing_distance = 2.0;
  config.minimum_preview_distance = 5.0;
  config.maximum_preview_distance = 30.0;
  config.preview_time = 1.5;

  PathCurvatureSupervisor supervisor(config);

  constexpr double kRadius = 50.0;  // curvature = 1/50 = 0.02, above mpc_enter (0.015)
  constexpr double kArcLength = 20.0;

  std::vector<Point2D> path;
  for (double s = 0.0; s <= 40.0; s += 0.5)
  {
    path.push_back({s, 0.0});
  }
  for (double arc_s = 0.5; arc_s <= kArcLength; arc_s += 0.5)
  {
    const double theta = -M_PI / 2.0 + arc_s / kRadius;
    path.push_back({40.0 + kRadius * std::cos(theta), kRadius + kRadius * std::sin(theta)});
  }
  const double theta_end = -M_PI / 2.0 + kArcLength / kRadius;
  const double heading_end = theta_end + M_PI / 2.0;
  const Point2D arc_end = path.back();
  for (double s = 0.5; s <= 20.0; s += 0.5)
  {
    path.push_back({arc_end.x + s * std::cos(heading_end), arc_end.y + s * std::sin(heading_end)});
  }

  supervisor.setPath(path);

  const auto& processed = supervisor.processedPath();
  const auto it = std::lower_bound(
      processed.begin(), processed.end(), 35.0,
      [](const PathPoint& point, double value) { return point.arc_length < value; });
  ASSERT_NE(it, processed.end());
  const std::size_t vehicle_index = static_cast<std::size_t>(it - processed.begin());

  // Sanity check: the vehicle's own path point is still on the straight
  // part -- the curve has not geometrically started yet.
  EXPECT_LT(std::abs(processed[vehicle_index].curvature), config.straight_curvature_threshold);

  const SupervisorOutput output = supervisor.update(vehicle_index, /*vehicle_velocity=*/10.0,
                                                     /*current_time_seconds=*/0.0);

  EXPECT_EQ(output.selected_controller, ControllerMode::MPC)
      << "supervisor should look ahead into the preview window and select MPC before the "
         "vehicle physically reaches the upcoming sharp curve";
}

}  // namespace
}  // namespace path_curvature_supervisor
