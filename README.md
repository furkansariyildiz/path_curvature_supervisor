# path_curvature_supervisor

A reusable, framework-agnostic C++17 library that analyzes the curvature of
a known global path ahead of time and helps an autonomous-vehicle stack
choose which lateral controller (Stanley, Pure Pursuit, MPC) should be
active at each point along the drive, with hysteresis and dwell-time
guards so the choice is stable in closed-loop operation.

**The library code has no ROS / ROS2 dependency** -- no ROS headers, no
ROS message types, no `roscpp`/`rclcpp` API usage anywhere under
`include/` or `src/`. It is packaged as a standard catkin package purely
so it builds inside this repository's catkin workspace (`catkin_ws`)
alongside [`ros_controllers`](../ros_controllers), which is the stack this
library is meant to plug into. The same `include/`/`src/` tree could be
dropped into a plain CMake project with a couple of lines removed from
`CMakeLists.txt`.

## 1. Purpose

The global path is known in full before the vehicle starts moving. This
library turns that raw path into:

- a uniformly-resampled, arc-length-annotated path with heading and signed
  curvature at every point,
- a list of classified curve regions (straight / moderate / sharp),
- and, at runtime, a recommended `ControllerMode` for the vehicle's current
  position, based on the curvature *ahead* of it (not just at its current
  point), together with the diagnostic information needed to reason about
  *why* that controller was chosen and to perform a safe transition.

It does not compute steering commands itself -- it is a decision-support
layer that sits above the individual lateral controllers.

## 2. Architecture

```
Point2D[] (raw global path)
        |
        v
  PathPreprocessor      -- dedup, uniform resample, arc length, heading
        |
        v
  CurvatureCalculator    -- signed curvature per point (arc-length window)
        |
        v
  CurvatureSmoother      -- arc-length moving average / median
        |
        v
  CurveRegionDetector    -- per-point zone classification + region merge
        |
        v
  ControllerSupervisor   -- stateful hysteresis + dwell-time state machine
        |
        v
  SupervisorOutput
```

`PathCurvatureSupervisor` is a facade that owns one instance of each stage
and wires them together. Everything up to and including
`CurveRegionDetector` is **stateless** (pure functions of their input) and
runs once in `setPath()`. Only `ControllerSupervisor` carries state across
calls, and it depends on nothing but a small `PreviewMetrics` summary --
it has no notion of `Point2D`/`PathPoint` at all, which keeps the
controller-selection state machine testable in complete isolation from
path geometry (see `tests/test_controller_supervisor.cpp`).

## 3. Curvature formula

Default method, `CurvatureMethod::ThreePoint`, uses signed Menger
curvature over three points `P0, P1, P2`:

```
kappa = 2 * [(x1-x0)(y2-y0) - (y1-y0)(x2-x0)] / (|P1-P0| * |P2-P1| * |P2-P0|)
```

`P1` is the point being evaluated; `P0` and `P2` are the closest available
points at approximately `-curvature_calculation_distance` and
`+curvature_calculation_distance` arc length away (see section 5). An
alternative `CurvatureMethod::HeadingDerivative` method computes
`kappa = dpsi/ds` over the same arc-length window using the already
-computed headings; it is provided as an alternative/consistency check,
not the default.

## 4. Curvature sign convention

- **Positive** = left turn (counter-clockwise), consistent with a path
  traversed in order of increasing arc length in a standard right-handed
  x-y plane.
- **Negative** = right turn (clockwise).
- **Near zero** = straight.

Verified in `test_curvature_calculator.cpp` against a known-radius circle
traversed in both directions.

## 5. Why arc-length windows, not index windows

Global paths are rarely evenly sampled -- a recorded/planned path can have
long straight stretches with sparse waypoints and tight curves with dense
waypoints. An index-based window (e.g. "3 points behind, 3 ahead") would
therefore correspond to a *different physical distance* depending on local
point density, making the curvature estimate and the smoothing behavior
inconsistent along the path. Defining `curvature_calculation_distance` and
`curvature_smoothing_distance` in meters and locating the window endpoints
via `arc_length` (with `std::lower_bound`) keeps the curvature estimate's
physical meaning constant everywhere on the path, regardless of input
sampling density.

## 6. Why uniform resampling matters

Resampling to a fixed arc-length step before computing curvature has two
benefits:

1. It removes the residual sampling-density dependence described above at
   the source, so the arc-length-window search in step 5 has a
   predictable, bounded number of candidate points to scan.
2. It makes an index-count average over a window numerically equivalent
   to an arc-length-weighted average, because every point already
   represents an equal length of path. `CurvatureSmoother` relies on this:
   it does a plain arithmetic mean/median over the points inside the
   window, which is only correct because those points are uniformly
   spaced.

## 7. Preview distance

```
L_preview = clamp(L_min + v * T, L_min, L_max)
```

`v` is `|vehicle_velocity|` (see section on negative velocity below), `T`
is `preview_time`. A faster vehicle needs to look further ahead to react
to an upcoming curve in time; the clamp keeps the preview window bounded
between `minimum_preview_distance` and `maximum_preview_distance`
regardless of speed.

## 8. Lateral acceleration

```
predicted_lateral_acceleration = v^2 * maximum_absolute_curvature(preview window)
```

using the *maximum* absolute curvature in the preview window (the
worst-case bend the vehicle will encounter within the lookahead), and
`|vehicle_velocity|` for `v`. This is deliberately conservative: it
answers "how hard will the tightest upcoming bend push the vehicle
sideways at the current speed", which is the quantity that should gate
MPC selection, not an average that could mask a single sharp apex.

## 9. Hysteresis logic

Every enter/exit pair (`mpc_enter`/`mpc_exit_curvature_threshold`,
`pure_pursuit_enter`/`exit_curvature_threshold`,
`mpc_lateral_acceleration_enter`/`exit_threshold`) is intentionally
asymmetric (enter threshold > exit threshold). `ControllerSupervisor`
evaluates enter/exit conditions relative to the *current* mode (see the
`switch` in `decideTargetMode`), so a value oscillating between the enter
and exit thresholds does not cause the controller to flip back and forth
-- see `ControllerSupervisor.HysteresisPreventsOscillationAroundMpcThreshold`.

**Guarding against a single curvature spike:** a curvature-based upgrade
additionally requires the *average* preview curvature to reach
`average_curvature_confirmation_ratio * mpc_enter_curvature_threshold`.
The primary defense against noise is actually earlier in the pipeline --
`CurvatureSmoother` already damps single-sample spikes before they reach
the supervisor -- this ratio is a second, independent safeguard against a
short-but-real high-curvature feature that is not representative of the
whole preview window. An MPC upgrade can *also* be triggered purely by
`predicted_lateral_acceleration` crossing its own threshold, independent
of the curvature condition, because a moderate curvature at very high
speed can be just as dangerous as a sharp curvature at low speed
(`v^2 * kappa`).

## 10. Minimum dwell time

Once a controller has been active, it must stay active for at least
`minimum_controller_dwell_time` seconds before switching to a **lower**
tier (MPC > Pure Pursuit > Stanley), to avoid rapid, physically
unrealistic controller thrashing. A switch to a **higher** tier is, by
default (`bypass_dwell_time_for_upgrade = true`), allowed to happen
immediately: if the path ahead suddenly demands a more capable/cautious
controller, waiting out an arbitrary timer before reacting would be the
wrong safety trade-off. Set `bypass_dwell_time_for_upgrade = false` if
your integration prefers to always honor the dwell time symmetrically.
The very first `update()` call after construction or `resetSupervisorState()`
is treated as initialization, not a transition: the correct starting mode
is adopted immediately (e.g. the path may begin inside a sharp curve) and
`controller_changed` is reported as `false`, with no dwell delay.

## 11. Bumpless transfer on controller switches

The library does not blend steering commands itself, but supports a
caller-side blend via `SupervisorOutput::transition_progress`, which ramps
linearly from `0.0` to `1.0` over `controller_blend_duration` seconds
after `controller_changed` becomes true:

```cpp
const double alpha = output.transition_progress;
const double blended_delta = (1.0 - alpha) * old_controller_delta + alpha * new_controller_delta;
```

Things a real integration must additionally handle when switching lateral
controllers (not covered by this library, listed here so they aren't
forgotten):

- **PID/Stanley integral or previous-error state**: either reset it or
  seed it from the outgoing controller's last state so the new controller
  doesn't start from a discontinuous internal state.
- **MPC warm start**: initialize the new MPC solve from the outgoing
  controller's current trajectory/steering, not from zero, to avoid a
  transient in the first solve.
- **Steering command continuity**: use `transition_progress` (or your own
  blend) so the commanded steering angle itself has no step discontinuity
  at the switch instant.
- **Steering rate limiting**: apply the vehicle's `max_steering_rate` limit
  across the transition the same way it is applied within a single
  controller's operation.
- **First-cycle initialization**: a controller that has just become active
  must not assume it was "already running" -- any internal derivative/
  integral terms must be initialized on their first cycle after a switch.

## 12. Configuration parameters

All parameters live in `SupervisorConfig` (`include/path_curvature_supervisor/config.hpp`)
and are validated by `SupervisorConfig::validate()` (called by
`PathCurvatureSupervisor`'s constructor), which throws
`std::invalid_argument` with a descriptive message for negative distances,
zero/negative resampling distance, and inverted enter/exit or min/max
threshold pairs.

| Field | Meaning |
|---|---|
| `resampling_distance` | Uniform arc-length step [m] after preprocessing. |
| `curvature_calculation_distance` | Half-window [m] for the 3-point/heading-derivative curvature estimate. |
| `curvature_smoothing_distance` | Half-window [m] for curvature smoothing. |
| `curvature_method` | `ThreePoint` (default) or `HeadingDerivative`. |
| `curvature_smoothing_mode` | `SignedMovingAverage` (default) or `Median`. |
| `straight_curvature_threshold`, `sharp_curvature_threshold` | Per-point zone classification bounds. |
| `mpc_enter/exit_curvature_threshold`, `pure_pursuit_enter/exit_curvature_threshold` | Hysteresis thresholds on preview max curvature. |
| `mpc_lateral_acceleration_enter/exit_threshold` | Hysteresis thresholds on `v^2*kappa`. |
| `average_curvature_confirmation_ratio` | Spike-rejection ratio, see section 9. |
| `minimum_preview_distance`, `maximum_preview_distance`, `preview_time` | Preview distance formula, section 7. |
| `minimum_region_length` | Regions shorter than this are absorbed into a neighbor. |
| `maximum_straight_gap_to_merge` | Straight gaps shorter than this between two curve regions get merged into one. |
| `region_entry_margin`, `region_exit_margin` | Arc length a curve region's reported bounds are padded by. |
| `minimum_controller_dwell_time` | Section 10. |
| `bypass_dwell_time_for_upgrade` | Section 10. |
| `controller_blend_duration` | Section 11. |
| `nearest_point_search_window` | `0` = unbounded search; `>0` = only search `+/-N` points around the last known index. |

**All curvature and lateral-acceleration thresholds are placeholders and
must be recalibrated against the target vehicle's actual steering/dynamic
limits and against each controller's real tracking performance** before
use on a real vehicle -- see section 16.

## 13. Build instructions

This package is a standard catkin package: it must live under a catkin
workspace's `src/` directory (e.g. `catkin_ws/src/path_curvature_supervisor`,
alongside `ros_controllers`) and is built from the **workspace root**, not
from inside the package. All build artifacts go into the workspace's
shared `build/` and `devel/` directories -- this package never creates its
own `build/` directory.

```bash
cd ~/catkin_ws
catkin_make
# or, with catkin tools:
catkin build path_curvature_supervisor
source devel/setup.bash
```

To use this library from another catkin package (such as
`ros_controllers`), add it as a dependency in that package's
`package.xml` and `CMakeLists.txt`, the same way `ros_controllers`
already depends on `runge_kutta` and `mpc_controller`:

```xml
<!-- package.xml -->
<build_depend>path_curvature_supervisor</build_depend>
<build_export_depend>path_curvature_supervisor</build_export_depend>
<exec_depend>path_curvature_supervisor</exec_depend>
```

```cmake
# CMakeLists.txt
find_package(catkin REQUIRED COMPONENTS
  path_curvature_supervisor
  # ... other existing components
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
```

## 14. Running the tests

Tests are wired into catkin's standard test targets via `catkin_add_gtest`
(the same mechanism `runge_kutta` uses) and only build when
`CATKIN_ENABLE_TESTING` is set, which `catkin_make run_tests` / `catkin
run_tests` do automatically:

```bash
cd ~/catkin_ws
catkin_make run_tests_path_curvature_supervisor
# or, with catkin tools:
catkin run_tests path_curvature_supervisor --no-deps
```

`gtest` must be resolvable by catkin (it already is, as a `test_depend`
in `package.xml`); on a Debian/Ubuntu system this is normally satisfied by
the `libgtest-dev` package.

## 15. Example usage

See `examples/basic_usage.cpp`, built automatically as the
`path_curvature_supervisor_basic_usage` target:

```cpp
#include "path_curvature_supervisor/path_curvature_supervisor.hpp"

using namespace path_curvature_supervisor;

SupervisorConfig config;
config.resampling_distance = 0.25;
PathCurvatureSupervisor supervisor(config);

std::vector<Point2D> global_path{ /* ... */ };
supervisor.setPath(global_path);

const SupervisorOutput output = supervisor.update(nearest_index, vehicle_velocity, current_time);
std::cout << toString(output.selected_controller) << '\n';
```

## 16. Known limitations

- **Nearest-point search is linear**, optionally windowed around the last
  known index (`nearest_point_search_window`). This is intentional for a
  first version (see the design questions below) but `findNearestIndex()`
  is the one seam where a KD-tree or similar spatial index could be
  substituted without changing `PathCurvatureSupervisor`'s public API.
- **No native loop-path (closed path) support.** A looped path can be
  passed in, but arc length and preview-window search will treat it as an
  open path and will not wrap the preview window around the seam.
- **Signed curvature smoothing can under-estimate two same-magnitude,
  opposite-sign bends spaced closer than `curvature_smoothing_distance`**
  (see the design note in `curvature_smoother.hpp`). Keep the smoothing
  distance small relative to the tightest expected bend spacing.
- **Curve region entry/exit margins can overlap neighboring regions** when
  two curves are close together; `CurveRegionDetector` does not attempt to
  resolve such overlaps, it is left to the consumer.
- **This library does not run two controllers in parallel** during a
  transition; it only reports `transition_progress` for the caller to use
  in whatever blending scheme (or none) its controllers support. Running
  both controllers in parallel and cross-fading their outputs would be a
  more robust bumpless-transfer strategy than blending a single scalar,
  but adds real-time cost (two controllers evaluated every cycle) that
  this library intentionally leaves as an integration-level decision
  rather than imposing it as a default.
- **Curvature calculation/smoothing distances are fixed, not
  speed-scaled**, in this version. A larger effective window at high speed
  (where small path-following errors matter less per meter but more per
  second) is a reasonable future extension; the current version keeps
  behavior deterministic and independent of the vehicle state during the
  one-time `setPath()` preprocessing, which only depends on geometry.
- **Threshold values ship as placeholders** and must be tuned per vehicle,
  see section 16 note in section 12.

## Design questions considered

- *Is maximum preview curvature alone too sensitive?* Yes -- mitigated by
  smoothing (primary) and the average-curvature confirmation ratio
  (secondary), see section 9.
- *Should the curvature calculation distance depend on speed?* Not in this
  version, see "Known limitations" above; the geometry-only preprocessing
  pipeline intentionally has no speed dependence, only the runtime preview
  distance does.
- *Can path smoothing distort geometry?* Only the curvature *values* are
  smoothed, never the path's x/y geometry itself -- so the vehicle's
  actual planned trajectory is never altered by this library, only the
  curvature signal used for controller selection.
- *Does signed smoothing break at S-curves?* Discussed in
  `curvature_smoother.hpp` and section "Known limitations".
- *Would gain scheduling be better than switching controllers outright?*
  Plausible for Stanley/Pure Pursuit (both are single-gain-ish trackers),
  but MPC is a structurally different optimizer, not a gain variant of the
  others, so a discrete mode switch with hysteresis is the right
  granularity here; gain scheduling *within* a controller mode is
  complementary, not a replacement, and out of scope for this library.
- *Should MPC trigger on `v^2*kappa`, not just curvature?* Yes --
  implemented as an independent OR condition, section 9.
- *Should thresholds be defined via physical turning radius?* Curvature
  and radius are reciprocal (`kappa = 1/R`), so the config is equivalent
  either way; curvature was kept because it is the quantity actually
  computed and is signed (radius is not, without an extra sign field).
- *Is running two controllers in parallel during a transition needed?*
  Not implemented by this library by default, see "Known limitations";
  `transition_progress` is exposed precisely so a caller that *does* want
  to run both controllers and cross-fade can do so.

## Thread safety

Not thread-safe by default. `PathCurvatureSupervisor::setPath()`,
`update()` and `resetSupervisorState()` all mutate internal state and must
not be called concurrently with each other or with themselves.
`processedPath()` and `curveRegions()` are read-only accessors and are
safe to call concurrently with each other (but not concurrently with
`setPath()`). The same rule applies to `ControllerSupervisor::update()`/
`reset()` directly, if used standalone.
