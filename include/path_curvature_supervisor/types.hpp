#ifndef PATH_CURVATURE_SUPERVISOR__TYPES_HPP_
#define PATH_CURVATURE_SUPERVISOR__TYPES_HPP_

#include <cmath>
#include <string>

namespace path_curvature_supervisor
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct PathPoint
{
  double x{0.0};
  double y{0.0};
  double arc_length{0.0};
  double heading{0.0};
  double curvature{0.0};
};

enum class CurvatureZone
{
  Straight,
  ModerateCurve,
  SharpCurve
};

struct CurveRegion
{
  std::size_t start_index{0};
  std::size_t end_index{0};

  double start_arc_length{0.0};
  double end_arc_length{0.0};

  double maximum_absolute_curvature{0.0};
  double average_absolute_curvature{0.0};

  CurvatureZone zone{CurvatureZone::Straight};
};

enum class ControllerMode
{
  Stanley,
  PurePursuit,
  MPC
};

enum class CurvatureMethod
{
  ThreePoint,
  HeadingDerivative
};

enum class CurvatureSmoothingMode
{
  SignedMovingAverage,
  Median
};

struct SupervisorOutput
{
  ControllerMode selected_controller{ControllerMode::Stanley};
  ControllerMode previous_controller{ControllerMode::Stanley};

  double current_arc_length{0.0};
  double preview_distance{0.0};
  double maximum_preview_curvature{0.0};
  double average_preview_curvature{0.0};
  double predicted_lateral_acceleration{0.0};

  bool controller_changed{false};
  double time_since_controller_change{0.0};

  // Fraction in [0, 1] of the configured blend duration that has elapsed
  // since the last controller change. Intended to be used by the caller as
  // the blend factor `alpha` in a bumpless-transfer scheme, see README.
  double transition_progress{1.0};
};

// Normalizes an angle in radians to the range (-pi, pi].
inline double normalizeAngle(double angle) noexcept
{
  constexpr double kPi = M_PI;
  constexpr double kTwoPi = 2.0 * M_PI;

  double result = std::fmod(angle + kPi, kTwoPi);
  if (result <= 0.0)
  {
    result += kTwoPi;
  }
  return result - kPi;
}

inline std::string toString(CurvatureZone zone)
{
  switch (zone)
  {
    case CurvatureZone::Straight:
      return "Straight";
    case CurvatureZone::ModerateCurve:
      return "ModerateCurve";
    case CurvatureZone::SharpCurve:
      return "SharpCurve";
  }
  return "Unknown";
}

inline std::string toString(ControllerMode mode)
{
  switch (mode)
  {
    case ControllerMode::Stanley:
      return "Stanley";
    case ControllerMode::PurePursuit:
      return "PurePursuit";
    case ControllerMode::MPC:
      return "MPC";
  }
  return "Unknown";
}

inline std::string toString(CurvatureMethod method)
{
  switch (method)
  {
    case CurvatureMethod::ThreePoint:
      return "ThreePoint";
    case CurvatureMethod::HeadingDerivative:
      return "HeadingDerivative";
  }
  return "Unknown";
}

inline std::string toString(CurvatureSmoothingMode mode)
{
  switch (mode)
  {
    case CurvatureSmoothingMode::SignedMovingAverage:
      return "SignedMovingAverage";
    case CurvatureSmoothingMode::Median:
      return "Median";
  }
  return "Unknown";
}

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__TYPES_HPP_
