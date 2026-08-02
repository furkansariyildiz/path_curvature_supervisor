#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "path_curvature_supervisor/curvature_calculator.hpp"
#include "path_curvature_supervisor/curvature_smoother.hpp"
#include "path_curvature_supervisor/path_preprocessor.hpp"

namespace path_curvature_supervisor
{
namespace
{

std::vector<PathPoint> buildProcessedPath(const std::vector<Point2D>& raw, double sample_distance)
{
  std::vector<PathPoint> path = PathPreprocessor::resample(raw, sample_distance);
  PathPreprocessor::computeHeadings(path);
  return path;
}

std::vector<Point2D> makeStraightLine(double length, double spacing)
{
  std::vector<Point2D> points;
  for (double x = 0.0; x <= length + 1e-9; x += spacing)
  {
    points.push_back({x, 0.0});
  }
  return points;
}

std::vector<Point2D> makeCircle(double radius, int num_points, bool counter_clockwise)
{
  std::vector<Point2D> points;
  const double angular_step = (2.0 * M_PI) / static_cast<double>(num_points);
  for (int i = 0; i <= num_points; ++i)
  {
    const double theta = counter_clockwise ? (i * angular_step) : -(i * angular_step);
    points.push_back({radius * std::cos(theta), radius * std::sin(theta)});
  }
  return points;
}

TEST(CurvatureCalculator, StraightLineHasNearZeroCurvature)
{
  const std::vector<Point2D> raw = makeStraightLine(20.0, 1.0);
  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);

  CurvatureCalculator::calculate(path, CurvatureMethod::ThreePoint, 2.0);

  for (const auto& point : path)
  {
    EXPECT_NEAR(point.curvature, 0.0, 1e-6);
  }
}

TEST(CurvatureCalculator, CounterClockwiseCircleHasPositiveCurvature)
{
  constexpr double kRadius = 20.0;
  const std::vector<Point2D> raw = makeCircle(kRadius, 200, /*counter_clockwise=*/true);
  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);

  CurvatureCalculator::calculate(path, CurvatureMethod::ThreePoint, 2.0);

  const std::size_t mid = path.size() / 2;
  EXPECT_NEAR(path[mid].curvature, 1.0 / kRadius, 0.005);
  EXPECT_GT(path[mid].curvature, 0.0);
}

TEST(CurvatureCalculator, ClockwiseCircleHasNegativeCurvature)
{
  constexpr double kRadius = 20.0;
  const std::vector<Point2D> raw = makeCircle(kRadius, 200, /*counter_clockwise=*/false);
  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);

  CurvatureCalculator::calculate(path, CurvatureMethod::ThreePoint, 2.0);

  const std::size_t mid = path.size() / 2;
  EXPECT_NEAR(path[mid].curvature, -1.0 / kRadius, 0.005);
  EXPECT_LT(path[mid].curvature, 0.0);
}

TEST(CurvatureCalculator, SCurveSignChangesFromPositiveToNegative)
{
  std::vector<Point2D> raw;
  for (double x = 0.0; x <= 40.0 + 1e-9; x += 0.5)
  {
    raw.push_back({x, 5.0 * std::sin(2.0 * M_PI * x / 40.0)});
  }
  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);
  CurvatureCalculator::calculate(path, CurvatureMethod::ThreePoint, 2.0);

  const double first_quarter_curvature = path[path.size() / 8].curvature;
  const double third_quarter_curvature = path[(5 * path.size()) / 8].curvature;

  EXPECT_LT(first_quarter_curvature * third_quarter_curvature, 0.0)
      << "expected a sign change between the two humps of the S-curve";
}

TEST(CurvatureCalculator, TwoPointPathHasZeroCurvature)
{
  const std::vector<Point2D> raw{{0.0, 0.0}, {5.0, 0.0}};
  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);
  CurvatureCalculator::calculate(path, CurvatureMethod::ThreePoint, 2.0);

  for (const auto& point : path)
  {
    EXPECT_DOUBLE_EQ(point.curvature, 0.0);
  }
}

TEST(CurvatureCalculator, EmptyAndSinglePointPathsAreHandledSafely)
{
  std::vector<PathPoint> empty_path;
  EXPECT_NO_THROW(CurvatureCalculator::calculate(empty_path, CurvatureMethod::ThreePoint, 2.0));

  std::vector<PathPoint> single_path(1);
  EXPECT_NO_THROW(CurvatureCalculator::calculate(single_path, CurvatureMethod::ThreePoint, 2.0));
  EXPECT_DOUBLE_EQ(single_path[0].curvature, 0.0);
}

TEST(CurvatureCalculator, HeadingDerivativeMethodAgreesWithThreePointOnCircle)
{
  constexpr double kRadius = 20.0;
  const std::vector<Point2D> raw = makeCircle(kRadius, 200, /*counter_clockwise=*/true);
  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);

  CurvatureCalculator::calculate(path, CurvatureMethod::HeadingDerivative, 2.0);

  const std::size_t mid = path.size() / 2;
  EXPECT_NEAR(path[mid].curvature, 1.0 / kRadius, 0.01);
}

TEST(CurvatureSmoother, ReducesVarianceOnNoisyStraightLine)
{
  std::vector<Point2D> raw;
  double seed = 0.0;
  for (double x = 0.0; x <= 30.0 + 1e-9; x += 0.5)
  {
    // Deterministic pseudo-noise, no <random> needed for a reproducible test.
    seed = std::fmod(seed * 1.1 + 0.37, 1.0);
    const double noise = (seed - 0.5) * 0.02;
    raw.push_back({x, noise});
  }

  std::vector<PathPoint> path = buildProcessedPath(raw, 0.25);
  CurvatureCalculator::calculate(path, CurvatureMethod::ThreePoint, 2.0);

  auto variance_of_curvature = [](const std::vector<PathPoint>& p) {
    double mean = 0.0;
    for (const auto& point : p) mean += point.curvature;
    mean /= static_cast<double>(p.size());
    double variance = 0.0;
    for (const auto& point : p) variance += (point.curvature - mean) * (point.curvature - mean);
    return variance / static_cast<double>(p.size());
  };

  const double variance_before = variance_of_curvature(path);

  CurvatureSmoother::smooth(path, CurvatureSmoothingMode::SignedMovingAverage, 2.0);
  const double variance_after = variance_of_curvature(path);

  EXPECT_LT(variance_after, variance_before);
}

}  // namespace
}  // namespace path_curvature_supervisor
