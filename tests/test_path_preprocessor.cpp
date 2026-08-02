#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "path_curvature_supervisor/path_preprocessor.hpp"

namespace path_curvature_supervisor
{
namespace
{

TEST(PathPreprocessor, RemoveDuplicatePointsCollapsesCloseConsecutivePoints)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {0.001, 0.0}, {0.002, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const std::vector<Point2D> result = PathPreprocessor::removeDuplicatePoints(path, 0.1);

  ASSERT_EQ(result.size(), 3u);
  EXPECT_DOUBLE_EQ(result[0].x, 0.0);
  EXPECT_DOUBLE_EQ(result[1].x, 1.0);
  EXPECT_DOUBLE_EQ(result[2].x, 2.0);
}

TEST(PathPreprocessor, RemoveDuplicatePointsHandlesEmptyAndSinglePoint)
{
  EXPECT_TRUE(PathPreprocessor::removeDuplicatePoints({}, 0.1).empty());

  const std::vector<Point2D> single{{1.0, 2.0}};
  const std::vector<Point2D> result = PathPreprocessor::removeDuplicatePoints(single, 0.1);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0].x, 1.0);
}

TEST(PathPreprocessor, RemoveDuplicatePointsAllDuplicatesCollapseToOne)
{
  const std::vector<Point2D> path{{5.0, 5.0}, {5.0, 5.0}, {5.0, 5.0}};
  const std::vector<Point2D> result = PathPreprocessor::removeDuplicatePoints(path, 0.1);
  ASSERT_EQ(result.size(), 1u);
}

TEST(PathPreprocessor, ValidatePathThrowsOnNaN)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {std::numeric_limits<double>::quiet_NaN(), 0.0}};
  EXPECT_THROW(PathPreprocessor::validatePath(path), std::invalid_argument);
}

TEST(PathPreprocessor, ValidatePathThrowsOnInfinity)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}};
  EXPECT_THROW(PathPreprocessor::validatePath(path), std::invalid_argument);
}

TEST(PathPreprocessor, ValidatePathAcceptsFinitePath)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 1.0}};
  EXPECT_NO_THROW(PathPreprocessor::validatePath(path));
}

TEST(PathPreprocessor, ResampleEmptyReturnsEmpty)
{
  EXPECT_TRUE(PathPreprocessor::resample({}, 0.5).empty());
}

TEST(PathPreprocessor, ResampleSinglePointReturnsSinglePoint)
{
  const std::vector<Point2D> path{{3.0, 4.0}};
  const std::vector<PathPoint> result = PathPreprocessor::resample(path, 0.5);
  ASSERT_EQ(result.size(), 1u);
  EXPECT_DOUBLE_EQ(result[0].x, 3.0);
  EXPECT_DOUBLE_EQ(result[0].y, 4.0);
  EXPECT_DOUBLE_EQ(result[0].arc_length, 0.0);
}

TEST(PathPreprocessor, ResampleTwoPointsPreservesEndpoints)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}};
  const std::vector<PathPoint> result = PathPreprocessor::resample(path, 0.5);

  ASSERT_GE(result.size(), 2u);
  EXPECT_NEAR(result.front().x, 0.0, 1e-9);
  EXPECT_NEAR(result.back().x, 1.0, 1e-9);
  EXPECT_NEAR(result.back().arc_length, 1.0, 1e-9);
}

TEST(PathPreprocessor, ResampleProducesUniformSpacingAndPreservesLastPoint)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<PathPoint> result = PathPreprocessor::resample(path, 0.25);

  EXPECT_NEAR(result.back().x, 10.0, 1e-9);
  EXPECT_NEAR(result.back().arc_length, 10.0, 1e-9);

  for (std::size_t i = 1; i + 1 < result.size(); ++i)
  {
    EXPECT_NEAR(result[i].arc_length - result[i - 1].arc_length, 0.25, 1e-9);
  }
}

TEST(PathPreprocessor, ResampleThrowsOnNonPositiveSampleDistance)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_THROW(PathPreprocessor::resample(path, 0.0), std::invalid_argument);
  EXPECT_THROW(PathPreprocessor::resample(path, -1.0), std::invalid_argument);
}

TEST(PathPreprocessor, ComputeHeadingsStraightLineIsZero)
{
  std::vector<PathPoint> path;
  for (int i = 0; i < 5; ++i)
  {
    PathPoint point;
    point.x = static_cast<double>(i);
    point.y = 0.0;
    point.arc_length = static_cast<double>(i);
    path.push_back(point);
  }

  PathPreprocessor::computeHeadings(path);

  for (const auto& point : path)
  {
    EXPECT_NEAR(point.heading, 0.0, 1e-9);
  }
}

TEST(PathPreprocessor, ComputeHeadingsHandlesShortPaths)
{
  std::vector<PathPoint> empty_path;
  EXPECT_NO_THROW(PathPreprocessor::computeHeadings(empty_path));

  std::vector<PathPoint> single_path(1);
  PathPreprocessor::computeHeadings(single_path);
  EXPECT_DOUBLE_EQ(single_path[0].heading, 0.0);

  std::vector<PathPoint> two_path(2);
  two_path[0].x = 0.0;
  two_path[0].y = 0.0;
  two_path[1].x = 1.0;
  two_path[1].y = 1.0;
  PathPreprocessor::computeHeadings(two_path);
  EXPECT_NEAR(two_path[0].heading, M_PI / 4.0, 1e-9);
  EXPECT_NEAR(two_path[1].heading, M_PI / 4.0, 1e-9);
}

}  // namespace
}  // namespace path_curvature_supervisor
