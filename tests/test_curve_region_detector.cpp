#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "path_curvature_supervisor/curve_region_detector.hpp"

namespace path_curvature_supervisor
{
namespace
{

// Builds a synthetic already-curvature-annotated path. `segments` is a list
// of (start_s, end_s, curvature) triples, half-open on `end_s`; any arc
// length not covered by a segment defaults to curvature 0 (straight).
std::vector<PathPoint> buildSyntheticPath(double spacing, double total_length,
                                          const std::vector<std::array<double, 3>>& segments)
{
  std::vector<PathPoint> path;
  for (double s = 0.0; s <= total_length + 1e-9; s += spacing)
  {
    PathPoint point;
    point.x = s;
    point.y = 0.0;
    point.arc_length = s;
    point.heading = 0.0;
    point.curvature = 0.0;

    for (const auto& segment : segments)
    {
      if (s >= segment[0] - 1e-9 && s < segment[1] - 1e-9)
      {
        point.curvature = segment[2];
        break;
      }
    }
    path.push_back(point);
  }
  return path;
}

SupervisorConfig makeTestConfig()
{
  SupervisorConfig config;
  config.straight_curvature_threshold = 0.003;
  config.sharp_curvature_threshold = 0.015;
  config.minimum_region_length = 5.0;
  config.maximum_straight_gap_to_merge = 3.0;
  return config;
}

TEST(CurveRegionDetector, IdentifiesStraightModerateAndSharpSegmentsInOrder)
{
  SupervisorConfig config = makeTestConfig();
  config.region_entry_margin = 0.0;
  config.region_exit_margin = 0.0;

  const std::vector<PathPoint> path = buildSyntheticPath(
      0.25, 40.0,
      {{{0.0, 10.0, 0.0}}, {{10.0, 20.0, 0.008}}, {{20.0, 26.0, 0.02}}, {{26.0, 40.0, 0.0}}});

  const CurveRegionDetector detector(config);
  const std::vector<CurveRegion> regions = detector.detect(path);

  ASSERT_EQ(regions.size(), 4u);
  EXPECT_EQ(regions[0].zone, CurvatureZone::Straight);
  EXPECT_EQ(regions[1].zone, CurvatureZone::ModerateCurve);
  EXPECT_EQ(regions[2].zone, CurvatureZone::SharpCurve);
  EXPECT_EQ(regions[3].zone, CurvatureZone::Straight);

  EXPECT_NEAR(regions[0].end_arc_length, 9.75, 0.26);
  EXPECT_NEAR(regions[1].start_arc_length, 10.0, 0.26);
  EXPECT_NEAR(regions[2].start_arc_length, 20.0, 0.26);
  EXPECT_NEAR(regions[3].start_arc_length, 26.0, 0.26);
}

TEST(CurveRegionDetector, ShortIsolatedCurveBlipIsMergedAwayAsNoise)
{
  SupervisorConfig config = makeTestConfig();

  // A 1 m sharp "blip" surrounded by 40 m of straight path -- too short to
  // be a real curve region, should be absorbed into a straight neighbor.
  const std::vector<PathPoint> path =
      buildSyntheticPath(0.25, 40.0, {{{20.0, 21.0, 0.02}}});

  const CurveRegionDetector detector(config);
  const std::vector<CurveRegion> regions = detector.detect(path);

  ASSERT_EQ(regions.size(), 1u);
  EXPECT_EQ(regions[0].zone, CurvatureZone::Straight);
}

TEST(CurveRegionDetector, CloseCurveRegionsAcrossShortStraightGapAreMerged)
{
  SupervisorConfig config = makeTestConfig();
  config.region_entry_margin = 0.0;
  config.region_exit_margin = 0.0;

  // Two sharp segments separated by a 2 m straight gap (< maximum_straight_gap_to_merge).
  const std::vector<PathPoint> path = buildSyntheticPath(
      0.25, 40.0, {{{0.0, 10.0, 0.02}}, {{10.0, 12.0, 0.0}}, {{12.0, 22.0, 0.02}}});

  const CurveRegionDetector detector(config);
  const std::vector<CurveRegion> regions = detector.detect(path);

  ASSERT_EQ(regions.size(), 2u);
  EXPECT_EQ(regions[0].zone, CurvatureZone::SharpCurve);
  EXPECT_NEAR(regions[0].start_arc_length, 0.0, 0.26);
  EXPECT_NEAR(regions[0].end_arc_length, 22.0, 0.26);
  EXPECT_EQ(regions[1].zone, CurvatureZone::Straight);
}

TEST(CurveRegionDetector, EntryAndExitMarginsExtendCurveRegionsButNotStraight)
{
  SupervisorConfig config = makeTestConfig();
  config.region_entry_margin = 5.0;
  config.region_exit_margin = 2.0;

  const std::vector<PathPoint> path = buildSyntheticPath(
      0.25, 40.0, {{{0.0, 10.0, 0.0}}, {{10.0, 20.0, 0.02}}, {{20.0, 40.0, 0.0}}});

  const CurveRegionDetector detector(config);
  const std::vector<CurveRegion> regions = detector.detect(path);

  ASSERT_EQ(regions.size(), 3u);
  EXPECT_NEAR(regions[1].start_arc_length, 10.0 - 5.0, 0.26);
  EXPECT_NEAR(regions[1].end_arc_length, 20.0 + 2.0, 0.26);

  // Straight regions are never padded by the margins.
  EXPECT_NEAR(regions[0].end_arc_length, 9.75, 0.26);
  EXPECT_NEAR(regions[2].start_arc_length, 20.0, 0.26);
}

TEST(CurveRegionDetector, MarginsAreClampedToPathBounds)
{
  SupervisorConfig config = makeTestConfig();
  config.region_entry_margin = 100.0;
  config.region_exit_margin = 100.0;

  const std::vector<PathPoint> path =
      buildSyntheticPath(0.25, 20.0, {{{5.0, 15.0, 0.02}}});

  const CurveRegionDetector detector(config);
  const std::vector<CurveRegion> regions = detector.detect(path);

  ASSERT_FALSE(regions.empty());
  for (const auto& region : regions)
  {
    if (region.zone != CurvatureZone::Straight)
    {
      EXPECT_GE(region.start_arc_length, 0.0);
      EXPECT_LE(region.end_arc_length, 20.0);
    }
  }
}

TEST(CurveRegionDetector, EmptyPathProducesNoRegions)
{
  const SupervisorConfig config = makeTestConfig();
  const CurveRegionDetector detector(config);
  EXPECT_TRUE(detector.detect({}).empty());
}

}  // namespace
}  // namespace path_curvature_supervisor
