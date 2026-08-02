#include "path_curvature_supervisor/curve_region_detector.hpp"

#include <algorithm>
#include <cmath>

namespace path_curvature_supervisor
{

CurveRegionDetector::CurveRegionDetector(const SupervisorConfig& config) : config_(config) {}

CurvatureZone CurveRegionDetector::classify(double absolute_curvature) const
{
  if (absolute_curvature < config_.straight_curvature_threshold)
  {
    return CurvatureZone::Straight;
  }
  if (absolute_curvature < config_.sharp_curvature_threshold)
  {
    return CurvatureZone::ModerateCurve;
  }
  return CurvatureZone::SharpCurve;
}

void CurveRegionDetector::recomputeStatistics(CurveRegion& region,
                                              const std::vector<PathPoint>& path) const
{
  region.start_arc_length = path[region.start_index].arc_length;
  region.end_arc_length = path[region.end_index].arc_length;

  double max_abs = 0.0;
  double sum_abs = 0.0;
  const std::size_t count = region.end_index - region.start_index + 1;
  for (std::size_t i = region.start_index; i <= region.end_index; ++i)
  {
    const double abs_curvature = std::abs(path[i].curvature);
    max_abs = std::max(max_abs, abs_curvature);
    sum_abs += abs_curvature;
  }
  region.maximum_absolute_curvature = max_abs;
  region.average_absolute_curvature = sum_abs / static_cast<double>(count);
}

std::vector<CurveRegion> CurveRegionDetector::buildRawRegions(
    const std::vector<PathPoint>& path) const
{
  std::vector<CurveRegion> regions;
  if (path.empty())
  {
    return regions;
  }

  std::size_t start = 0;
  CurvatureZone current_zone = classify(std::abs(path[0].curvature));

  for (std::size_t i = 1; i <= path.size(); ++i)
  {
    const bool at_end = (i == path.size());
    const CurvatureZone zone = at_end ? current_zone : classify(std::abs(path[i].curvature));

    if (at_end || zone != current_zone)
    {
      CurveRegion region;
      region.start_index = start;
      region.end_index = i - 1;
      region.zone = current_zone;
      recomputeStatistics(region, path);
      regions.push_back(region);

      start = i;
      current_zone = zone;
    }
  }

  return regions;
}

void CurveRegionDetector::mergeShortRegions(std::vector<CurveRegion>& regions,
                                            const std::vector<PathPoint>& path) const
{
  if (regions.size() <= 1)
  {
    return;
  }

  bool merged_any = true;
  std::size_t safety_counter = 0;
  const std::size_t max_iterations = regions.size() + 1;

  while (merged_any && regions.size() > 1 && safety_counter < max_iterations)
  {
    merged_any = false;
    ++safety_counter;

    for (std::size_t idx = 0; idx < regions.size(); ++idx)
    {
      const double length = regions[idx].end_arc_length - regions[idx].start_arc_length;
      if (length >= config_.minimum_region_length)
      {
        continue;
      }

      const bool has_prev = idx > 0;
      const bool has_next = idx + 1 < regions.size();
      if (!has_prev && !has_next)
      {
        break;
      }

      std::size_t neighbor_idx;
      if (has_prev && has_next)
      {
        const double prev_len = regions[idx - 1].end_arc_length - regions[idx - 1].start_arc_length;
        const double next_len = regions[idx + 1].end_arc_length - regions[idx + 1].start_arc_length;
        neighbor_idx = (prev_len >= next_len) ? (idx - 1) : (idx + 1);
      }
      else
      {
        neighbor_idx = has_prev ? (idx - 1) : (idx + 1);
      }

      const CurvatureZone absorbing_zone = regions[neighbor_idx].zone;
      const std::size_t merged_start = std::min(regions[idx].start_index, regions[neighbor_idx].start_index);
      const std::size_t merged_end = std::max(regions[idx].end_index, regions[neighbor_idx].end_index);

      const std::size_t keep_idx = std::min(idx, neighbor_idx);
      const std::size_t drop_idx = std::max(idx, neighbor_idx);

      regions[keep_idx].start_index = merged_start;
      regions[keep_idx].end_index = merged_end;
      regions[keep_idx].zone = absorbing_zone;
      recomputeStatistics(regions[keep_idx], path);
      regions.erase(regions.begin() + static_cast<std::ptrdiff_t>(drop_idx));

      merged_any = true;
      break;
    }
  }
}

void CurveRegionDetector::mergeCloseCurveRegions(std::vector<CurveRegion>& regions,
                                                 const std::vector<PathPoint>& path) const
{
  bool merged_any = true;
  std::size_t safety_counter = 0;
  const std::size_t max_iterations = regions.size() + 1;

  while (merged_any && safety_counter < max_iterations)
  {
    merged_any = false;
    ++safety_counter;

    for (std::size_t idx = 0; idx + 2 < regions.size(); ++idx)
    {
      const CurveRegion& a = regions[idx];
      const CurveRegion& gap = regions[idx + 1];
      const CurveRegion& b = regions[idx + 2];

      const bool a_is_curve = (a.zone != CurvatureZone::Straight);
      const bool b_is_curve = (b.zone != CurvatureZone::Straight);
      const bool gap_is_straight = (gap.zone == CurvatureZone::Straight);
      const double gap_length = gap.end_arc_length - gap.start_arc_length;

      if (a_is_curve && b_is_curve && gap_is_straight &&
          gap_length < config_.maximum_straight_gap_to_merge)
      {
        CurveRegion merged;
        merged.start_index = a.start_index;
        merged.end_index = b.end_index;
        merged.zone = (a.zone == CurvatureZone::SharpCurve || b.zone == CurvatureZone::SharpCurve)
                          ? CurvatureZone::SharpCurve
                          : CurvatureZone::ModerateCurve;
        recomputeStatistics(merged, path);

        regions[idx] = merged;
        regions.erase(regions.begin() + static_cast<std::ptrdiff_t>(idx) + 1,
                      regions.begin() + static_cast<std::ptrdiff_t>(idx) + 3);

        merged_any = true;
        break;
      }
    }
  }
}

void CurveRegionDetector::coalesceAdjacentSameZoneRegions(std::vector<CurveRegion>& regions,
                                                          const std::vector<PathPoint>& path) const
{
  for (std::size_t idx = 0; idx + 1 < regions.size();)
  {
    if (regions[idx].zone == regions[idx + 1].zone)
    {
      regions[idx].end_index = regions[idx + 1].end_index;
      recomputeStatistics(regions[idx], path);
      regions.erase(regions.begin() + static_cast<std::ptrdiff_t>(idx) + 1);
    }
    else
    {
      ++idx;
    }
  }
}

void CurveRegionDetector::applyEntryExitMargins(std::vector<CurveRegion>& regions,
                                                double total_arc_length) const
{
  for (auto& region : regions)
  {
    if (region.zone == CurvatureZone::Straight)
    {
      continue;
    }
    region.start_arc_length = std::max(0.0, region.start_arc_length - config_.region_entry_margin);
    region.end_arc_length =
        std::min(total_arc_length, region.end_arc_length + config_.region_exit_margin);
  }
}

std::vector<CurveRegion> CurveRegionDetector::detect(const std::vector<PathPoint>& path) const
{
  if (path.empty())
  {
    return {};
  }

  std::vector<CurveRegion> regions = buildRawRegions(path);
  mergeShortRegions(regions, path);
  coalesceAdjacentSameZoneRegions(regions, path);
  mergeCloseCurveRegions(regions, path);
  coalesceAdjacentSameZoneRegions(regions, path);
  applyEntryExitMargins(regions, path.back().arc_length);
  return regions;
}

}  // namespace path_curvature_supervisor
