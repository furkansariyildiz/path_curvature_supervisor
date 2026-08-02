#ifndef PATH_CURVATURE_SUPERVISOR__CURVE_REGION_DETECTOR_HPP_
#define PATH_CURVATURE_SUPERVISOR__CURVE_REGION_DETECTOR_HPP_

#include <vector>

#include "path_curvature_supervisor/config.hpp"
#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// Classifies every path point into a CurvatureZone and groups consecutive
// same-zone points into CurveRegion segments, filtering noise-sized
// regions and merging nearby curve regions. Depends only on the config
// reference it is constructed with; holds no other state.
class CurveRegionDetector
{
public:
  explicit CurveRegionDetector(const SupervisorConfig& config);

  // `path` must already have smoothed curvature values filled in.
  std::vector<CurveRegion> detect(const std::vector<PathPoint>& path) const;

private:
  const SupervisorConfig& config_;

  CurvatureZone classify(double absolute_curvature) const;
  std::vector<CurveRegion> buildRawRegions(const std::vector<PathPoint>& path) const;
  void mergeShortRegions(std::vector<CurveRegion>& regions,
                        const std::vector<PathPoint>& path) const;
  void mergeCloseCurveRegions(std::vector<CurveRegion>& regions,
                             const std::vector<PathPoint>& path) const;
  void coalesceAdjacentSameZoneRegions(std::vector<CurveRegion>& regions,
                                      const std::vector<PathPoint>& path) const;
  void applyEntryExitMargins(std::vector<CurveRegion>& regions, double total_arc_length) const;
  void recomputeStatistics(CurveRegion& region, const std::vector<PathPoint>& path) const;
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__CURVE_REGION_DETECTOR_HPP_
