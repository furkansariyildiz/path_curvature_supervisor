#ifndef PATH_CURVATURE_SUPERVISOR__CURVATURE_SMOOTHER_HPP_
#define PATH_CURVATURE_SUPERVISOR__CURVATURE_SMOOTHER_HPP_

#include <vector>

#include "path_curvature_supervisor/types.hpp"

namespace path_curvature_supervisor
{

// Smooths raw per-point curvature values to reduce sensitivity to path
// digitization noise. Stateless.
//
// Design note on signed smoothing near sign changes (S-curves): a real
// curvature signal is a continuous function of arc length that passes
// through (or near) zero at every inflection point -- averaging signed
// values near an inflection therefore reproduces genuine geometry, it does
// not "cancel out" a real curve. The failure mode this smoother cannot
// avoid is two same-magnitude, opposite-sign bends spaced closer together
// than `smoothing_distance`: in that case both peaks are under-estimated
// because each window straddles part of the neighboring bend. Keep
// `smoothing_distance` small relative to the tightest expected bend
// spacing to avoid this; `CurvatureSmoothingMode::Median` is somewhat more
// robust to this than `SignedMovingAverage` because a single dominant sign
// within the window still wins the median, but it is not immune either.
class CurvatureSmoother
{
public:
  CurvatureSmoother() = default;

  // Replaces `path[i].curvature` in place with its smoothed value, using an
  // arc-length window of +/- `smoothing_distance` meters around each point.
  // `smoothing_distance` must be > 0.
  static void smooth(std::vector<PathPoint>& path, CurvatureSmoothingMode mode,
                     double smoothing_distance);

private:
  static void smoothSignedMovingAverage(std::vector<PathPoint>& path, double smoothing_distance);
  static void smoothMedian(std::vector<PathPoint>& path, double smoothing_distance);
};

}  // namespace path_curvature_supervisor

#endif  // PATH_CURVATURE_SUPERVISOR__CURVATURE_SMOOTHER_HPP_
