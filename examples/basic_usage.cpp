#include <iostream>
#include <vector>

#include "path_curvature_supervisor/path_curvature_supervisor.hpp"

int main()
{
  using namespace path_curvature_supervisor;

  SupervisorConfig config;
  config.resampling_distance = 0.25;
  config.curvature_calculation_distance = 2.0;
  config.minimum_preview_distance = 5.0;
  config.maximum_preview_distance = 30.0;
  config.preview_time = 1.5;

  PathCurvatureSupervisor supervisor(config);

  std::vector<Point2D> global_path{
      {0.0, 0.0},
      {5.0, 0.0},
      {10.0, 0.0},
      {15.0, 1.0},
      {20.0, 4.0},
      {25.0, 10.0},
  };

  supervisor.setPath(global_path);

  std::cout << "Resampled path has " << supervisor.processedPath().size() << " points\n";
  std::cout << "Detected " << supervisor.curveRegions().size() << " curve region(s)\n";
  for (const auto& region : supervisor.curveRegions())
  {
    std::cout << "  [" << region.start_arc_length << " m, " << region.end_arc_length << " m] zone="
              << toString(region.zone) << " max_curvature=" << region.maximum_absolute_curvature
              << " avg_curvature=" << region.average_absolute_curvature << '\n';
  }

  const std::size_t nearest_index = 10;
  const double vehicle_velocity = 8.0;
  const double current_time = 12.5;

  const SupervisorOutput output = supervisor.update(nearest_index, vehicle_velocity, current_time);

  std::cout << "Selected controller: " << toString(output.selected_controller) << '\n';
  std::cout << "Maximum preview curvature: " << output.maximum_preview_curvature << " 1/m\n";
  std::cout << "Predicted lateral acceleration: " << output.predicted_lateral_acceleration
            << " m/s^2\n";

  // Simulate a few more control cycles to show the dwell-time / hysteresis
  // bookkeeping in the output.
  for (int step = 1; step <= 3; ++step)
  {
    const double t = current_time + step * 0.1;
    const SupervisorOutput next_output = supervisor.update(nearest_index, vehicle_velocity, t);
    std::cout << "t=" << t << " controller=" << toString(next_output.selected_controller)
              << " changed=" << next_output.controller_changed
              << " transition_progress=" << next_output.transition_progress << '\n';
  }

  return 0;
}
