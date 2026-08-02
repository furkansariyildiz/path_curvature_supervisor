#!/usr/bin/env python3
"""
Live lane/path curvature monitor for path_curvature_supervisor.

Draws the full lane from the planning trajectory, colors it by the curve
region zone classification (Straight/ModerateCurve/SharpCurve), highlights
the vehicle's current preview window, and reports the controller mode
path_curvature_supervisor currently selects.

All of the "decision" data (curve regions, selected controller, preview
metrics) is read from the real C++ library's output via ros_controllers'
/path_curvature_supervisor/* topics -- this script does not re-implement
any curvature or controller-selection logic itself.

Topics:
  /planning/motion_planning/optimized_trajectory  (autoware_planning_msgs/Trajectory)
    Full lane geometry, used only to draw the path in x/y.
  /path_curvature_supervisor/curve_regions  (autoware_debug_msgs/Float32MultiArrayStamped, latched)
    Published once per setPath() call. 5 floats per region:
    [start_arc_length, end_arc_length, zone_code, maximum_absolute_curvature,
    average_absolute_curvature]. zone_code: 0=Straight, 1=ModerateCurve, 2=SharpCurve.
  /path_curvature_supervisor/debug  (autoware_debug_msgs/Float32MultiArrayStamped)
    Published every control-loop cycle. 10 floats:
    [selected_controller_code, previous_controller_code, current_arc_length,
    preview_distance, maximum_preview_curvature, average_preview_curvature,
    predicted_lateral_acceleration, controller_changed (0/1),
    time_since_controller_change, transition_progress].
    controller_code: 0=PID, 1=Stanley, 2=PurePursuit, 3=MPC.

See ros_controllers/src/main_node.cpp (trajectoryCallback, refreshPlanningCache)
for the C++ side of this wire format.
"""

import threading

import matplotlib.pyplot as plt
import numpy as np
import rospy
from autoware_debug_msgs.msg import Float32MultiArrayStamped
from autoware_planning_msgs.msg import Trajectory
from matplotlib.animation import FuncAnimation

ZONE_NAMES = ["Straight", "ModerateCurve", "SharpCurve"]
ZONE_COLORS = ["#2ca02c", "#ff9f1c", "#d62728"]  # green, orange, red

CONTROLLER_NAMES = ["PID", "Stanley", "PurePursuit", "MPC"]


def cumulative_arc_length(xs, ys):
    arc_length = np.zeros(len(xs))
    if len(xs) > 1:
        segment_lengths = np.hypot(np.diff(xs), np.diff(ys))
        arc_length[1:] = np.cumsum(segment_lengths)
    return arc_length


def controller_name(code):
    if code is None or not (0 <= code < len(CONTROLLER_NAMES)):
        return "Unknown"
    return CONTROLLER_NAMES[code]


def zone_name(code):
    if code is None or not (0 <= code < len(ZONE_NAMES)):
        return "Unknown"
    return ZONE_NAMES[code]


class LiveLaneMonitor:
    def __init__(self):
        rospy.init_node("path_curvature_supervisor_lane_monitor", anonymous=True)

        self.data_lock = threading.Lock()

        self.path_x = np.array([])
        self.path_y = np.array([])
        self.path_arc_length = np.array([])

        self.regions = []

        self.selected_controller = None
        self.previous_controller = None
        self.current_arc_length = 0.0
        self.preview_distance = 0.0
        self.maximum_preview_curvature = 0.0
        self.average_preview_curvature = 0.0
        self.predicted_lateral_acceleration = 0.0
        self.controller_changed = False
        self.transition_progress = 1.0

        self._last_logged_controller = None

        rospy.Subscriber(
            "/planning/motion_planning/optimized_trajectory", Trajectory, self.trajectory_callback
        )
        rospy.Subscriber(
            "/path_curvature_supervisor/curve_regions",
            Float32MultiArrayStamped,
            self.regions_callback,
        )
        rospy.Subscriber(
            "/path_curvature_supervisor/debug", Float32MultiArrayStamped, self.debug_callback
        )

        self.setup_plot()

        print("\n" + "=" * 70)
        print("  PATH_CURVATURE_SUPERVISOR - LIVE LANE MONITOR")
        print("=" * 70)
        print("  Waiting for:")
        print("    /planning/motion_planning/optimized_trajectory")
        print("    /path_curvature_supervisor/curve_regions")
        print("    /path_curvature_supervisor/debug")
        print("  Close the plot window to exit")
        print("=" * 70 + "\n")

    def trajectory_callback(self, msg):
        if not msg.points:
            return
        xs = np.array([point.pose.position.x for point in msg.points])
        ys = np.array([point.pose.position.y for point in msg.points])
        with self.data_lock:
            self.path_x = xs
            self.path_y = ys
            self.path_arc_length = cumulative_arc_length(xs, ys)

    def regions_callback(self, msg):
        data = msg.data
        regions = []
        for i in range(0, len(data) - 4, 5):
            regions.append(
                {
                    "start": data[i],
                    "end": data[i + 1],
                    "zone": int(round(data[i + 2])),
                    "max_curvature": data[i + 3],
                    "avg_curvature": data[i + 4],
                }
            )
        with self.data_lock:
            self.regions = regions

        print(f"[path_curvature_supervisor] received {len(regions)} curve region(s):")
        for region in regions:
            print(
                f"    [{region['start']:7.2f} m, {region['end']:7.2f} m] "
                f"{zone_name(region['zone']):<13} "
                f"max_kappa={region['max_curvature']:.4f} avg_kappa={region['avg_curvature']:.4f}"
            )

    def debug_callback(self, msg):
        if len(msg.data) < 10:
            return

        with self.data_lock:
            self.selected_controller = int(round(msg.data[0]))
            self.previous_controller = int(round(msg.data[1]))
            self.current_arc_length = msg.data[2]
            self.preview_distance = msg.data[3]
            self.maximum_preview_curvature = msg.data[4]
            self.average_preview_curvature = msg.data[5]
            self.predicted_lateral_acceleration = msg.data[6]
            self.controller_changed = msg.data[7] > 0.5
            self.transition_progress = msg.data[9]
            current_arc_length = self.current_arc_length
            regions = self.regions

        name = controller_name(self.selected_controller)
        if name != self._last_logged_controller:
            zone = self._zone_at(current_arc_length, regions)
            print(
                f"[path_curvature_supervisor] controller -> {name} "
                f"(path zone here: {zone}, max_preview_kappa={self.maximum_preview_curvature:.4f})"
            )
            self._last_logged_controller = name

    @staticmethod
    def _zone_at(arc_length, regions):
        for region in regions:
            if region["start"] <= arc_length <= region["end"]:
                return zone_name(region["zone"])
        return "Unknown"

    def setup_plot(self):
        try:
            plt.style.use("seaborn-darkgrid")
        except (OSError, ValueError):
            plt.style.use("default")

        self.fig, (self.ax_path, self.ax_stats) = plt.subplots(
            1, 2, figsize=(14, 7), gridspec_kw={"width_ratios": [2.2, 1]}
        )
        self.fig.suptitle(
            "path_curvature_supervisor - Live Lane Monitor", fontsize=15, fontweight="bold"
        )

        self.ax_path.set_title("Lane / path (colored by curvature zone)", fontweight="bold")
        self.ax_path.set_xlabel("x [m]")
        self.ax_path.set_ylabel("y [m]")
        self.ax_path.grid(True, alpha=0.3)
        self.ax_path.set_aspect("equal", adjustable="datalim")

        self.line_path_base, = self.ax_path.plot([], [], color="#888888", linewidth=1.0, zorder=1)
        self.region_lines = []
        self.line_preview, = self.ax_path.plot(
            [], [], color="cyan", linewidth=4.0, alpha=0.6, zorder=3, label="Preview window"
        )
        self.marker_vehicle, = self.ax_path.plot(
            [], [], "o", color="black", markersize=10, zorder=4, label="Vehicle"
        )
        self.ax_path.legend(loc="upper right")

        self.ax_stats.axis("off")
        self.status_text = self.ax_stats.text(
            0.02,
            0.98,
            "",
            fontsize=11,
            family="monospace",
            va="top",
            ha="left",
            transform=self.ax_stats.transAxes,
        )

    def _rebuild_region_lines(self, path_x, path_y, path_arc_length, regions):
        for line in self.region_lines:
            line.remove()
        self.region_lines = []

        if len(path_arc_length) == 0:
            return

        for region in regions:
            start_index = int(np.searchsorted(path_arc_length, region["start"]))
            end_index = int(np.searchsorted(path_arc_length, region["end"]))
            start_index = max(0, min(start_index, len(path_x) - 1))
            end_index = max(start_index + 1, min(end_index, len(path_x) - 1))
            color = ZONE_COLORS[region["zone"]] if 0 <= region["zone"] < len(ZONE_COLORS) else "#888888"
            line, = self.ax_path.plot(
                path_x[start_index:end_index + 1],
                path_y[start_index:end_index + 1],
                color=color,
                linewidth=2.5,
                zorder=2,
            )
            self.region_lines.append(line)

    def update_plot(self, _frame):
        with self.data_lock:
            if len(self.path_x) == 0:
                return
            path_x = self.path_x
            path_y = self.path_y
            path_arc_length = self.path_arc_length
            current_arc_length = self.current_arc_length
            preview_distance = self.preview_distance
            selected_controller = self.selected_controller
            previous_controller = self.previous_controller
            max_curvature = self.maximum_preview_curvature
            avg_curvature = self.average_preview_curvature
            lateral_acceleration = self.predicted_lateral_acceleration
            controller_changed = self.controller_changed
            transition_progress = self.transition_progress
            regions = list(self.regions)

        self.line_path_base.set_data(path_x, path_y)
        self._rebuild_region_lines(path_x, path_y, path_arc_length, regions)

        start_index = int(np.searchsorted(path_arc_length, current_arc_length))
        end_index = int(np.searchsorted(path_arc_length, current_arc_length + preview_distance))
        start_index = max(0, min(start_index, len(path_x) - 1))
        end_index = max(start_index, min(end_index, len(path_x) - 1))
        self.line_preview.set_data(
            path_x[start_index:end_index + 1], path_y[start_index:end_index + 1]
        )
        self.marker_vehicle.set_data([path_x[start_index]], [path_y[start_index]])

        self.ax_path.relim()
        self.ax_path.autoscale_view()

        zone_here = self._zone_at(current_arc_length, regions)
        lines = [
            "DECISION",
            "--------",
            f"Zone under vehicle : {zone_here}",
            f"Selected controller: {controller_name(selected_controller)}",
            f"Previous controller: {controller_name(previous_controller)}",
            f"Changed this cycle : {'YES' if controller_changed else 'no'}",
            f"Transition progress: {transition_progress:5.2f}",
            "",
            "PREVIEW WINDOW",
            "--------------",
            f"Current arc length : {current_arc_length:7.2f} m",
            f"Preview distance   : {preview_distance:7.2f} m",
            f"Max preview kappa  : {max_curvature:7.4f} 1/m",
            f"Avg preview kappa  : {avg_curvature:7.4f} 1/m",
            f"Predicted a_y      : {lateral_acceleration:7.3f} m/s^2",
            "",
            "CURVE REGIONS",
            "-------------",
        ]
        for region in regions[:12]:
            lines.append(
                f"[{region['start']:6.1f},{region['end']:6.1f}] {zone_name(region['zone'])}"
            )
        if len(regions) > 12:
            lines.append(f"... and {len(regions) - 12} more")

        self.status_text.set_text("\n".join(lines))

    def run(self):
        ros_thread = threading.Thread(target=lambda: rospy.spin())
        ros_thread.daemon = True
        ros_thread.start()

        anim = FuncAnimation(
            self.fig, self.update_plot, interval=200, blit=False, cache_frame_data=False
        )
        plt.show()
        return anim


if __name__ == "__main__":
    try:
        monitor = LiveLaneMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nMonitor stopped by user")
