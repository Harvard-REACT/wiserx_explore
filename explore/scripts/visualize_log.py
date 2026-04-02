import re
import matplotlib.pyplot as plt
import numpy as np
import os
import argparse
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# --- Configuration ---
# LOG_FILE_PATH = "/home/react-ws-1/catkin_ws/src/m-explore/explore/data/data_wiserx/shellspace_trial15_partial_success_twoRobots/explore_log_tb3_1_explore_2026-03-04_20-51-42.log"

# --- Data Structures ---
class NeighborData:
    def __init__(self, map_x, map_y, omega):
        self.map_x = map_x
        self.map_y = map_y
        self.omega = omega
        self.world_x = None # Will be calculated later
        self.world_y = None # Will be calculated later

class ViewpointData:
    def __init__(self, x, y, info_gain, utility, info_loss=0.0):
        self.x = x
        self.y = y
        self.info_gain = info_gain
        self.utility = utility
        self.info_loss = info_loss
        self.neighbors = []

class FrontierGroup:
    def __init__(self):
        self.viewpoints = []
        self.max_info_gain = 0.0

class TimestepData:
    def __init__(self, robot_world_x, robot_world_y, robot_heading, timestamp):
        self.robot_world_x = robot_world_x
        self.robot_world_y = robot_world_y
        self.robot_heading = robot_heading
        self.frontier_groups = []
        self.total_cells = 0
        self.filled_cells = 0
        self.fill_percentage = 0.0
        self.timestamp = timestamp
        self.soft_threshold_reached = False

# --- Global variables for map conversion (initialized during parsing) ---
map_resolution = None
map_origin_x = None
map_origin_y = None

# --- Parsing Functions ---
def parse_log(log_file_path):
    global map_resolution, map_origin_x, map_origin_y

    with open(log_file_path, 'r') as f:
        lines = f.readlines()

    # --- Pre-scan for map parameters ---
    re_map_res = re.compile(r"map_resolution__ (\d+\.\d+)")
    re_own_pos_world = re.compile(r"Own position \(world\): (\-?\d+\.\d+), (\-?\d+\.\d+)")
    re_own_pos_map = re.compile(r"Own position \(map\): (\d+), (\d+)")

    # Find map resolution first
    for line in lines:
        match_res = re_map_res.search(line)
        if match_res:
            map_resolution = float(match_res.group(1))
            print(f"Detected map_resolution: {map_resolution}")
            break
    
    if map_resolution is None:
        print("Error: Could not find 'map_resolution__' in log file. Cannot proceed.")
        return []

    # Now find the first pair of world/map coordinates to calculate origin
    for i, line in enumerate(lines):
        match_own_world = re_own_pos_world.search(line)
        if match_own_world:
            # Look for the map coordinates on the next line
            if i + 1 < len(lines):
                match_own_map = re_own_pos_map.search(lines[i+1])
                if match_own_map:
                    world_x = float(match_own_world.group(1))
                    world_y = float(match_own_world.group(2))
                    map_x = int(match_own_map.group(1))
                    map_y = int(match_own_map.group(2))
                    
                    map_origin_x = world_x - map_x * map_resolution
                    map_origin_y = world_y - map_y * map_resolution
                    print(f"Calculated map_origin_x: {map_origin_x}, map_origin_y: {map_origin_y}")
                    break # Found it, exit the loop
    
    if map_origin_x is None:
        print("Error: Could not find corresponding 'Own position (world)' and 'Own position (map)' logs. Cannot calculate map origin.")
        return []

    # --- Main parsing loop ---
    timesteps_data = []
    current_timestep = None
    current_frontier_group = None
    
    # Regex patterns for main loop
    re_timestamp = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})")
    re_planning_pose_heading_only = re.compile(r"Own pose when relative measurements obtained \(x, y, heading\): \-?\d+\.\d+, \-?\d+\.\d+, (\-?\d+\.\d+)")
    re_new_make_plan = re.compile(r"REACHED 3/4th to goal ---  REVAULATING ALL FRONTIERS")
    re_new_frontier = re.compile(r"##############################################################")
    re_frontier_viewpoint_world_pos = re.compile(r"Frontier viewpoint world pos: (\-?\d+\.\d+), (\-?\d+\.\d+)")
    re_info_gain = re.compile(r"Info gain = (\d+\.\d+)")
    re_info_loss = re.compile(r"Info loss at the frontier \(percent\) = (\d+\.\d+)")
    re_neighbor_pos_omega = re.compile(r"Neighbor \(est_mx,est_my\): \((\d+),(\d+)\), Omega\(uncertainty trace\): (\d+\.\d+)")
    re_max_info_gain = re.compile(r"======== Max Info gain around frontier: (\d+\.\d+) ========")
    re_sorted_frontiers_header = re.compile(r"===============Sorted frontiers===================")
    re_sorted_frontier_utility = re.compile(r"frontier \d+ utility: (\-?\d+\.\d+)")
    re_sorted_frontier_position = re.compile(r"frontier \d+ position: \(\s*(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\s*\)")
    re_total_cells = re.compile(r"Total cells = (\d+\.\d+)")
    re_filled_cells = re.compile(r"Filled cells = (\d+\.\d+)")
    re_fill_percentage = re.compile(r"Estimated map fill percentage = (\d+\.\d+)")
    re_soft_threshold = re.compile(r"Exploration Termination condition satisfied - Soft Threshold")

    for i, line in enumerate(lines):
        # Detect start of a new planning cycle
        if re_new_make_plan.search(line):
            robot_world_x, robot_world_y, robot_heading = None, None, 0.0

            # Scan for position from "Own position (world): ..."
            for j in range(i, max(0, i-50), -1): # Look back a few lines
                match_own_world = re_own_pos_world.search(lines[j])
                if match_own_world:
                    robot_world_x = float(match_own_world.group(1))
                    robot_world_y = float(match_own_world.group(2))
                    break

            # Separately scan for heading from "Own pose when relative measurements obtained..."
            for j in range(i, max(0, i-50), -1): # Look back up to 50 lines for the pose
                match_heading = re_planning_pose_heading_only.search(lines[j])
                if match_heading:
                    heading_deg = float(match_heading.group(1))
                    robot_heading = np.deg2rad(heading_deg) # Convert to radians
                    break
            
            if robot_world_x is not None:
                timestamp = "Unknown"
                match_ts = re_timestamp.search(line)
                if match_ts:
                    timestamp = match_ts.group(1)
                current_timestep = TimestepData(robot_world_x, robot_world_y, robot_heading, timestamp)
                timesteps_data.append(current_timestep)
                current_frontier_group = None # Reset
            else:
                print(f"Warning: Could not find 'Own position (world):' log for a new timestep near line {i+1}. Skipping this timestep.")
                current_timestep = None # Skip this timestep if robot pos not found

        if current_timestep:
            # Detect start of a new frontier block
            if re_new_frontier.search(line):
                current_frontier_group = FrontierGroup()
                current_timestep.frontier_groups.append(current_frontier_group)

            # Detect max info gain line, which signals the end of a frontier block
            if re_max_info_gain.search(line):
                if current_frontier_group: # Should not be None here
                    match = re_max_info_gain.search(line)
                    current_frontier_group.max_info_gain = float(match.group(1))
                current_frontier_group = None # This frontier block is now complete

            # If we are inside a frontier block, parse viewpoints
            if current_frontier_group:
                match_frontier_vp = re_frontier_viewpoint_world_pos.search(line)
                if match_frontier_vp:
                    f_x = float(match_frontier_vp.group(1))
                    f_y = float(match_frontier_vp.group(2))
                    
                    info_gain = 0.0 # Default
                    info_loss = 0.0 # Default
                    neighbors_for_vp = []

                    # Look ahead for info gain, utility, and neighbors for this viewpoint
                    for j in range(i + 1, min(len(lines), i + 50)):
                        line_ahead = lines[j]
                        
                        # Stop if we hit the next viewpoint or the end of the frontier block
                        if re_frontier_viewpoint_world_pos.search(line_ahead) or re_new_frontier.search(line_ahead) or re_max_info_gain.search(line_ahead):
                            break

                        match_info_gain = re_info_gain.search(line_ahead)
                        if match_info_gain:
                            info_gain = float(match_info_gain.group(1))

                        match_info_loss = re_info_loss.search(line_ahead)
                        if match_info_loss:
                            info_loss = float(match_info_loss.group(1))

                        match_neighbor = re_neighbor_pos_omega.search(line_ahead)
                        if match_neighbor:
                            n_map_x = int(match_neighbor.group(1))
                            n_map_y = int(match_neighbor.group(2))
                            n_omega = float(match_neighbor.group(3))
                            
                            neighbor = NeighborData(n_map_x, n_map_y, n_omega)
                            # This is where the origin parameters are crucial
                            neighbor.world_x = map_origin_x + neighbor.map_x * map_resolution
                            neighbor.world_y = map_origin_y + neighbor.map_y * map_resolution
                            neighbors_for_vp.append(neighbor)

                    viewpoint = ViewpointData(f_x, f_y, info_gain, 0.0, info_loss)
                    viewpoint.neighbors = neighbors_for_vp
                    current_frontier_group.viewpoints.append(viewpoint)

            # After parsing frontier groups, look for the final sorted utilities
            if re_sorted_frontiers_header.search(line):
                # Create a map of position -> viewpoint object for easy lookup
                viewpoint_map = {}
                for group in current_timestep.frontier_groups:
                    for vp in group.viewpoints:
                        # Use a tuple of rounded coordinates as key to handle float precision
                        key = (round(vp.x, 4), round(vp.y, 4))
                        viewpoint_map[key] = vp
                
                # Now parse the block
                for j in range(i + 1, len(lines)):
                    line_ahead = lines[j]
                    
                    # Stop if we hit the next planning cycle or another major block
                    if re_new_make_plan.search(line_ahead) or "Total cells" in line_ahead:
                        break

                    match_util = re_sorted_frontier_utility.search(line_ahead)
                    if match_util:
                        if j + 1 < len(lines):
                            match_pos = re_sorted_frontier_position.search(lines[j+1])
                            if match_pos:
                                utility = float(match_util.group(1))
                                pos_x = float(match_pos.group(1))
                                pos_y = float(match_pos.group(2))
                                
                                key = (round(pos_x, 4), round(pos_y, 4))
                                if key in viewpoint_map:
                                    viewpoint_map[key].utility = utility
                                else:
                                    # Fallback search with tolerance is tricky, let's just warn
                                    print(f"Warning: Could not match sorted frontier position ({pos_x:.4f}, {pos_y:.4f}) to a viewpoint.")
            
            # Look for cell fill info
            match_total = re_total_cells.search(line)
            if match_total and current_timestep:
                current_timestep.total_cells = int(float(match_total.group(1)))

            match_filled = re_filled_cells.search(line)
            if match_filled and current_timestep:
                current_timestep.filled_cells = int(float(match_filled.group(1)))

            match_percent = re_fill_percentage.search(line)
            if match_percent and current_timestep:
                current_timestep.fill_percentage = float(match_percent.group(1))

            match_soft = re_soft_threshold.search(line)
            if match_soft and current_timestep:
                current_timestep.soft_threshold_reached = True

    return timesteps_data

def draw_combined_plots(fig, timesteps_data, timestep_idx, xlim=None, ylim=None, color_ranges=None, info_gain_bar_ylim=None, utility_bar_ylim=None, info_loss_bar_ylim=None):
    if timestep_idx >= len(timesteps_data):
        print(f"Error: Timestep index {timestep_idx} out of range.")
        return

    data = timesteps_data[timestep_idx]
    
    # 1. Use a nested gridspec for better layout control.
    gs_main = fig.add_gridspec(1, 2, width_ratios=[1.5, 1])
    gs_left = gs_main[0].subgridspec(2, 1, height_ratios=[3, 1.2])
    gs_right = gs_main[1].subgridspec(3, 1)

    ax_info_map = fig.add_subplot(gs_left[0])
    ax_omega_cdf = fig.add_subplot(gs_left[1])
    ax_info_bar = fig.add_subplot(gs_right[0])
    ax_loss_bar = fig.add_subplot(gs_right[1])
    ax_util_bar = fig.add_subplot(gs_right[2])

    # --- Common Data Preparation ---
    robot_x_rot, robot_y_rot = -data.robot_world_y, data.robot_world_x
    robot_heading_rad = data.robot_heading

    
    # --- 1. Information Gain Map Plot (Top-Left) ---
    # The plot's X axis is the world's -Y, and the plot's Y is the world's X. This is a +90 degree rotation.
    plot_heading_rad = robot_heading_rad + np.pi / 2.0
    
    # Draw robot position as a square
    ax_info_map.scatter(robot_x_rot, robot_y_rot, marker='s',
                        s=150, color='blue', label='Robot Position', zorder=10)

    # Draw a short red line on top of the triangle to indicate heading
    line_length = 0.60 # meters (increased by 20%)
    line_dx = line_length * np.cos(plot_heading_rad)
    line_dy = line_length * np.sin(plot_heading_rad)
    ax_info_map.plot([robot_x_rot, robot_x_rot + line_dx], [robot_y_rot, robot_y_rot + line_dy], color='red', linewidth=2, zorder=11)

    all_viewpoints_info = []
    all_neighbors_for_plot = []
    for i, group in enumerate(data.frontier_groups):
        all_viewpoints_info.extend(group.viewpoints)
        if len(group.viewpoints) > 1:
            vp_xs_rot = [-vp.y for vp in group.viewpoints]
            vp_ys_rot = [vp.x for vp in group.viewpoints]
            ax_info_map.plot(vp_xs_rot, vp_ys_rot, color='gray', linestyle='--', alpha=0.5)
        if group.viewpoints:
            avg_x = np.mean([vp.x for vp in group.viewpoints])
            avg_y = np.mean([vp.y for vp in group.viewpoints])
            label_x_rot, label_y_rot = -avg_y, avg_x
            ax_info_map.text(label_x_rot + 0.1, label_y_rot + 0.1, f'F{i}', fontsize=10, color='darkred', weight='bold')
        for vp in group.viewpoints:
            all_neighbors_for_plot.extend(vp.neighbors)

    if all_viewpoints_info:
        frontier_xs_rot = [-vp.y for vp in all_viewpoints_info]
        frontier_ys_rot = [vp.x for vp in all_viewpoints_info]
        info_gains = [vp.info_gain for vp in all_viewpoints_info]
        min_gain = color_ranges['info_gain_min']
        max_gain = color_ranges['info_gain_max']
        if max_gain > min_gain:
            norm_gains = (np.array(info_gains) - min_gain) / (max_gain - min_gain)
            norm_gains = np.clip(norm_gains, 0, 1)
        else:
            norm_gains = np.ones_like(info_gains) * 0.5
        circle_sizes = 50 + norm_gains * 200
        scatter = ax_info_map.scatter(frontier_xs_rot, frontier_ys_rot, s=circle_sizes, c=info_gains, cmap='viridis', alpha=0.7, label='Frontiers (Info Gain)', vmin=min_gain, vmax=max_gain)
        # 2. Show the gradient display on the right of the information gain plot
        fig.colorbar(scatter, ax=ax_info_map, label='Information Gain', orientation='vertical', pad=0.08, aspect=30)

    omegas = []
    if all_neighbors_for_plot:
        neighbor_xs_rot = [-n.world_y for n in all_neighbors_for_plot if n.world_x is not None]
        neighbor_ys_rot = [n.world_x for n in all_neighbors_for_plot if n.world_x is not None]
        omegas = [n.omega for n in all_neighbors_for_plot if n.world_x is not None]
        if neighbor_xs_rot:
            min_omega = color_ranges['omega_min']
            max_omega = color_ranges['omega_max']
            scatter_circles = ax_info_map.scatter(neighbor_xs_rot, neighbor_ys_rot, c=omegas, cmap='Reds', marker='o', s=300, alpha=0.5, label='Neighbor Uncertainty (Omega)', zorder=4, vmin=min_omega, vmax=max_omega)
            # Keep single omega bar on the right of the info map
            fig.colorbar(scatter_circles, ax=ax_info_map, label='Neighbor Omega Value', orientation='vertical', pad=0.18, aspect=30)
            ax_info_map.scatter(neighbor_xs_rot, neighbor_ys_rot, color='black', marker='x', s=40, zorder=5)

    ax_info_map.set_xlabel("World -Y (meters)")
    ax_info_map.set_ylabel("World X (meters)")
    ax_info_map.set_title("Frontiers and Information Gain")
    # 3. Move the legend on the left of the information gain plot
    ax_info_map.legend(loc='upper right', bbox_to_anchor=(-0.15, 1), fontsize=9)
    ax_info_map.grid(True)
    if xlim: ax_info_map.set_xlim(xlim)
    if ylim: ax_info_map.set_ylim(ylim)
    ax_info_map.set_aspect('equal', adjustable='box')

    # --- Omega CDF Plot (Bottom-Left) ---
    if omegas:
        sorted_omegas = np.sort(omegas)
        y_cdf = np.arange(1, len(sorted_omegas) + 1) / len(sorted_omegas)
        ax_omega_cdf.plot(sorted_omegas, y_cdf, drawstyle='steps-post', marker='o', markersize=4, linestyle='-')
        ax_omega_cdf.set_title("CDF of Neighbor Omega Values")
        ax_omega_cdf.set_xlabel("Omega Value")
        ax_omega_cdf.set_ylabel("Cumulative Probability")
        ax_omega_cdf.grid(True)
        ax_omega_cdf.set_ylim(0, 1.05)

        # 2. Invert the x-axis
        ax_omega_cdf.invert_xaxis()

        # 1. Add vertical lines at x=0.75 (red) and x=0.25 (black)
        ax_omega_cdf.axvline(x=0.75, color='r', linestyle='--')
        ax_omega_cdf.axvline(x=0.25, color='k', linestyle='--')
        
        # 2. Update annotations for values > 0.75 and < 0.25
        total_count = len(omegas)
        count_greater_than_0_75 = np.sum(np.array(omegas) > 0.75)
        count_less_than_0_25 = np.sum(np.array(omegas) < 0.25)
        stats_text = (f"Total: {total_count}\n"
                      f"Omega > 0.75: {count_greater_than_0_75}\n"
                      f"Omega < 0.25: {count_less_than_0_25}")
        ax_omega_cdf.text(0.97, 0.95, stats_text, transform=ax_omega_cdf.transAxes, fontsize=10,
                verticalalignment='top', horizontalalignment='right',
                bbox=dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.7))

        # Set xlim based on global range if available
        if color_ranges and color_ranges.get('omega_max') is not None:
             ax_omega_cdf.set_xlim(left=color_ranges['omega_max'], right=0)
        else:
             ax_omega_cdf.set_xlim(right=0)
    else:
        ax_omega_cdf.text(0.5, 0.5, "No Neighbor Data", ha='center', va='center')
        ax_omega_cdf.set_xticks([])
        ax_omega_cdf.set_yticks([])

    # --- 4. Information Gain Bar Plot (Right Side) ---
    num_frontier_groups = len(data.frontier_groups)
    bar_width = 0.25
    group_indices = np.arange(num_frontier_groups)
    viewpoint_labels = ['Centroid', 'Left Extreme', 'Right Extreme']
    norm_info = plt.Normalize(vmin=color_ranges['info_gain_min'], vmax=color_ranges['info_gain_max'])
    cmap_info = plt.get_cmap('viridis')
    for i, group in enumerate(data.frontier_groups):
        num_viewpoints = len(group.viewpoints)
        for j, viewpoint in enumerate(group.viewpoints):
            offset = (j - (num_viewpoints - 1) / 2) * bar_width
            bar_pos = group_indices[i] + offset
            info_gain = viewpoint.info_gain
            color = cmap_info(norm_info(info_gain))
            ax_info_bar.bar(bar_pos, info_gain, width=bar_width, color=color, label=viewpoint_labels[j] if i == 0 else "", edgecolor='black')
            ax_info_bar.text(bar_pos, info_gain, f'{info_gain:.1f}', ha='center', va='bottom', fontsize=8, rotation=90, color='red')
    ax_info_bar.set_xlabel("Frontiers")
    ax_info_bar.set_ylabel("Information Gain")
    ax_info_bar.set_xticks(group_indices)
    ax_info_bar.set_xticklabels([f'F{i}' for i in range(num_frontier_groups)])
    if any(group.viewpoints for group in data.frontier_groups):
        ax_info_bar.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9)
    ax_info_bar.grid(True, axis='y')
    if info_gain_bar_ylim:
        ax_info_bar.set_ylim(0, info_gain_bar_ylim)

    # --- 5. Info Loss Bar Plot (Middle-Right) ---
    norm_loss = plt.Normalize(vmin=color_ranges['info_loss_min'], vmax=color_ranges['info_loss_max'])
    cmap_loss = plt.get_cmap('YlOrRd')
    for i, group in enumerate(data.frontier_groups):
        num_viewpoints = len(group.viewpoints)
        for j, viewpoint in enumerate(group.viewpoints):
            offset = (j - (num_viewpoints - 1) / 2) * bar_width
            bar_pos = group_indices[i] + offset
            info_loss = viewpoint.info_loss
            color = cmap_loss(norm_loss(info_loss))
            ax_loss_bar.bar(bar_pos, info_loss, width=bar_width, color=color, label=viewpoint_labels[j] if i == 0 else "", edgecolor='black')
            ax_loss_bar.text(bar_pos, info_loss, f'{info_loss:.1f}', ha='center', va='bottom', fontsize=8, rotation=90, color='red')
    ax_loss_bar.set_xlabel("Frontiers")
    ax_loss_bar.set_ylabel("Info Loss (%)")
    ax_loss_bar.set_xticks(group_indices)
    ax_loss_bar.set_xticklabels([f'F{i}' for i in range(num_frontier_groups)])
    if any(group.viewpoints for group in data.frontier_groups):
        ax_loss_bar.legend(loc='upper left', bbox_to_anchor=(1.02, 1), fontsize=9)
    ax_loss_bar.grid(True, axis='y')
    if info_loss_bar_ylim:
        ax_loss_bar.set_ylim(0, info_loss_bar_ylim)

    # --- 6. Utility Bar Plot (Bottom-Right) ---
    norm_util = plt.Normalize(vmin=color_ranges['utility_min'], vmax=color_ranges['utility_max'])
    cmap_util = plt.get_cmap('plasma')
    for i, group in enumerate(data.frontier_groups):
        chosen_vp = next((vp for vp in group.viewpoints if vp.utility > 0), None)
        if chosen_vp:
            utility = chosen_vp.utility
            color = cmap_util(norm_util(utility))
            ax_util_bar.bar(group_indices[i], utility, width=0.5, color=color, edgecolor='black')
            ax_util_bar.text(group_indices[i], utility, f'{utility:.1f}', ha='center', va='bottom', fontsize=8, color='red')
    ax_util_bar.set_xlabel("Frontiers")
    ax_util_bar.set_ylabel("Utility")
    ax_util_bar.set_xticks(group_indices)
    ax_util_bar.set_xticklabels([f'F{i}' for i in range(num_frontier_groups)])
    ax_util_bar.grid(True, axis='y')
    if utility_bar_ylim:
        ax_util_bar.set_ylim(0, utility_bar_ylim)

    # --- Final Figure Adjustments ---
    fig.suptitle(f"Timestep {timestep_idx}: Frontier Analysis", fontsize=20, weight='bold')
    fig.text(0.01, 0.98, f"Timestamp: {data.timestamp}", ha='left', va='top', fontsize=12)

    if data.total_cells > 0:
        fill_text = (f"Total Cells: {data.total_cells} | "
                     f"Filled Cells: {data.filled_cells} | "
                     f"Fill: {data.fill_percentage:.1f}%")
        
        text_color = 'black'
        if data.soft_threshold_reached:
            fill_text += " (Soft Threshold Reached)"
            text_color = 'red'

        fig.text(0.5, 0.91, fill_text, ha='center', fontsize=14, color=text_color, bbox=dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.5))
    fig.tight_layout(rect=[0.05, 0.03, 0.95, 0.89])

class OnlineVisualizer:
    def __init__(self, log_file_path):
        self.log_file_path = log_file_path
        self.root = tk.Tk()
        self.root.title(f"Online Log Visualizer - {os.path.basename(self.log_file_path)}")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.fig = plt.figure(figsize=(20, 15))
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # --- Parsing State for Online Mode ---
        self.timesteps_data = []
        self.current_timestep = None
        self.current_frontier_group = None
        self.last_line_processed = 0

        self._setup_regex()
        self._pre_scan_for_map_params()

        self.is_running = True
        self.update_plot()  # Initial plot
        self.root.mainloop()

    def on_closing(self):
        self.is_running = False
        plt.close(self.fig)
        self.root.quit()
        self.root.destroy()

    def _setup_regex(self):
        self.re_timestamp = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2})")
        self.re_planning_pose_heading_only = re.compile(r"Own pose when relative measurements obtained \(x, y, heading\): \-?\d+\.\d+, \-?\d+\.\d+, (\-?\d+\.\d+)")
        self.re_new_frontier = re.compile(r"##############################################################")
        self.re_frontier_viewpoint_world_pos = re.compile(r"Frontier viewpoint world pos: (\-?\d+\.\d+), (\-?\d+\.\d+)")
        self.re_info_gain = re.compile(r"Info gain = (\d+\.\d+)")
        self.re_info_loss = re.compile(r"Info loss at the frontier \(percent\) = (\d+\.\d+)")
        self.re_neighbor_pos_omega = re.compile(r"Neighbor \(est_mx,est_my\): \((\d+),(\d+)\), Omega\(uncertainty trace\): (\d+\.\d+)")
        self.re_max_info_gain = re.compile(r"======== Max Info gain around frontier: (\d+\.\d+) ========")
        self.re_sorted_frontiers_header = re.compile(r"===============Sorted frontiers===================")
        self.re_sorted_frontier_utility = re.compile(r"frontier \d+ utility: (\-?\d+\.\d+)")
        self.re_sorted_frontier_position = re.compile(r"frontier \d+ position: \(\s*(\-?\d+\.\d+)\s*,\s*(\-?\d+\.\d+)\s*\)")
        self.re_total_cells = re.compile(r"Total cells = (\d+\.\d+)")
        self.re_filled_cells = re.compile(r"Filled cells = (\d+\.\d+)")
        self.re_fill_percentage = re.compile(r"Estimated map fill percentage = (\d+\.\d+)")
        self.re_soft_threshold = re.compile(r"Exploration Termination condition satisfied - Soft Threshold")
        self.re_new_make_plan = re.compile(r"REACHED 3/4th to goal ---  REVAULATING ALL FRONTIERS")
        self.re_own_pos_world = re.compile(r"Own position \(world\): (\-?\d+\.\d+), (\-?\d+\.\d+)")

    def _pre_scan_for_map_params(self):
        global map_resolution, map_origin_x, map_origin_y
        try:
            with open(self.log_file_path, 'r') as f:
                lines = f.readlines()
            parse_log(self.log_file_path) # Use offline parser once to set globals
        except Exception as e:
            print(f"Failed to pre-scan for map parameters: {e}")
            self.root.destroy()
            raise

    def _parse_new_lines(self, all_lines):
        new_cycle_found = False
        for i in range(self.last_line_processed, len(all_lines)):
            line = all_lines[i]
            
            if self.re_new_make_plan.search(line):
                new_cycle_found = True
                robot_world_x, robot_world_y, robot_heading = None, None, 0.0

                # Scan for position
                for j in range(i, max(0, i-50), -1):
                    match_own_world = self.re_own_pos_world.search(all_lines[j])
                    if match_own_world:
                        robot_world_x = float(match_own_world.group(1))
                        robot_world_y = float(match_own_world.group(2))
                        break

                # Separately scan for heading from "Own pose when relative measurements obtained..."
                for j in range(i, max(0, i-50), -1):
                    match_heading = self.re_planning_pose_heading_only.search(all_lines[j])
                    if match_heading:
                        heading_deg = float(match_heading.group(1))
                        robot_heading = np.deg2rad(heading_deg)
                        break

                if robot_world_x is not None:
                    timestamp = "Unknown"
                    match_ts = self.re_timestamp.search(line)
                    if match_ts:
                        timestamp = match_ts.group(1)
                    self.current_timestep = TimestepData(robot_world_x, robot_world_y, robot_heading, timestamp)
                    self.timesteps_data.append(self.current_timestep)
                    self.current_frontier_group = None
                else:
                    self.current_timestep = None

            if self.current_timestep:
                if self.re_new_frontier.search(line):
                    self.current_frontier_group = FrontierGroup()
                    self.current_timestep.frontier_groups.append(self.current_frontier_group)
                if self.re_max_info_gain.search(line):
                    if self.current_frontier_group:
                        match = self.re_max_info_gain.search(line)
                        self.current_frontier_group.max_info_gain = float(match.group(1))
                    self.current_frontier_group = None
                if self.current_frontier_group:
                    match_frontier_vp = self.re_frontier_viewpoint_world_pos.search(line)
                    if match_frontier_vp:
                        f_x, f_y = float(match_frontier_vp.group(1)), float(match_frontier_vp.group(2))
                        info_gain, info_loss, neighbors_for_vp = 0.0, 0.0, []
                        for j in range(i + 1, min(len(all_lines), i + 50)):
                            line_ahead = all_lines[j]
                            if self.re_frontier_viewpoint_world_pos.search(line_ahead) or self.re_new_frontier.search(line_ahead) or self.re_max_info_gain.search(line_ahead):
                                break
                            match_info_gain = self.re_info_gain.search(line_ahead)
                            if match_info_gain: info_gain = float(match_info_gain.group(1))
                            match_info_loss = self.re_info_loss.search(line_ahead)
                            if match_info_loss: info_loss = float(match_info_loss.group(1))
                            match_neighbor = self.re_neighbor_pos_omega.search(line_ahead)
                            if match_neighbor:
                                n_map_x, n_map_y, n_omega = int(match_neighbor.group(1)), int(match_neighbor.group(2)), float(match_neighbor.group(3))
                                neighbor = NeighborData(n_map_x, n_map_y, n_omega)
                                neighbor.world_x = map_origin_x + neighbor.map_x * map_resolution
                                neighbor.world_y = map_origin_y + neighbor.map_y * map_resolution
                                neighbors_for_vp.append(neighbor)
                        viewpoint = ViewpointData(f_x, f_y, info_gain, 0.0, info_loss)
                        viewpoint.neighbors = neighbors_for_vp
                        self.current_frontier_group.viewpoints.append(viewpoint)
                if self.re_sorted_frontiers_header.search(line):
                    viewpoint_map = { (round(vp.x, 4), round(vp.y, 4)): vp for group in self.current_timestep.frontier_groups for vp in group.viewpoints }
                    for j in range(i + 1, len(all_lines)):
                        line_ahead = all_lines[j]
                        if self.re_new_make_plan.search(line_ahead) or "Total cells" in line_ahead: break
                        match_util = self.re_sorted_frontier_utility.search(line_ahead)
                        if match_util and j + 1 < len(all_lines):
                            match_pos = self.re_sorted_frontier_position.search(all_lines[j+1])
                            if match_pos:
                                utility, pos_x, pos_y = float(match_util.group(1)), float(match_pos.group(1)), float(match_pos.group(2))
                                key = (round(pos_x, 4), round(pos_y, 4))
                                if key in viewpoint_map: viewpoint_map[key].utility = utility
                match_total = self.re_total_cells.search(line)
                if match_total: self.current_timestep.total_cells = int(float(match_total.group(1)))
                match_filled = self.re_filled_cells.search(line)
                if match_filled: self.current_timestep.filled_cells = int(float(match_filled.group(1)))
                match_percent = self.re_fill_percentage.search(line)
                if match_percent: self.current_timestep.fill_percentage = float(match_percent.group(1))
                match_soft = self.re_soft_threshold.search(line)
                if match_soft: self.current_timestep.soft_threshold_reached = True

        self.last_line_processed = len(all_lines)
        return new_cycle_found

    def update_plot(self):
        if not self.is_running:
            return
            
        print("Updating plot...")
        
        try:
            with open(self.log_file_path, 'r') as f:
                all_lines = f.readlines()
        except IOError as e:
            print(f"Could not read log file: {e}")
            self.root.after(30000, self.update_plot)
            return

        new_cycle_found = self._parse_new_lines(all_lines)

        if not new_cycle_found and len(self.timesteps_data) > 0:
            # No new evaluation cycle detected, and we already have data displayed.
            self.root.after(30000, self.update_plot)
            return

        if not self.timesteps_data:
            print("No data parsed yet, will retry in 30 seconds.")
        else:
            # Clear the figure for redrawing
            self.fig.clear()

            # --- Recalculate global bounds based on all data so far ---
            global_color_ranges = {
                'info_gain_min': 0, 'info_gain_max': 1,
                'utility_min': 0, 'utility_max': 1,
                'omega_min': 0, 'omega_max': 1,
                'info_loss_min': 0, 'info_loss_max': 1
            }
            all_gains = [vp.info_gain for ts in self.timesteps_data for group in ts.frontier_groups for vp in group.viewpoints]
            all_utils = [vp.utility for ts in self.timesteps_data for group in ts.frontier_groups for vp in group.viewpoints if vp.utility > 0]
            all_omegas = [n.omega for ts in self.timesteps_data for group in ts.frontier_groups for vp in group.viewpoints for n in vp.neighbors]
            all_losses = [vp.info_loss for ts in self.timesteps_data for group in ts.frontier_groups for vp in group.viewpoints]

            def get_buffered_range(values, buffer_percent=0.2):
                if not values: return 0, 1
                min_val, max_val = float(min(values)), float(max(values))
                if min_val == max_val: return min_val - 0.5, max_val + 0.5
                range_val = max_val - min_val
                buffer = range_val * (buffer_percent / 2.0)
                return min_val - buffer, max_val + buffer

            if all_gains: global_color_ranges['info_gain_min'], global_color_ranges['info_gain_max'] = get_buffered_range(all_gains)
            if all_utils: global_color_ranges['utility_min'], global_color_ranges['utility_max'] = get_buffered_range(all_utils)
            if all_omegas: global_color_ranges['omega_min'], global_color_ranges['omega_max'] = get_buffered_range(all_omegas)
            if all_losses: global_color_ranges['info_loss_min'], global_color_ranges['info_loss_max'] = get_buffered_range(all_losses)
            
            global_max_info_gain_bar_y = max(all_gains) * 1.1 if all_gains else None
            global_max_utility_bar_y = max(all_utils) * 1.1 if all_utils else None
            global_max_info_loss_bar_y = max(all_losses) * 1.1 if all_losses else None

            all_x_rot, all_y_rot = [], []
            for ts_data in self.timesteps_data:
                all_x_rot.append(-ts_data.robot_world_y)
                all_y_rot.append(ts_data.robot_world_x)
                for group in ts_data.frontier_groups:
                    for vp in group.viewpoints:
                        all_x_rot.append(-vp.y)
                        all_y_rot.append(vp.x)
                        for n in vp.neighbors:
                            if n.world_x is not None:
                                all_x_rot.append(-n.world_y)
                                all_y_rot.append(n.world_x)
            
            plot_xlim, plot_ylim = None, None
            if all_x_rot:
                min_x, max_x = min(all_x_rot), max(all_x_rot)
                x_range = max_x - min_x
                x_padding = x_range * 0.1 if x_range > 0 else 1.0
                plot_xlim = (min_x - x_padding, max_x + x_padding)

                min_y, max_y = min(all_y_rot), max(all_y_rot)
                y_range = max_y - min_y
                y_padding = y_range * 0.1 if y_range > 0 else 1.0
                plot_ylim = (min_y - y_padding, max_y + y_padding)

            # We only visualize the LATEST timestep in online mode
            latest_timestep_idx = len(self.timesteps_data) - 1
            
            print(f"  -> Visualizing latest timestep {latest_timestep_idx}...")
            draw_combined_plots(self.fig, self.timesteps_data, latest_timestep_idx, plot_xlim, plot_ylim, global_color_ranges, global_max_info_gain_bar_y, global_max_utility_bar_y, global_max_info_loss_bar_y)

            self.canvas.draw()

        # Schedule the next update
        self.root.after(30000, self.update_plot) # 30000 ms = 30 seconds

# --- Main Execution ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse and visualize exploration log files.")
    parser.add_argument("log_file", help="Path to the exploration log file.")
    parser.add_argument("--online", action="store_true", help="Run in online mode with a GUI, periodically updating from the log file.")
    args = parser.parse_args()

    if args.online:
        OnlineVisualizer(args.log_file)
    else:
        # --- Offline Mode ---
        log_file_path_input = args.log_file
        if not os.path.isfile(log_file_path_input):
            print(f"Error: The path '{log_file_path_input}' is not a valid file. Exiting.")
            exit()

        log_dir = os.path.dirname(log_file_path_input)
        log_basename = os.path.basename(log_file_path_input)
        base_output_dir_name = f"plots_{os.path.splitext(log_basename)[0]}"
        base_output_dir = os.path.join(log_dir, base_output_dir_name)
        os.makedirs(base_output_dir, exist_ok=True)
        print(f"Plots will be saved to: {base_output_dir}")

        parsed_data = parse_log(log_file_path_input)

        if not parsed_data:
            print("No data parsed or an error occurred during parsing.")
        else:
            print(f"Successfully parsed data for {len(parsed_data)} timesteps.")
            
            global_color_ranges = {
                'info_gain_min': 0, 'info_gain_max': 1,
                'utility_min': 0, 'utility_max': 1,
                'omega_min': 0, 'omega_max': 1,
                'info_loss_min': 0, 'info_loss_max': 1
            }
            all_gains = [vp.info_gain for ts in parsed_data for group in ts.frontier_groups for vp in group.viewpoints]
            all_utils = [vp.utility for ts in parsed_data for group in ts.frontier_groups for vp in group.viewpoints if vp.utility > 0]
            all_omegas = [n.omega for ts in parsed_data for group in ts.frontier_groups for vp in group.viewpoints for n in vp.neighbors]
            all_losses = [vp.info_loss for ts in parsed_data for group in ts.frontier_groups for vp in group.viewpoints]

            def get_buffered_range(values, buffer_percent=0.2):
                if not values: return 0, 1
                min_val, max_val = float(min(values)), float(max(values))
                if min_val == max_val: return min_val - 0.5, max_val + 0.5
                range_val = max_val - min_val
                buffer = range_val * (buffer_percent / 2.0)
                return min_val - buffer, max_val + buffer

            if all_gains: global_color_ranges['info_gain_min'], global_color_ranges['info_gain_max'] = get_buffered_range(all_gains)
            if all_utils: global_color_ranges['utility_min'], global_color_ranges['utility_max'] = get_buffered_range(all_utils)
            if all_omegas: global_color_ranges['omega_min'], global_color_ranges['omega_max'] = get_buffered_range(all_omegas)
            if all_losses: global_color_ranges['info_loss_min'], global_color_ranges['info_loss_max'] = get_buffered_range(all_losses)
            print(f"Using global color ranges from all data: {global_color_ranges}")

            global_max_info_gain_bar_y = max(all_gains) * 1.1 if all_gains else None
            global_max_utility_bar_y = max(all_utils) * 1.1 if all_utils else None
            global_max_info_loss_bar_y = max(all_losses) * 1.1 if all_losses else None
            
            all_x_rot, all_y_rot = [], []
            for ts_data in parsed_data:
                all_x_rot.append(-ts_data.robot_world_y)
                all_y_rot.append(ts_data.robot_world_x)
                for group in ts_data.frontier_groups:
                    for vp in group.viewpoints:
                        all_x_rot.append(-vp.y)
                        all_y_rot.append(vp.x)
                        for n in vp.neighbors:
                            if n.world_x is not None:
                                all_x_rot.append(-n.world_y)
                                all_y_rot.append(n.world_x)

            plot_xlim, plot_ylim = None, None
            if all_x_rot:
                min_x, max_x = min(all_x_rot), max(all_x_rot)
                x_range = max_x - min_x
                x_padding = x_range * 0.1 if x_range > 0 else 1.0
                plot_xlim = (min_x - x_padding, max_x + x_padding)
                min_y, max_y = min(all_y_rot), max(all_y_rot)
                y_range = max_y - min_y
                y_padding = y_range * 0.1 if y_range > 0 else 1.0
                plot_ylim = (min_y - y_padding, max_y + y_padding)

            generated_plots_count = 0
            for i, ts_data in enumerate(parsed_data):
                if ts_data.frontier_groups:
                    print(f"  -> Generating plots for timestep {i} (frontiers found)...")
                    fig = plt.figure(figsize=(20, 15))
                    draw_combined_plots(fig, parsed_data, i, plot_xlim, plot_ylim, global_color_ranges, global_max_info_gain_bar_y, global_max_utility_bar_y, global_max_info_loss_bar_y)
                    save_path = os.path.join(base_output_dir, f"timestep_{i:04d}_combined.png")
                    fig.savefig(save_path, dpi=300)
                    plt.close(fig)
                    generated_plots_count += 1

            if generated_plots_count > 0:
                print(f"\nFinished. Generated plots for {generated_plots_count} timesteps with frontiers.")
            else:
                print("\nFinished. No timesteps with frontiers were found to generate plots for.")