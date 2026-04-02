#!/usr/bin/env python3

import rospy
from wiserx_explore_lite.msg import RangeBearing, LocalMeasurement
import numpy as np
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# Parameters
MATRIX_ROWS = 360
MATRIX_COLS = 90

class MatrixViewer:
    def __init__(self, root):
        
        rospy.init_node('AOA_profile_viewer', anonymous=True)
        self.robot_name = rospy.get_param('~robot_name', 'tb3')
        self.robot_id = int(self.robot_name.split("_")[1])
        rospy.Subscriber('/'+self.robot_name+'/range_bearing_estimates', LocalMeasurement, self.matrix_callback)

        self.root = root
        self.root.title("AOA Profile computed by Robot : " + self.robot_name)

        # Layout Frames
        self.main_frame = tk.Frame(root)
        self.main_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.side_frame = tk.Frame(root, width=250)
        self.side_frame.pack(side=tk.RIGHT, fill=tk.Y)

        # Main Header
        self.id_label = tk.Label(self.main_frame, text="Profile obtained for Robot ID: -", font=("Arial", 12, "bold"))
        self.id_label.pack(side=tk.TOP, pady=5)

        # Dropdown for robot selection
        self.selected_robot_var = tk.StringVar(root)
        self.selected_robot_var.set("Select Robot")
        self.robot_selector = tk.OptionMenu(self.main_frame, self.selected_robot_var, "Select Robot")
        self.robot_selector.pack(side=tk.TOP)
        self.known_neighbor_ids = []

        # Set up matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.fig.subplots_adjust(left=0.08, right=0.98, top=0.95, bottom=0.1)
        self.matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS))
        self.img = self.ax.imshow(self.matrix.T, cmap='viridis', interpolation='nearest', vmin=0, vmax=1, aspect='auto')
        
        x_tick_pos = np.linspace(0, MATRIX_ROWS-1, num=7)
        x_tick_labels = np.linspace(-180, 180, num=7).astype(int)
        self.ax.set_xticks(x_tick_pos)
        self.ax.set_xticklabels(x_tick_labels)
        self.ax.set_xlabel('Azimuth Angle (degree)')
        self.ax.set_ylabel('Elevation Angle (degree)')

        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.main_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

        # Side plots
        self.side_plots = []
        for _ in range(3):
            f = tk.Frame(self.side_frame, bd=1, relief=tk.RIDGE)
            f.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=2, pady=2)
            l = tk.Label(f, text="Robot ID: -")
            l.pack(side=tk.TOP)
            sf, sa = plt.subplots(figsize=(3, 2))
            sf.subplots_adjust(left=0, right=1, top=1, bottom=0)
            si = sa.imshow(np.zeros((MATRIX_COLS, MATRIX_ROWS)), cmap='viridis', vmin=0, vmax=1, aspect='auto')
            sa.axis('off')
            sc = FigureCanvasTkAgg(sf, master=f)
            sc.get_tk_widget().pack(fill=tk.BOTH, expand=True)
            self.side_plots.append({'label': l, 'img': si, 'canvas': sc})

        self.latest_msg = None
        self.new_data = False

        # GUI update loop
        self.update_gui()

    def matrix_callback(self, msg):
        self.latest_msg = msg
        self.new_data = True

    def update_gui(self):
        if self.new_data and self.latest_msg:
            msg = self.latest_msg
            self.new_data = False
            
            # Extract IDs
            current_ids = []
            id_to_profile = {}
            for rb in msg.other_robots_rb:
                current_ids.append(rb.robot_id)
                id_to_profile[rb.robot_id] = rb
            
            current_ids.sort()
            
            # Update dropdown if IDs changed
            if current_ids != self.known_neighbor_ids:
                self.known_neighbor_ids = current_ids
                menu = self.robot_selector["menu"]
                menu.delete(0, "end")
                for r_id in self.known_neighbor_ids:
                    menu.add_command(label=str(r_id), command=tk._setit(self.selected_robot_var, str(r_id)))
                
                # Auto-select if nothing selected and we have robots
                if self.selected_robot_var.get() == "Select Robot" and self.known_neighbor_ids:
                     self.selected_robot_var.set(str(self.known_neighbor_ids[0]))

            # Update visualization based on selection
            selected = self.selected_robot_var.get()
            selected_id_int = -1
            if selected != "Select Robot":
                try:
                    sel_id = int(selected)
                    selected_id_int = sel_id
                    self.id_label.config(text="Profile obtained for Robot ID: " + str(sel_id))
                    if sel_id in id_to_profile:
                        rb_data = id_to_profile[sel_id]
                        data = np.array(rb_data.aoa_profile)
                        if data.size == MATRIX_ROWS * MATRIX_COLS:
                            self.matrix = data.reshape((MATRIX_ROWS, MATRIX_COLS))
                            rospy.loginfo_throttle(5, "Visualizing Robot: " + str(sel_id) + ", CSI timestamp: " + str(rb_data.csi_timestamp))
                            
                            self.img.set_data(self.matrix.T)
                            self.img.set_clim(vmin=np.min(self.matrix), vmax=np.max(self.matrix))
                            self.canvas.draw()
                        else:
                            rospy.logwarn("Received matrix size does not match expected dimensions.")
                except ValueError:
                    pass

            # Update side plots
            side_ids = [rid for rid in current_ids if rid != selected_id_int]
            for i in range(3):
                plot = self.side_plots[i]
                if i < len(side_ids):
                    rid = side_ids[i]
                    plot['label'].config(text="Robot ID: " + str(rid))
                    if rid in id_to_profile:
                        data = np.array(id_to_profile[rid].aoa_profile)
                        if data.size == MATRIX_ROWS * MATRIX_COLS:
                            mat = data.reshape((MATRIX_ROWS, MATRIX_COLS))
                            plot['img'].set_data(mat.T)
                            plot['img'].set_clim(vmin=np.min(mat), vmax=np.max(mat))
                            plot['canvas'].draw()
                else:
                    plot['label'].config(text="Robot ID: -")
                    plot['img'].set_data(np.zeros((MATRIX_COLS, MATRIX_ROWS)))
                    plot['canvas'].draw()

        self.root.after(100, self.update_gui)  # Update every 100 ms

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            root = tk.Tk()
            viewer = MatrixViewer(root)
            tk.mainloop()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exiting.")
