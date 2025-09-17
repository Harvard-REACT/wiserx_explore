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
        print(self.robot_id)
        rospy.Subscriber('/'+self.robot_name+'/range_bearing_estimates', LocalMeasurement, self.matrix_callback)

        self.root = root
        self.root.title("AOA Profile computed by Robot : " + self.robot_name)

        # Set up matplotlib figure
        self.fig, self.ax = plt.subplots()
        self.matrix = np.zeros((MATRIX_ROWS, MATRIX_COLS))
        self.img = self.ax.imshow(self.matrix, cmap='viridis', interpolation='nearest', vmin=0, vmax=1)
        self.fig.colorbar(self.img)
        self.ax.set_aspect('auto')
        x_tick_pos = np.linspace(0, MATRIX_ROWS-1, num=7)
        x_tick_labels = np.linspace(-180, 180, num=7).astype(int)
        self.ax.set_xticks(x_tick_pos)
        self.ax.set_xticklabels(x_tick_labels)
        self.ax.set_xlabel('Azimuth Angle (degree)')
        self.ax.set_ylabel('Elevation Angle (degree)')

        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self.canvas.draw()

        # GUI update loop
        self.update_gui()
        self.neighbor_id_dict = {}

    def matrix_callback(self, msg):
        for i in range(len(msg.other_robots_rb)): 
            if i not in self.neighbor_id_dict:
                self.neighbor_id_dict[i] = msg.other_robots_rb[i].robot_id
        
        rospy.loginfo("List of Neighboring Robot IDs: ")
        rospy.loginfo(self.neighbor_id_dict)
        index_key = [key for key, value in self.neighbor_id_dict.items() if value != self.robot_id]
        
        data = np.array(msg.other_robots_rb[index_key[0]].aoa_profile)
        if data.size != MATRIX_ROWS * MATRIX_COLS:
            rospy.logwarn("Received matrix size does not match expected dimensions.")
            return
        self.matrix = data.reshape((MATRIX_ROWS, MATRIX_COLS))
        rospy.loginfo("For Robot:" + self.robot_name + ", data with CSI timestamp: %f", msg.other_robots_rb[index_key[0]].csi_timestamp)
        self.img = self.ax.imshow(self.matrix.T, cmap='viridis', interpolation='nearest', vmin=np.min(self.matrix), vmax=np.max(self.matrix))
        self.ax.set_aspect('auto')


    def update_gui(self):
        self.img.set_data(self.matrix.T)
        self.canvas.draw()
        self.root.after(3000, self.update_gui)  # Update every 3 sec

if __name__ == '__main__':
    try:
        root = tk.Tk()
        viewer = MatrixViewer(root)
        tk.mainloop()
    except rospy.ROSInterruptException:
        pass
