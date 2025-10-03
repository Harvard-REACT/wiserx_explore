#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import time
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseArray
from matplotlib import gridspec
from wiserx_explore_lite.msg import QuadmapViz

class Point:
    """A point located at (x,y) in 2D space.

    Each Point object may be associated with a payload object.

    """
    def __init__(self, x, y, payload=None):
        self.x, self.y = x, y
        self.payload = payload

    def __repr__(self):
        return '{}: {}'.format(str((self.x, self.y)), repr(self.payload))
    def __str__(self):
        return 'P({:.2f}, {:.2f})'.format(self.x, self.y)

    def distance_to(self, other):
        try:
            other_x, other_y = other.x, other.y
        except AttributeError:
            other_x, other_y = other
        return np.hypot(self.x - other_x, self.y - other_y)


class Rect:
    """A rectangle centred at (cx, cy) with width w and height h."""
    def __init__(self, cx, cy, w, h):
        self.cx, self.cy = cx, cy
        self.w, self.h = w, h
        self.west_edge, self.east_edge = cx - w/2, cx + w/2
        self.north_edge, self.south_edge = cy - h/2, cy + h/2

    def __repr__(self):
        return str((self.west_edge, self.east_edge, self.north_edge,
                self.south_edge))

    def __str__(self):
        return '({:.2f}, {:.2f}, {:.2f}, {:.2f})'.format(self.west_edge,
                    self.north_edge, self.east_edge, self.south_edge)

    def contains(self, point):
        """Is point (a Point object or (x,y) tuple) inside this Rect?"""
        try:
            point_x, point_y = point.x, point.y
        except AttributeError:
            point_x, point_y = point

        return (point_x >= self.west_edge and
                point_x <  self.east_edge and
                point_y >= self.north_edge and
                point_y < self.south_edge)

    def intersects(self, other):
        """Does Rect object other interesect this Rect?"""
        return not (other.west_edge > self.east_edge or
                    other.east_edge < self.west_edge or
                    other.north_edge > self.south_edge or
                    other.south_edge < self.north_edge)

    def draw(self, ax, c='k', lw=1, **kwargs):
        x1, y1 = self.west_edge, self.north_edge
        x2, y2 = self.east_edge, self.south_edge
        ax.plot([x1,x2,x2,x1,x1],[y1,y1,y2,y2,y1], c=c, lw=lw, **kwargs)



class QuadMap:
    """A class implementing a quadtree."""

    def __init__(self, boundary, max_points=4, sensor_range=10, depth=0,):
        """Initialize this node of the quadtree.

        boundary is a Rect object defining the region from which points are
        placed into this node; max_points is the maximum number of points the
        node can hold before it must divide (branch into four more nodes);
        depth keeps track of how deep into the quadtree this node lies.

        """
        if boundary.w != boundary.h: #impose constraint on dimension
            print("Error: Initialize with same dimensions of length and breadth")
            exit(1)
    
        self.boundary = boundary
        self.max_points = max_points
        self.points = []
        self.depth = depth
        # A flag to indicate whether this node has divided (branched) or not.
        self.divided = False
        self.sensor_range = sensor_range #meters
        self.map_resolution = 0.25 #meters
        self.divide()
        

    def __str__(self):
        """Return a string representation of this node, suitably formatted."""
        sp = ' ' * self.depth * 2
        s = str(self.boundary) + '\n'
        s += sp + ', '.join(str(point) for point in self.points)
        if not self.divided:
            return s
        return s + '\n' + '\n'.join([
                sp + 'nw: ' + str(self.nw), sp + 'ne: ' + str(self.ne),
                sp + 'se: ' + str(self.se), sp + 'sw: ' + str(self.sw)])

    def divide(self):
        """Divide (branch) this node by spawning four children nodes."""

        cx, cy = self.boundary.cx, self.boundary.cy
        w, h = self.boundary.w / 2, self.boundary.h / 2
        # The boundaries of the four children nodes are "northwest",
        # "northeast", "southeast" and "southwest" quadrants within the
        # boundary of the current node.
        
        # if (self.boundary.w * self.map_resolution) <= 2*self.sensor_range:
        if (self.boundary.w) <= self.sensor_range:
        # if (self.boundary.w * self.map_resolution) <= self.sensor_range:
        # if self.boundary.w <= 2*self.sensor_range: 
            return False
        else:        
            self.nw = QuadMap(Rect(cx - w/2, cy - h/2, w, h),
                                        self.max_points, self.sensor_range, self.depth + 1)
            self.ne = QuadMap(Rect(cx + w/2, cy - h/2, w, h),
                                        self.max_points, self.sensor_range, self.depth + 1)
            self.se = QuadMap(Rect(cx + w/2, cy + h/2, w, h),
                                        self.max_points, self.sensor_range, self.depth + 1)
            self.sw = QuadMap(Rect(cx - w/2, cy + h/2, w, h),
                                        self.max_points, self.sensor_range, self.depth + 1)
            self.divided = True
            self.nw.divide()
            self.ne.divide()
            self.se.divide()
            self.sw.divide()

    def draw(self, ax):
        """Draw a representation of the QuadMap on Matplotlib Axes ax."""
        self.boundary.draw(ax)
        if self.divided:
            self.nw.draw(ax)
            self.ne.draw(ax)
            self.se.draw(ax)
            self.sw.draw(ax)



class Viz:
    def __init__(self,sensor_range=0.1, map_resolution=0.1, robot_viz_color="green"):
        DPI = 75
        self.fig , self.ax = plt.subplots(figsize=(450/DPI, 450/DPI), dpi=DPI)
        self.x_data, self.y_data = np.array([]) , np.array([])
        self.x_data_true, self.y_data_true = np.array([]) , np.array([])
        self.x_data_fc, self.y_data_fc = np.array([]) , np.array([])
        self.x_data_all, self.y_data_all = np.array([]) , np.array([])
        self.ln = self.ax.scatter(self.x_data_all, self.y_data_all, s=50)
        self.own_position_x, self.own_position_y = [], []
        self.neighboring_robot_x, self.neighboring_robot_y = [], []
        
        other_robot_color = "blue"
        if(robot_viz_color == other_robot_color):
            other_robot_color = "green"

        self.colors_list = [robot_viz_color,other_robot_color, "red", "black", 'orange']
        self.colors = []
        self.colors_true = []
        self.colors_fc = []
        self.colors_all = []
        self.x_data_oshot = []
        self.y_data_oshot = []
        self.colors_oshot = []
        self.sensor_range = float(sensor_range)
        factor_val = 2.2
        self.map_resolution = float(map_resolution)
        self.width = 24 / self.map_resolution 
        self.height = self.width
        domain =  Rect(self.width/2, self.height/2, self.width, self.height)
        max_points = 4
        self.qmap = QuadMap(domain, max_points, self.sensor_range/self.map_resolution)

    def plot_init(self):
        self.ax.set_title("Robot_1 (green)", fontsize=20)
        self.ax.set_xlim(0, self.width)
        self.ax.set_ylim(0, self.height)
        self.qmap.draw(self.ax)
        return self.ln  

    def Pose_callback(self, msg):
        colors_temp = []
        colors_temp_true = []
        colors_temp_oshot = []
        robot_id_temp = []
        x_data_temp = []
        y_data_temp = []
        x_data_temp_true = []
        y_data_temp_true = []
        x_data_temp_oshot = []
        y_data_temp_oshot = []
        other_robot_cov_x_temp = []
        other_robot_cov_y_temp = []
        other_robot_status_temp = []

        #Add own details
        robot_id_temp.append(msg.robot_id)
        x_data_temp.append(msg.own_position.x)
        y_data_temp.append(msg.own_position.y)
        colors_temp.append(self.colors_list[0]) #Denotes Robot's own position
        

        for i in range(len(msg.other_robots)):
            robot_id_temp.append(msg.other_robots[i].robot_id)
            # x_data_temp_true.append(msg.other_robots[i].true_map_position.x)
            # y_data_temp_true.append(msg.other_robots[i].true_map_position.y)
            x_data_temp.append(msg.other_robots[i].estimated_map_position.x)
            y_data_temp.append(msg.other_robots[i].estimated_map_position.y)
            # x_data_temp_oshot.append(msg.other_robots[i].one_shot_map_position.x)
            # y_data_temp_oshot.append(msg.other_robots[i].one_shot_map_position.y)

            other_robot_cov_x_temp.append(msg.other_robots[i].covariance_meter_sq[0]) 
            other_robot_cov_y_temp.append(msg.other_robots[i].covariance_meter_sq[3])
            other_robot_status_temp.append(msg.other_robots[i].status)

            colors_temp.append(self.colors_list[1]) #Denotes Relative_position estimates    
            colors_temp_true.append(self.colors_list[3]) #Denotes true position of the other robot
            colors_temp_oshot.append(self.colors_list[4])
            
        
        x_array = np.array(x_data_temp)
        y_array = np.array(y_data_temp)
        colors_array = np.array(colors_temp)
        self.x_data = np.append(self.x_data, x_array)
        self.y_data = np.append(self.y_data, y_array)
        self.colors = np.append(self.colors, colors_array)  

        # x_array_true = np.array(x_data_temp_true)
        # y_array_true = np.array(y_data_temp_true)
        # colors_array_true = np.array(colors_temp_true)
        # self.x_data_true = np.append(self.x_data_true, x_array_true)
        # self.y_data_true = np.append(self.y_data_true, y_array_true)
        # self.colors_true = np.append(self.colors_true, colors_array_true)  
        
        # x_array_oshot = np.array(x_data_temp_oshot)
        # y_array_oshot = np.array(y_data_temp_oshot)
        # colors_array_oshot = np.array(colors_temp_oshot)
        # self.x_data_oshot = np.append(self.x_data_oshot, x_array_oshot)
        # self.y_data_oshot = np.append(self.y_data_oshot, y_array_oshot)
        # self.colors_oshot = np.append(self.colors_oshot, colors_array_oshot)  

        #Only keep the latest frontier centroids
        x_data_temp = []
        y_data_temp = []
        colors_temp = []
        for i in range(len(msg.frontiers)):
            x_data_temp.append(msg.frontiers[i].centroid.x)
            y_data_temp.append(msg.frontiers[i].centroid.y)
            colors_temp.append(self.colors_list[2]) #Denotes Frontier centroids
        
        x_array = np.array(x_data_temp)
        y_array = np.array(y_data_temp)
        colors_array = np.array(colors_temp)
        self.x_data_fc = x_array
        self.y_data_fc = y_array
        self.colors_fc = colors_array

        # self.x_data_all = np.append(self.x_data, self.x_data_true)
        # self.x_data_all = np.append(self.x_data_all, self.x_data_oshot)
        self.x_data_all = self.x_data
        self.x_data_all = np.append(self.x_data_all, self.x_data_fc)

        # self.y_data_all = np.append(self.y_data, self.y_data_true)
        # self.y_data_all = np.append(self.y_data_all, self.y_data_oshot)
        self.y_data_all = self.y_data
        self.y_data_all = np.append(self.y_data_all, self.y_data_fc)

        # self.colors_all = np.append(self.colors, self.colors_true)  
        # self.colors_all = np.append(self.colors_all, self.colors_oshot)  
        self.colors_all = self.colors
        self.colors_all = np.append(self.colors_all, self.colors_fc)  
  

    def update_plot(self, frame):
        # print(self.colors)
        self.ln.set_offsets(np.c_[self.x_data_all, self.y_data_all])
        self.ln.set_color(self.colors_all)
        return self.ln



if __name__ == "__main__":
    rospy.init_node('quadmap_viz_node', anonymous=True)
    robot_name = rospy.get_param('~robot_name', 'tb3')
    robot_viz_color = rospy.get_param('~viz_color', 'green')
    sensor_range= 0 
    map_resolution = 0
    
    try:
        while not rospy.is_shutdown():
            if rospy.has_param('/'+robot_name+'/slam_gmapping/maxUrange'):
                sensor_range = rospy.get_param('/'+robot_name+'/slam_gmapping/maxUrange', 0.1)
                map_resolution = rospy.get_param('/'+robot_name+'/slam_gmapping/delta', 0.1)
                rospy.loginfo(robot_name + ": Found quadmap visualization parameters.")
                break
            else:
                rospy.logwarn("Missing gmapping parameters for quadmap visualization.Waiting..")
                time.sleep(5)
            
        viz = Viz(sensor_range, map_resolution,robot_viz_color)
        sub = rospy.Subscriber("/"+robot_name+"/explore/node_list", QuadmapViz, viz.Pose_callback)
        ani = FuncAnimation(viz.fig, viz.update_plot, init_func=viz.plot_init)
        plt.show(block=True) 

    except:
        rospy.loginfo("Exiting.")

        

