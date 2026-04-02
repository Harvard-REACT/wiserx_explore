#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid
from math import sqrt
import matplotlib.pyplot as plt
import numpy as np
import time
import csv
from std_msgs.msg import Bool, String
import random

class ExplorationEval:
    def __init__(self):
        rospy.init_node('wsr_exploration_eval', anonymous=True)
        self.exp_type = rospy.get_param('~exp_type')
        self.exploration_threshold = rospy.get_param('~explore_threshold')
        self.base_path = '/home/react-ws-1/catkin_ws/src/m-explore/explore/data/data_'+self.exp_type
        self.fail_threshold = 70
        self.robot_failed = False
        self.fail_robot = rospy.get_param('~fail_robot', False)
        self.failed_robot = rospy.get_param('~failed_robot', 'tb3_2')
        self.fail_robot_pub = rospy.Publisher('/set_failed_neighboring_robot', String, queue_size=10)
        self.coverage_of_failed_robot = 0

        self.total_iterations = 0
        self.check_itr = 0
        self.distance = []
        self.iteration = []

        # AuthorADDEED VARIABLES
        self.time_elapsed = []
        self.merged_map_coverage = []
        self.robot_1_coverage = []
        self.robot_2_coverage = []
        self.coverage_overlap_data = []

        self.x_lim = 50
        self.itr = 1

        # Flags to check if stop time has been logged
        self.logged_stop_time_1 = False
        self.logged_stop_time_2 = False
        self.logged_stop_time_3 = False

        self.got_world_map = False
        self.got_merged_map = False
        self.start = time.time()
        self.merged_map_count = 0
        self.world_map_count = 0
        self.exploration_stats = []
        self.current_distance = 0
        self.details = ['time_elapsed_sec','merged_coverage_percent','r_1_per','r1_stop_time','r_2_per','r2_stop_time','r_3_per','r3_stop_time']
        self.auto_stop_evaluation = False
       
        self.stop_evaluation_1 = False
        self.stop_evaluation_2 = False
        self.stop_evaluation_3 = False
        self.itr_1 = 0
        self.itr_2 = 0
        self.soft_threshold_evaluation_1 = False
        self.soft_threshold_evaluation_2 = False
        self.soft_threshold_evaluation_3 = False


        rospy.loginfo("Exploration coverage threshold: " +str(self.exploration_threshold))
        rospy.loginfo("Connecting to topics")
        rospy.Subscriber("/world_map", OccupancyGrid, self.world_map_cb)
        rospy.Subscriber("/map_merge/map", OccupancyGrid, self.merged_map_cb)
        # rospy.Subscriber("/map", OccupancyGrid, self.merged_map_cb)
        rospy.Subscriber("/tb3_1/map", OccupancyGrid, self.map_1_cb)
        rospy.Subscriber("/tb3_2/map", OccupancyGrid, self.map_2_cb)
        rospy.Subscriber("/tb3_3/map", OccupancyGrid, self.map_3_cb)
        
        rospy.Subscriber("/tb3_1/stop_evaluation", Bool, self.eval_cb_r1)
        rospy.Subscriber("/tb3_2/stop_evaluation", Bool, self.eval_cb_r2)
        rospy.Subscriber("/tb3_3/stop_evaluation", Bool, self.eval_cb_r3)
        
        self.map_1_count =0
        self.map_2_count =0
        self.map_3_count =0

    def world_map_cb(self, msg):
        if(not self.got_world_map):
            rospy.loginfo("Got groundtruth world map")
            rospy.loginfo("%d x %d", msg.info.height, msg.info.width)
            # self.world_ogrid = np.array(msg.data).reshape((msg.info.height, msg.info.width))
            self.world_ogrid = np.array(msg.data)
            self.merged_ogrid = np.empty(0)
            # self.ogrid_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
            # self.ogrid_cpm = 1 / msg.info.resolution
            self.got_world_map = True 

    def merged_map_cb(self, msg):
        self.merged_ogrid = np.array(msg.data)

    def map_1_cb(self, msg):
        self.map_1 = np.array(msg.data)
        self.map_1_count = np.count_nonzero(self.map_1 > -1)

    def map_2_cb(self, msg):
        self.map_2 = np.array(msg.data)
        self.map_2_count = np.count_nonzero(self.map_2 > -1)

    def map_3_cb(self, msg):
        self.map_3 = np.array(msg.data)
        self.map_3_count = np.count_nonzero(self.map_3 > -1)

    def eval_cb_r1(self, msg):
        self.stop_evaluation_1 = msg.data
        print("Got stopping flag for tb3-1")

    def eval_cb_r2(self, msg):
        self.stop_evaluation_2 = msg.data
        print("Got stopping flag for tb3-2")

    def eval_cb_r3(self, msg):
        self.stop_evaluation_3 = msg.data
        print("Got stopping flag for tb3-3")

    # def modelState_cb(self, msg):
    #     self.total_iterations +=1
        
    #     if(self.total_iterations%5000 == 0 and not rospy.is_shutdown()): #gazebo topic publishing frequency
    #         self.check_itr+=1
    #         self.iteration.append(self.check_itr)
    #         exploration_time = time.time() - self.start  

    #         self.time_elapsed.append(exploration_time)

    #         names = msg.name
    #         n_count = 0
    #         robot_id_list = []
    #         robot_positions = []
    #         for val in names:
    #             if "tb3" in val:
    #                 robot_id_list.append(n_count)
    #                 print("Got a robot")
    #             n_count+=1


    #         #DO we use average distance when testing with muliple robots?? -Yes-
    #         if len(robot_id_list) > 1:
    #             robot_1 = msg.pose[robot_id_list[0]].position
    #             robot_2 = msg.pose[robot_id_list[1]].position
    #             robot_3 = msg.pose[robot_id_list[2]].position if len(robot_id_list) > 2 else None

    #             # Calculate pairwise distances
    #             distance_12 = sqrt(pow(robot_1.x - robot_2.x, 2) + pow(robot_1.y - robot_2.y, 2))
                
    #             if robot_3:
    #                 distance_13 = sqrt(pow(robot_1.x - robot_3.x, 2) + pow(robot_1.y - robot_3.y, 2))
    #                 distance_23 = sqrt(pow(robot_2.x - robot_3.x, 2) + pow(robot_2.y - robot_3.y, 2))
                    
    #                 # Calculate average distance
    #                 self.current_distance = (distance_12 + distance_13 + distance_23) / 3
    #             else:
    #                 self.current_distance = distance_12
                
    #             # Print the current distance (for debugging purposes)
    #             # print("Current average distance = {} %".format(self.current_distance))           
    #             self.distance.append(self.current_distance)
    #         # if(len(robot_id_list) > 1):
    #         #     robot_1 = msg.pose[robot_id_list[0]].position
    #         #     robot_2 = msg.pose[robot_id_list[1]].position
    #         #     self.current_distance = sqrt(pow(robot_1.x - robot_2.x ,2) + pow(robot_1.y - robot_2.y ,2)) 
    #         #     print("Current distance = {} %".format(self.current_distance))           
    #         #     self.distance.append(self.current_distance)
            
    #         # Authoradded function
    #         if self.got_world_map: 
    #             self.world_map_count = np.count_nonzero(self.world_ogrid > -1)
    #             self.merged_map_count = np.count_nonzero(self.merged_ogrid > -1)
    #             map_coverage_percentage = self.merged_map_count * 100 / self.world_map_count
    #             r_1_per = self.map_1_count*100/self.world_map_count
    #             r_2_per = self.map_2_count*100/self.world_map_count
    #             r_3_per = self.map_3_count*100/self.world_map_count

    #             self.merged_map_coverage.append(map_coverage_percentage)
    #             self.robot_1_coverage.append(r_1_per)
    #             self.robot_2_coverage.append(r_2_per)
    #             self.robot_3_coverage.append(r_3_per)
            
    #         if self.fail_robot and not self.robot_failed and map_coverage_percentage >= self.fail_threshold:
    #             failed_robot = self.failed_robot
    #             self.fail_robot_pub.publish(String(data=failed_robot))
    #             self.robot_failed = True
    #             self.robot_failed_map_perct = map_coverage_percentage
    #             print(f"Failing {failed_robot} at {self.robot_failed_map_perct}% map coverage")
    #             ts = str(time.time()).split(".")[0]
    #             with open(self.base_path + '/failures/failed_' + ts +'.csv', 'w') as f:
    #                 write = csv.writer(f)
    #                 write.writerow(['Robot', 'Failure Coverage (%)'])
    #                 write.writerow([failed_robot, map_coverage_percentage])
        
    #     # time.sleep(5)

    def save_data(self):
        ts = str(time.time()).split(".")[0]
        op_name = 'exploration_test_'+ts+'.csv'

        with open(self.base_path + '/details_' + self.exp_type + "_" + op_name, 'w') as f: 
            write = csv.writer(f) 
            write.writerow(self.details) 
            write.writerows(self.exploration_stats) 

        with open(self.base_path + '/coverage_overlap_' + self.exp_type + "_" + op_name, 'w') as f: 
            write = csv.writer(f)
            write.writerow(['time_elapsed_sec', 'coverage_overlap_percent'])
            write.writerows(self.coverage_overlap_data)
        print("Output filenames: {}".format(self.base_path+"/"+op_name))


    def compare_maps(self):
        while not rospy.is_shutdown():
            if(self.got_world_map):
                exploration_time = time.time() - self.start
                self.world_map_count = np.count_nonzero(self.world_ogrid > -1)
                self.merged_map_count = np.count_nonzero(self.merged_ogrid > -1)
                print("world map count: ", self.world_map_count)
                print("merged map count: ", self.merged_map_count)

                # REAL TIME MAP COVERAGE CALCULATION 
                r_1_per = self.map_1_count*100/self.world_map_count
                r_2_per = self.map_2_count*100/self.world_map_count
                r_3_per = self.map_3_count*100/self.world_map_count

                map_coverage_percentage = self.merged_map_count*100/self.world_map_count
                
                if(self.fail_robot and self.robot_failed):
                    map_coverage_percentage = self.merged_map_count*100/self.world_map_count - self.coverage_of_failed_robot #We don't know the exact overlap here.
              
                
                if(map_coverage_percentage <= 0 ): 
                    time.sleep(1)
                    continue
                
                print("Current map coverage = {} %, time = {} seconds".format(map_coverage_percentage,exploration_time))
                if(map_coverage_percentage/5 > self.itr):
                    print("*****************************************")
                    print("Map covered by robot 1 = {} %".format(r_1_per))
                    print("Map covered by robot 2 = {} %".format(r_2_per))
                    print("Map covered by robot 3 = {} %".format(r_3_per))
                    print("*****************************************")
                    self.itr+=1
                
                current_data = [exploration_time,map_coverage_percentage,r_1_per,0,r_2_per,0,r_3_per,0]
                self.exploration_stats.append(current_data)                
                self.auto_stop_evaluation = self.stop_evaluation_1 and self.stop_evaluation_2 and self.stop_evaluation_3

                # COVERAGE OVERLAP
                total_robot_overlap = (r_1_per + r_2_per + r_3_per) - map_coverage_percentage
                self.coverage_overlap_data.append([exploration_time, total_robot_overlap])

                if self.stop_evaluation_1 and not self.logged_stop_time_1:
                    current_data = [exploration_time,map_coverage_percentage,r_1_per,exploration_time,r_2_per,0,r_3_per,0]
                    self.exploration_stats.append(current_data)
                    self.coverage_overlap_data.append([exploration_time, total_robot_overlap])
                    self.logged_stop_time_1 = True

                if self.stop_evaluation_2 and not self.logged_stop_time_2:
                    current_data = [exploration_time,map_coverage_percentage,r_1_per,0,r_2_per,exploration_time,r_3_per,0]
                    self.exploration_stats.append(current_data)
                    self.coverage_overlap_data.append([exploration_time, total_robot_overlap])
                    self.logged_stop_time_2 = True

                if self.stop_evaluation_3 and not self.logged_stop_time_3:
                    current_data = [exploration_time,map_coverage_percentage,r_1_per,0,r_2_per,0,r_3_per,exploration_time]
                    self.exploration_stats.append(current_data)
                    self.coverage_overlap_data.append([exploration_time, total_robot_overlap])
                    self.logged_stop_time_3 = True

                if self.fail_robot and not self.robot_failed and map_coverage_percentage >= self.fail_threshold:
                    failed_robot = self.failed_robot
                    self.fail_robot_pub.publish(String(data=failed_robot))
                    self.robot_failed = True
                    self.robot_failed_map_perct = map_coverage_percentage
                    print(f"Failing {failed_robot} at {self.robot_failed_map_perct}% map coverage")

                    # Dynamic calculation of failed robot coverage contribution
                    robot_cov = {'tb3_1': r_1_per, 'tb3_2': r_2_per, 'tb3_3': r_3_per}
                    other_robots_cov = sum([cov for name, cov in robot_cov.items() if name != failed_robot])

                    # Estimate unique contribution. Clamp to 0 to avoid negative values if overlap is high.
                    self.coverage_of_failed_robot = max(0, map_coverage_percentage - other_robots_cov)

                    r1_stop = -1 if failed_robot == 'tb3_1' else 0
                    r2_stop = -1 if failed_robot == 'tb3_2' else 0
                    r3_stop = -1 if failed_robot == 'tb3_3' else 0

                    current_data = [exploration_time,map_coverage_percentage,r_1_per,r1_stop,r_2_per,r2_stop,r_3_per,r3_stop]
                    self.exploration_stats.append(current_data)
                    self.coverage_overlap_data.append([exploration_time, total_robot_overlap])
                    
                    if failed_robot == 'tb3_1': self.stop_evaluation_1 = True
                    elif failed_robot == 'tb3_2': self.stop_evaluation_2 = True
                    elif failed_robot == 'tb3_3': self.stop_evaluation_3 = True

                    print(f"Coverage reduced by {self.coverage_of_failed_robot}%")

                if(self.auto_stop_evaluation or (map_coverage_percentage >= self.exploration_threshold)):
                    print("Stopping evaluation")
                    self.save_data()
                    rospy.signal_shutdown("Finished "+str(self.exploration_threshold)+"% exploration. Exiting")
            time.sleep(1)

def main(): 
    obj = ExplorationEval()
    obj.compare_maps()

if __name__=="__main__":
    main()
            