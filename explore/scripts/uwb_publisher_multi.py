#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division  # Enforce Py3 division (3/2=1.5) and print()
import sys
import time
import re
import argparse

# Third-party dependencies
import serial
from serial import SerialException

# ROS dependencies
import rospy
from std_msgs.msg import Float64MultiArray

class UwbNode(object):  # Must inherit from object in Py2 for new-style classes
    def __init__(self, device_path, robot_id, namespace, target_indices):
        self.device_path = device_path
        self.namespace = namespace
        
        # Parse indices from string "0,1,5" -> [0, 1, 5]
        try:
            # List comprehension works in Py2
            self.target_indices = [int(x.strip()) for x in target_indices.split(',')]
        except ValueError:
            rospy.logerr("Invalid format for --indices. Using default [0, 1, 5]")
            self.target_indices = [0, 1, 5]

        # Parse Robot ID
        try:
            self.robot_id = int(robot_id)
        except ValueError:
            rospy.logwarn("Invalid robot_id '{}'. Defaulting to -1.".format(robot_id))
            self.robot_id = -1
        
        # Serial Configuration
        self.serial_conn = None
        self.baudrate = 115200
        self.is_connected = False
        
        # ROS Initialization
        rospy.init_node('uwb_node', anonymous=True)
        
        # String formatting replacement for f-string
        if self.namespace:
            topic_name = "{}/distance_multi".format(self.namespace)
        else:
            topic_name = "distance_multi"
            
        self.publisher = rospy.Publisher(topic_name, Float64MultiArray, queue_size=10)
        self.rate = rospy.Rate(5)  # 5Hz

        rospy.loginfo("UWB Node Initialized. Tracking Indices: {} | Masking Robot ID: {}".format(
            self.target_indices, self.robot_id))

        # Initial connection attempt
        self.connect_serial()

    def connect_serial(self):
        """Attempts to establish a serial connection with the sensor."""
        while not rospy.is_shutdown() and not self.is_connected:
            try:
                self.serial_conn = serial.Serial(
                    self.device_path,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    stopbits=serial.STOPBITS_ONE,
                    parity=serial.PARITY_NONE,
                    dsrdtr=False,
                    timeout=1.0 
                )
                rospy.loginfo("*** Connected to UWB Sensor at {} ***".format(self.device_path))
                self.is_connected = True
                self.clean_buffer()
            except SerialException as e:
                rospy.logwarn("Connection failed: {}. Retrying in 1s...".format(e))
                time.sleep(1.0)

    def clean_buffer(self):
        """Flushes the input/output buffers to remove stale data."""
        try:
            if self.serial_conn:
                self.serial_conn.flushInput()  # PySerial 2.7/Py2 legacy name (reset_input_buffer is Py3)
                self.serial_conn.flushOutput() # PySerial 2.7/Py2 legacy name
        except SerialException:
            self.is_connected = False

    def parse_data(self, raw_data):
        """
        Dynamically parses columns based on self.target_indices.
        """
        # Create a list of empty lists, one for each target index
        vals = [[] for _ in range(len(self.target_indices))]
        
        # Decode handling: In Py2, read() returns str (bytes), but decode is safer for unicode compliance
        try:
            decoded_data = raw_data.decode("utf-8", "ignore")
        except Exception:
            decoded_data = raw_data

        for line in decoded_data.split("\r\n"):
            if not line.strip():
                continue
            
            print(line)
            tokens = re.split(' ', line)
            
            # Iterate through the user-specified indices
            for i, target_idx in enumerate(self.target_indices):
                # Safety check: ensure the line has enough tokens
                if target_idx >= len(tokens):
                    continue

                try:
                    val = float(tokens[target_idx])

                    # --- SCALABLE MASKING LOGIC ---
                    # If robot_id is 1, we mask the 1st index.
                    if self.robot_id == (i + 1):
                        val = 0.0

                    # Filter noise
                    if val >= 0.01:
                        vals[i].append(val)
                        
                except ValueError:
                    continue

        return vals

    def run(self):
        """Main loop: Triggers sensor, reads data, calculates averages, and publishes."""
        trigger_cmd = b'T' # Byte string literal is valid in both Py2 and Py3

        while not rospy.is_shutdown():
            if not self.is_connected:
                self.connect_serial()
                continue

            try:
                if self.serial_conn.inWaiting() > 0: # inWaiting() is the Py2/PySerial 2.x method
                    data = self.serial_conn.read(self.serial_conn.inWaiting())
                    parsed_batches = self.parse_data(data)

                    # Check if any sub-list is not empty
                    has_data = False
                    for batch in parsed_batches:
                        if len(batch) > 0:
                            has_data = True
                            break

                    if has_data:
                        avgs = []
                        for batch in parsed_batches:
                            if len(batch) > 0:
                                # Division is safe due to 'from __future__ import division'
                                avg = sum(batch) / len(batch)
                                avgs.append(avg)
                            else:
                                avgs.append(0.0)

                        msg = Float64MultiArray(data=avgs)
                        self.publisher.publish(msg)
                        rospy.loginfo("Published: {}".format(avgs))

                self.serial_conn.write(trigger_cmd)

            except (SerialException, OSError, IOError) as e:
                rospy.logerr("Serial Error during loop: {}".format(e))
                self.is_connected = False
                if self.serial_conn:
                    self.serial_conn.close()
            
            self.rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="UWB Sensor ROS Node")
    parser.add_argument('-n', '--namespace', default='', help='ROS Namespace')
    parser.add_argument('-id', '--robot_id', default='-1', help='Robot ID (Matches order of indices)')
    parser.add_argument('-d', '--device', default='/dev/sensor_uwb', help='Serial Device Path')
    parser.add_argument('--indices', default='0,1,5', help='Comma separated list of data indices (e.g. "0,1,5")')
    
    args, unknown = parser.parse_known_args()

    try:
        node = UwbNode(
            device_path=args.device, 
            robot_id=args.robot_id, 
            namespace=args.namespace,
            target_indices=args.indices
        )
        node.run()
    except rospy.ROSInterruptException:
        pass
