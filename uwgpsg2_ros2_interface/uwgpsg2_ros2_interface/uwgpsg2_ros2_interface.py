#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

#from std_msgs.msg import Int32
#from std_msgs.msg import Float32
#from std_msgs.msg import String
#from std_msgs.msg import Bool
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import Joy
#from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Pose

import math
import sys
import inputs


import requests
import argparse
import json
#from examples.getposition import *# get_data, get_acoustic_position, get_global_position

class WaterLinedUWGPSG2Interface(Node):

    def declare_node_parameters(self):
        #self.declare_parameter('use_gamepad', False)
        #self.declare_parameter('camera_bitrate_min', 1000000)
        #self.declare_parameter('camera_resolution_values', [480, 720, 1080])
        #self.declare_parameter('velocity_force_min', -1.0)
        print("Declaring ROS parameters")
        # ROS params
        self.declare_parameter('rate', 10.0)
       
    def get_ros_params(self):
        # Setting ROS parameters
        #self.IS_SIMULATION = self.get_parameter(
        #    'is_simulation').get_parameter_value().bool_value        
        #self.CAMERA_BITRATE_MIN = self.get_parameter(
        #    'camera_bitrate_min').get_parameter_value().integer_value       
        #self.CAMERA_FRAMERATE_VALUES = self.get_parameter(
        #    'camera_framerate_values').get_parameter_value().integer_array_value
        print("Getting ROS parameters.")
        self.RATE = self.get_parameter(
            'rate').get_parameter_value().double_value  
        
    def set_ros_params(self):
        return
        
    def initialize_timer(self):
        print("Initializing timer")
        self.timer_period = 1.0/self.RATE
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback)

    def timer_callback(self):
        # self.get_ros_params() # Blueye params values should be changed
        # through topic publishing and not ROS2 node param change
        self.set_ros_params()
        self.publish_all_waterlinked_variables()
            
    def publish_all_waterlinked_variables(self):
        # Publishing ROV params and variables to ROS topics
       """ if not self.IS_SIMULATION:
            # Publishing depth and orientation into a Pose msg
            msg = Pose()
            msg.position.x = 0.0
            msg.position.y = 0.0
            # conversion of depth from [mm]to [m]
            msg.position.z = float(self.drone.depth) / 1000.0
            # Make sure the quaternion is valid and normalized
            # conversion of roll, pith and yaw from [degrees]to [rad]
            roll = math.radians(float(list(self.drone.pose.values())[0]))
            pitch = math.radians(float(list(self.drone.pose.values())[1]))
            yaw = math.radians(float(list(self.drone.pose.values())[2]))
            x, y, z, w = self.euler_to_quaternion(roll, pitch, yaw)
            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)
            msg.orientation.w = float(w)
            self.pose_pub.publish(msg)

            msg = Twist()
            msg.linear.x = float(self.drone.motion.surge)
            msg.linear.y = float(self.drone.motion.sway)
            msg.linear.z = float(self.drone.motion.heave)
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(self.drone.motion.yaw)
            self.thruster_force_norm_pub.publish(msg)            

            msg = Bool()
            msg.data = self.drone.connection_established
            self.connected_status_pub.publish(msg)           

            msg = Float32()
            msg.data = float(self.drone.motion.slow)
            self.slow_gain_pub.publish(msg)

            msg = Int32()
            msg.data = int(self.drone.config.water_density)
            self.water_density_pub.publish(msg)
            
            msg = String()
            msg.data = self.drone.software_version
            self.software_version_pub.publish(msg)

        else:
            return
            """

    def euler_to_quaternion(self, roll, pitch, yaw):  # yaw (Z), pitch (Y), roll (X)
        # Abbreviations for the various angular functions
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w

    def initialize_subscribers(self):
        # Initialize ROS subscribers to ROV variables' reference - ROV set topics
        # self.create_subscription(
        #    Twist, "thruster_force_norm_ref", self.thruster_force_norm_ref_callback, 10)
        print("Initializing ROS subscribers")
        
    def initialize_publishers(self):
        # Initialize ROS publishers of ROV variables - ROV get topics
        #self.pose_pub = self.create_publisher(Pose, "pose", 10)
        print("Initializing ROS publishers")
        
    def initialize_connection(self):
        print("Initializing connection")
        
    def __init__(self):
        print("Initializing WaterLinedUWGPSG2Interface class instance.")
        super().__init__('waterlinked_interface')
        self.declare_node_parameters()
        self.get_ros_params()
        self.initialize_timer()
        self.initialize_subscribers()
        self.initialize_publishers()
        self.initialize_connection()
        

def main(args=None):
    print("Started")
    rclpy.init(args=args)

    try:
        interface = WaterLinedUWGPSG2Interface()
        rclpy.spin(interface)
        # interface.run()
    except:
        print("Exception caught!")
        e = sys.exc_info()[0]
        write_to_page("<p>Error: %s</p>" % e)
        pass

    interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

