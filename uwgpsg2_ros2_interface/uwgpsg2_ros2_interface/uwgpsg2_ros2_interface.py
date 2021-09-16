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
from geometry_msgs.msg import Pose
from geographic_msgs.msg import GeoPoint



import math
import sys
import inputs


import requests
import argparse
import json

#sys.path.append('~/waterlinked-uwgpsg2-ros2-pkg/uwgpsg2_ros2_interface/uwgpsg2_ros2_interface/examples')
#from getposition import get_data, get_acoustic_position, get_global_position
#from . import get_data, get_acoustic_position, get_global_position

class WaterLinedUWGPSG2Interface(Node):

# WaterLinked API methods
    def get_data(self, url):  
        try:
            r = requests.get(url)
        except requests.exceptions.RequestException as exc:
            print("Exception occured {}".format(exc))
            return None
            
        if r.status_code != requests.codes.ok:
            print("Got error {}: {}".format(r.status_code, r.text))
            return None
    
        return r.json()

    def get_acoustic_position(self, base_url):
        return self.get_data("{}/api/v1/position/acoustic/filtered".format(base_url))

    def get_global_position(self, base_url):
        return self.get_data("{}/api/v1/position/global".format(base_url))

# ROS-related methods
    def declare_node_parameters(self):
        #self.declare_parameter('use_gamepad', False)
        #self.declare_parameter('camera_bitrate_min', 1000000)
        #self.declare_parameter('camera_resolution_values', [480, 720, 1080])
        #self.declare_parameter('velocity_force_min', -1.0)
        print("Declaring ROS parameters")
        # ROS params
        self.declare_parameter('ros_rate', 10.0)
        self.declare_parameter('waterlinked_url', '')
       
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
            'ros_rate').get_parameter_value().double_value  
        self.WATERLINKED_URL = self.get_parameter(
            'waterlinked_url').get_parameter_value().string_value  
        
        print(self.WATERLINKED_URL)
        
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
        self.get_waterlinked_measuremets()
        self.publish_all_waterlinked_variables()
    
    def get_waterlinked_measuremets(self):
        data = self.get_acoustic_position(self.WATERLINKED_URL)
        if data:
            self.locator_wrt_base_relative_x = data["x"]
            self.locator_wrt_base_relative_y = data["y"]
            self.locator_wrt_base_relative_z = data["z"]

        pos = self.get_global_position(self.WATERLINKED_URL)
        if pos:
            self.locator_global_lat = pos["lat"]
            self.locator_global_lon = pos["lon"]
            
    def publish_all_waterlinked_variables(self):        
        if hasattr(self, 'locator_wrt_base_relative_x'):
            msg = Pose()
            msg.position.x = float(self.locator_wrt_base_relative_x)
            msg.position.y = float(self.locator_wrt_base_relative_y)
            # conversion of depth from [mm]to [m]
            msg.position.z = float(self.locator_wrt_base_relative_z)
            # Make sure the quaternion is valid and normalized
            # conversion of roll, pith and yaw from [degrees]to [rad]
            roll = 0.0
            pitch = 0.0
            yaw = 0.0
            x, y, z, w = self.euler_to_quaternion(roll, pitch, yaw)
            msg.orientation.x = float(x)
            msg.orientation.y = float(y)
            msg.orientation.z = float(z)
            msg.orientation.w = float(w)
            self.pose_pub.publish(msg)
        
        if hasattr(self, 'locator_global_lat'):
            msg = GeoPoint()
            msg.latitude = float(self.locator_global_lat)
            msg.longitude = float(self.locator_global_lon)
            msg.altitude = 0.0; # or -self.locator_wrt_base_relative_z 
            self.gps_pub.publish(msg)
    
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
        print("Initializing ROS subscribers")
        
    def initialize_publishers(self):
        print("Initializing ROS publishers")
        self.pose_pub = self.create_publisher(Pose, "waterlinked_locator_position_relative", 10)
        self.gps_pub = self.create_publisher(GeoPoint, "waterlinked_locator_position_global", 10)
                
    def __init__(self):
        print("Initializing WaterLinedUWGPSG2Interface class instance.")
        super().__init__('uwgpsg2_interface')
        self.declare_node_parameters()
        self.get_ros_params()
        self.initialize_timer()
        self.initialize_subscribers()
        self.initialize_publishers()       

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

