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
from geometry_msgs.msg import Vector3Stamped  
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geographic_msgs.msg import GeoPointStamped

import math
import sys
import inputs

import requests
import argparse
import json
import time
import math


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
        self.declare_parameter('use_external_gps_fixed', False)
        self.declare_parameter('external_gps_fixed_lat', 0.0)
        self.declare_parameter('external_gps_fixed_lon', 0.0)       
        self.declare_parameter('use_external_gps_asv_measurements', True)
        self.declare_parameter('external_gps_asv_measurements_topic', '/usv/gps')
        self.declare_parameter('external_ned_asv_measurements_topic', '/usv/gps_ned')
        self.declare_parameter('external_map_origin_asv_measurements_topic', '/usv/map_origin')
        self.declare_parameter('use_external_heading_fixed', False)
        self.declare_parameter('external_heading_fixed_value', 0.0)
        self.declare_parameter('use_external_heading_asv_measurements', True)
        self.declare_parameter('external_imu_asv_measurements_topic', '/usv/imu/imu')
       
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
        self.USE_EXTERNAL_GPS_FIXED = self.get_parameter(
            'use_external_gps_fixed').get_parameter_value().bool_value 
        self.EXTERNAL_GPS_FIXED_LAT = self.get_parameter(
            'external_gps_fixed_lat').get_parameter_value().double_value    
        self.EXTERNAL_GPS_FIXED_LON = self.get_parameter(
            'external_gps_fixed_lon').get_parameter_value().double_value      
        self.USE_EXTERNAL_GPS_MEASUREMENTS = self.get_parameter(
            'use_external_gps_asv_measurements').get_parameter_value().bool_value 
        self.EXTERNAL_GPS_MEASUREMENTS_TOPIC = self.get_parameter(
            'external_gps_asv_measurements_topic').get_parameter_value().string_value 
        self.EXTERNAL_NED_MEASUREMENTS_TOPIC = self.get_parameter(
            'external_ned_asv_measurements_topic').get_parameter_value().string_value 
        self.EXTERNAL_MAP_ORIGIN_MEASUREMENTS_TOPIC = self.get_parameter(
            'external_map_origin_asv_measurements_topic').get_parameter_value().string_value        
        self.USE_EXTERNAL_HEADING_FIXED = self.get_parameter(
            'use_external_heading_fixed').get_parameter_value().bool_value 
        self.EXTERNAL_HEADING_FIXED_VALUE = self.get_parameter(
            'external_heading_fixed_value').get_parameter_value().double_value         
        self.USE_EXTERNAL_HEADING_MEASUREMENTS = self.get_parameter(
            'use_external_heading_asv_measurements').get_parameter_value().bool_value 
        self.EXTERNAL_IMU_MEASUREMENTS_TOPIC = self.get_parameter(
            'external_imu_asv_measurements_topic').get_parameter_value().string_value     
        
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
            msg = Vector3Stamped()
            time_ = time.time()
            time_nanosec, time_sec  = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec) 
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.vector.x = float(self.locator_wrt_base_relative_x)
            msg.vector.y = float(self.locator_wrt_base_relative_y)
            msg.vector.z = float(self.locator_wrt_base_relative_z)
            """msg.position.x = float(self.locator_wrt_base_relative_x)
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
            msg.orientation.w = float(w)"""
            self.pose_pub.publish(msg)
        
        if hasattr(self, 'locator_global_lat'):
            msg = GeoPointStamped()
            time_ = time.time()
            time_nanosec, time_sec  = math.modf(time_)
            time_sec = int(time_sec)
            time_nanosec = int(1e9*time_nanosec) #+2**32
            msg.header.stamp.sec = time_sec
            msg.header.stamp.nanosec = time_nanosec
            msg.position.latitude = float(self.locator_global_lat)
            msg.position.longitude = float(self.locator_global_lon)
            msg.position.altitude = 0.0; # or -self.locator_wrt_base_relative_z 
            self.gps_pub.publish(msg)
    

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
        
    def external_gps_measurements_callback(self, msg):
        return 0
    
    def external_ned_measurements_callback(self, msg):
        return 0
    
    def external_map_origin_measurements_callback(self, msg):
        return 0
    
    def external_imu_measurements_callback(self, msg):
        return 0

    def initialize_subscribers(self):
        print("Initializing ROS subscribers")
        if (self.USE_EXTERNAL_GPS_FIXED ^ self.USE_EXTERNAL_GPS_MEASUREMENTS):
            if self.USE_EXTERNAL_GPS_MEASUREMENTS:
                self.create_subscription(
            NavSatFix, self.EXTERNAL_GPS_MEASUREMENTS_TOPIC, self.external_gps_measurements_callback, 10)
                self.create_subscription(
            Vector3Stamped, self.EXTERNAL_NED_MEASUREMENTS_TOPIC, self.external_ned_measurements_callback, 10)
                self.create_subscription(
            GeoPointStamped, self.EXTERNAL_MAP_ORIGIN_MEASUREMENTS_TOPIC, self.external_map_origin_measurements_callback, 10)
        else: 
            rclpy.logging.ERROR("USE_EXTERNAL_GPS_FIXED and USE_EXTERNAL_GPS_MEASUREMENTS must not have the same value!")
        
        if (self.USE_EXTERNAL_HEADING_FIXED ^ self.USE_EXTERNAL_HEADING_MEASUREMENTS):            
            if self.USE_EXTERNAL_HEADING_MEASUREMENTS:
                self.create_subscription(
            Imu, self.EXTERNAL_IMU_MEASUREMENTS_TOPIC, self.external_imu_measurements_callback, 10)
        else: 
            rclpy.logging.ERROR("USE_EXTERNAL_HEADING_FIXED and USE_EXTERNAL_HEADING_ASV_MEASUREMENTS must not have the same value!")
        
    def initialize_publishers(self):
        print("Initializing ROS publishers")
        self.pose_pub = self.create_publisher(Vector3Stamped, "waterlinked_locator_position_relative", 10)
        self.gps_pub = self.create_publisher(GeoPointStamped, "waterlinked_locator_position_global", 10)
                
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

