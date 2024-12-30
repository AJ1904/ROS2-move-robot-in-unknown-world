import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
import os
import tempfile
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math
import random
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
import yaml
import xml.etree.ElementTree as ET
import json
import xmltodict 
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import numpy as np
from rclpy.qos import QoSProfile, DurabilityPolicy
from project4b.disc_robot import load_disc_robot, disc_robot_urdf
import yaml
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class SimulatorNode(Node):

    def __init__(self):
        super().__init__('simulator_node')
        
        
        self.get_logger().info(f"Started simulator_node")

        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}

        # MAP MAKER FUNCTIONALITY
        # publisher to send out vr data
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        
        # Declare parameters for wheel properties
        self.declare_parameter('world_file', "")

        # Get parameter values
        self.map_file = self.get_parameter('world_file').value

        # Initialize the OccupancyGrid message
        self.map_msg = OccupancyGrid()
        self.map_msg.header = Header()
        self.map_msg.header.frame_id = "world"
        # Set the height of the map
        self.map_msg.info.origin = Pose()
        self.map_msg.info.origin.position = Point()
        self.map_msg.info.origin.orientation = Quaternion()
        
        self.two_d_map = None
        self.get_map_data()
        

	# Set the timer for the callback function
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Subscriber for vl
        self.vl_subscription = self.create_subscription(Float64, '/vl', self.vl_callback, 10)
        self.vl_subscription

        # Subscriber for vr
        self.vr_subscription = self.create_subscription(Float64, '/vr',self.vr_callback, 10)
        self.vr_subscription
        
        # Declare parameters for the robot
        self.declare_parameter('radius', 0.0)
        self.declare_parameter('distance', 0.0)
        self.declare_parameter('height', 0.0)
        self.declare_parameter('error_variance_left', 0.0)
        self.declare_parameter('error_variance_right', 0.0)
        self.declare_parameter('error_update_rate', 0.0)
        
        # Declare parameters for the laser
        self.declare_parameter('laser_rate', 1)
        self.declare_parameter('laser_count', 0)
        self.declare_parameter('laser_angle_min', 0.0)
        self.declare_parameter('laser_angle_max', 0.0)
        self.declare_parameter('laser_range_min', 0.0)
        self.declare_parameter('laser_range_max', 0.0)
        self.declare_parameter('laser_error_variance', 0.0)
        self.declare_parameter('laser_fail_probability', 0.0)

	# Get parameter values
        self.radius = self.get_parameter('radius').value
        self.distance = self.get_parameter('distance').value
        self.height = self.get_parameter('height').value
        self.error_variance_left = self.get_parameter('error_variance_left').value
        self.error_variance_right = self.get_parameter('error_variance_right').value
        self.error_update_rate = self.get_parameter('error_update_rate').value
        
        # Get laser parameter values
        self.laser_rate = self.get_parameter('laser_rate').value
        self.laser_count = self.get_parameter('laser_count').value
        self.laser_angle_min = self.get_parameter('laser_angle_min').value
        self.laser_angle_max = self.get_parameter('laser_angle_max').value
        self.laser_range_min = self.get_parameter('laser_range_min').value
        self.laser_range_max = self.get_parameter('laser_range_max').value
        self.laser_error_variance = self.get_parameter('laser_error_variance').value
        self.laser_fail_probability = self.get_parameter('laser_fail_probability').value
        
        # publisher to send out laser scan data
        self.laser_scan_publisher = self.create_publisher(LaserScan, '/scan', 10)
        # timer to periodically publish the laser scan
        self.laser_timer = self.create_timer(self.laser_rate, self.publish_laser_scan)
        # self.laser_timer = self.create_timer(10.0, self.publish_laser_scan)
        
		
        # Set up the initial transform from world to base_link
        self.base_link_transform = TransformStamped()
        self.base_link_transform.header.frame_id = 'world'
        self.base_link_transform.child_frame_id = 'base_link'
        
        # Initialize TF2 Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # initial velocities
        self.vl = 0.0
        self.vr = 0.0
        
        # Initialize error variables
        self.error_left = 0.0  # Initialize with no error
        self.error_right = 0.0  # Initialize with no error
        
        # Initialize timestamp for velocity commands and error update
        self.last_velocity_update_time = self.get_clock().now()
        self.last_error_update_time = self.get_clock().now()
        



    #GETS MAP DATA FROM THE FILE
    def get_map_data(self):
        self.get_logger().info(f"getting map data from the file")
        with open(self.map_file) as f:
            world = yaml.safe_load(f)


        resolution = world['resolution']
        self.map_msg.info.resolution = resolution  # Set the resolution of the map

        
        initial_pose = world['initial_pose']
        self.pose = {'x': initial_pose[0], 'y': initial_pose[1], 'theta': initial_pose[2]}

        map_layout = world['map']

        # self.get_logger().info('Map resolution: %s' % str(resolution))
        # self.get_logger().info('Map initial pose: %s' % str(initial_pose))
        # self.get_logger().info('Map initial pose: %s' % str(self.initial_pose))
        # self.get_logger().info('Map layout: %s' % str(map_layout))
        # self.get_logger().info('Map initial pose: %s' % str(map_layout[0][0]))

        current_row = []
        current_width = 0 # used to dynamically get the width
        current_height = 0 # used to dynamically get the height
        first_row = True

        self.current_map = []


        # A 2d map for ease of use in other functions
        self.two_d_map = []
        current_row = []
        for row in map_layout:
            
            if first_row and (str(row) != "\n"):
                current_width += 1

            if str(row) == "#":
                # found obstacle
               self.current_map.append(int(100))
               current_row.append(int(100))

               
            elif str(row) == ".":
                # found free space
                self.current_map.append(int(0))
                current_row.append(int(0))
            elif str(row) == "\n":
                # end of line
                current_height += 1
                first_row = False
                self.two_d_map.append(current_row)
                current_row = []

        self.map_msg.info.width = current_width
        self.map_msg.info.height = current_height
        self.get_logger().info(f"current_width: {current_width}, current_height: {current_height}")
        
        self.get_logger().info(f"map 2d: {self.two_d_map}")
        self.get_logger().info(f"map 1d: {self.current_map}")

        self.publish_map()

        # This timer calls publish for the map every 2 seconds
        self.world_timer = self.create_timer(2, self.publish_map)



    def publish_map(self):
        self.map_msg.data = self.current_map

        # Update the timestamp in the header
        self.map_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the map
        self.map_publisher.publish(self.map_msg)
        
                
    def generate_scan_data(self):
        scan = LaserScan()
        scan.header.frame_id = 'laser' 

        # Populate the scan data according to the specified parameters
        scan.angle_min = self.laser_angle_min
        scan.angle_max = self.laser_angle_max
        scan.angle_increment = (self.laser_angle_max - self.laser_angle_min) / self.laser_count
        scan.time_increment = 0.0
        scan.scan_time = 0.0
        scan.range_min = self.laser_range_min
        scan.range_max = self.laser_range_max

        scan.ranges = []
        
        resolution = self.map_msg.info.resolution
        
        # self.get_logger().info(f"{self.current_map}")

        # Generate random scan data based on parameters
        for angle in range(self.laser_count):
            if random.uniform(0, 1) < self.laser_fail_probability:
                # Simulate failure by appending NaN
                scan.ranges.append(float('nan'))
            else:
                current_angle = self.pose['theta'] + self.laser_angle_min + angle * scan.angle_increment
                # Initialize variables for the current direction
                min_distance = self.laser_range_max
                x = self.pose['x'] 
                y = self.pose['y'] 
                
                map_x_index = 0
                map_y_index = 0

                # Cast rays and check for obstacles in the map
                while 0 <= x < self.map_msg.info.width and 0 <= y < self.map_msg.info.height:

                    map_x_index = int(math.floor(x / resolution))
                    map_y_index = int(math.floor(y / resolution))

                    if 0 <= map_x_index < self.map_msg.info.width and 0 <= map_y_index < self.map_msg.info.height:
                        map_cell =  self.two_d_map[map_y_index][map_x_index]
                    else:
                        map_cell = 100
                    
                    
                    # Check for obstacles
                    if map_cell == 100:
                        distance = math.sqrt((x - self.pose['x'])**2 + (y - self.pose['y'])**2) - self.radius
                        if distance < min_distance:
                            min_distance = distance
                   
                    y += math.sin(current_angle) * 0.05#* (resolution/10)
                    x += math.cos(current_angle) *0.05 #* (resolution/10)
                    
                
                measured_distance = min_distance + random.gauss(0, math.sqrt(self.laser_error_variance))
                # Append the minimum distance where an obstacle is found for this angle to the scan data
                scan.ranges.append(measured_distance if (measured_distance > self.laser_range_min) and (measured_distance <= self.laser_range_max) else float('nan'))
                
        # self.get_logger().info(f"scan data: {scan}")

        return scan



        
    def publish_laser_scan(self):
        scan_data = self.generate_scan_data()
        self.laser_scan_publisher.publish(scan_data)


    #callback for when vl is received
    def vl_callback(self, msg):
        # self.get_logger().info(f"Current vl: {msg.data}")
        self.vl = msg.data
        self.last_velocity_update_time = self.get_clock().now()
        if self.error_left != 0.0:
            self.vl = self.vl * self.error_left

        

    #callback for when vl is received
    def vr_callback(self, msg):
        # self.get_logger().info(f"Current vr: {msg.data}")
        self.vr = msg.data
        self.last_velocity_update_time = self.get_clock().now()
        if self.error_right != 0.0:
            self.vr = self.vr * self.error_right

    # Timer callback function        
    def timer_callback(self):
         # Calculate elapsed time since last velocity update
         current_time = self.get_clock().now()
         elapsed_time_velocity = (current_time - self.last_velocity_update_time).nanoseconds / 1e9
         elapsed_time_error = (current_time - self.last_error_update_time).nanoseconds / 1e9
         
         # If no velocity command received for 1 second, stop the robot
         if elapsed_time_velocity > 1.0:
             self.vl = 0.0
             self.vr = 0.0
             
         # Update the errors if the error_update_rate is reached
         if elapsed_time_error >= self.error_update_rate:
             self.update_errors()
         self.update_robot_pose(elapsed_time_velocity)
             
        # Broadcast the transform
         self.broadcast_transform()


         
    def update_errors(self):
        # Update the errors for left and right wheels with random values
        self.error_left = random.gauss(1.0, self.error_variance_left)
        self.error_right = random.gauss(1.0, self.error_variance_right)
        self.last_error_update_time = self.get_clock().now()
        # self.get_logger().info(f'error_left = {self.error_left}, error_right = {self.error_right}')
         

    def update_robot_pose(self, time):
        # Update robot pose based on velocities and errors

        if(self.vl != 0.0 or self.vr != 0.0):
            # Update robot pose only if it is moving

            omega = (self.vr - self.vl) / self.distance
            omega_t = omega * 0.1
            if self.vl == self.vr:
                R = 10000000000.0 # icc at infinity
            elif self.vl == -self.vr: 
                R = 0
            else:
                R = (self.distance / 2) * (self.vr + self.vl) / (self.vr - self.vl)

            c = [self.pose['x'] - R * math.sin(self.pose['theta']), self.pose['y'] + R * math.cos(self.pose['theta'])]
            cos_omega_t = math.cos(omega_t)
            sin_omega_t = math.sin(omega_t)

            updated_x, updated_y, updated_theta = self.pose['x'], self.pose['y'], self.pose['theta']

            if self.vl == self.vr:
                # If the velocities are equal, then this is the equation we use
                updated_x =  self.pose['x'] + self.vl * math.cos(self.pose['theta'])*0.1
                updated_y =  self.pose['y'] + self.vl * math.sin(self.pose['theta'])*0.1

            else:
                # If the velocities are not equal
                updated_x = cos_omega_t * (self.pose['x'] - c[0]) - sin_omega_t * (self.pose['y'] - c[1]) + c[0]
                updated_y = sin_omega_t * (self.pose['x'] - c[0]) + cos_omega_t * (self.pose['y'] - c[1]) + c[1]
                updated_theta = self.pose['theta'] + omega_t

            if not self.is_collision(updated_x, updated_y):
                self.pose['x'] = updated_x
                self.pose['y'] = updated_y
                self.pose['theta'] = updated_theta
                # self.broadcast_transform()
            else:
                self.get_logger().info("collision detected. not updating pose")
                self.vl = 0.0
                self.vr = 0.0

            # self.get_logger().info(f"x : {self.pose['x']}, y : {self.pose['y']}, theta : {self.pose['theta']}")
       
    def broadcast_transform(self):
        # Broadcast the transform from world to base_link
	# Get the current time and set it as the stamp for the transform
        self.base_link_transform.header.stamp = self.get_clock().now().to_msg()
	    
	# Set the translation values of the transform
        self.base_link_transform.transform.translation.x = self.pose['x']
        self.base_link_transform.transform.translation.y = self.pose['y']

	# Calculate quaternion values for the rotation from the robot's theta angle
        quat_tf = [0.0, 0.0, math.sin(self.pose['theta']/2), math.cos(self.pose['theta']/2)]
        quaternion = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])

	# Set the rotation of the transform using the calculated quaternion
        self.base_link_transform.transform.rotation = quaternion

	# Send the transform using the TF2 Broadcaster
        self.tf_broadcaster.sendTransform(self.base_link_transform)



    def is_collision(self, x, y):
        #This returns true if the new x and y would yield a collision

        # self.get_logger().info("Checking for collision")

        # This looks at all the points around the robot and checks if they would collide with an obstacle
        for angle in range(360):
            new_y = y
            new_x = x

            current_rad = angle * (math.pi /180)

            new_y += math.sin(current_rad) * self.radius
            new_x += math.cos(current_rad) * self.radius

            resolution = self.map_msg.info.resolution
        
            map_x_index = int(math.floor(new_x / resolution))
            map_y_index = int(math.floor(new_y / resolution))
            if 0 <= map_x_index < self.map_msg.info.width and 0 <= map_y_index < self.map_msg.info.height:
                map_cell =  self.two_d_map[map_y_index][map_x_index]
            else:
                map_cell = 100
                
            # Check for obstacles
            if map_cell == 100:
                return True
        
        return False

        
def main(args=None):
    rclpy.init(args=args)
    simulator_node = SimulatorNode()
    rclpy.spin(simulator_node)
    simulator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
