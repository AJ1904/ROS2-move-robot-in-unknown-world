import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class NavigationControllerNode(Node):

    def __init__(self):
        super().__init__('navigation_controller_node')

        self.get_logger().info(f"Started navigation_controller_node node")

        # subscriber for /scan
        self.laser_scan_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)

        # publisher for twists
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        

        # Declare parameters for wheel properties
        self.declare_parameter('distance', 0.0)
        self.declare_parameter('radius', 0.0)

        # Get parameter values
        self.radius = self.get_parameter('radius').value
        self.distance = self.get_parameter('distance').value
        
        # distance between obstacle and robot
        self.safe_distance = self.radius + 0.10
        
        # Constants for velocities
        self.linear_scale = 0.01
        self.angular_scale = 1

    def laser_callback(self, msg):
        # self.get_logger().info(f"Laser scan received")
        
        current_angle = msg.angle_min  

        # vector to process and aggregate information from the laser scan
        # Initialize vector components for direction
        vector_x = 0
        vector_y = 0

        # Loop through the laser scan data to process distances and angles
        for distance in msg.ranges:

            # Skip NaN values in the laser scan data
            if math.isnan(distance):
                current_angle += msg.angle_increment
                continue

            # Check distance to determine the direction of movement
            # If distance is safe, update the vectors accordingly
            if distance >= self.safe_distance:
                vector_x += distance * np.cos(current_angle)
                vector_y += distance * np.sin(current_angle)               
            else:
                # If distance is not safe, update the vectors accordingly
                vector_x -= distance * np.cos(current_angle) * 2
                vector_y -= distance * np.sin(current_angle) * 2

            # Update the current angle for the next iteration
            current_angle += msg.angle_increment

        # Generate Twist message for velocity commands
        twist_msg = Twist()
        twist_msg.angular.z = np.arctan2(vector_y, vector_x) * self.angular_scale 
        twist_msg.linear.x = np.sqrt(vector_x**2 + vector_y**2) * self.linear_scale

        self.twist_pub.publish(twist_msg)

        
def main(args=None):
    rclpy.init(args=args)
    navigation_controller_node = NavigationControllerNode()
    rclpy.spin(navigation_controller_node)
    navigation_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
