""" Drive the turtlebot towards the nearest object detected in its lidar scan
"""

import rclpy
from rclpy.node import Node

# Library for handling velocity data
from geometry_msgs.msg import Twist

# Library for handling lidar data
from sensor_msgs.msg import LaserScan

import numpy as np

class Driver(Node):
    def __init__(self):
        super().__init__('driver')
        # Publisher for velocities
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scans
        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            '/shaped_scan',
            self.scan_callback,
            10)
        self.scan_subscriber # prevent unused variable warning
    
        # Controller parameters
        self.max_angular_z = 2.84 # rad/s
        self.max_linear_x = 0.22 # m/s
        self.turn_p = 0.003
        self.angle_threshold = 5 # degrees
        self.dist_threshold = 0.5 # m

    def scan_callback(self, scan_msg):
        # pass
        # self.get_logger().info('Msg recieved')

        # Create velocity command for robot
        twist_msg = Twist()
        twist_msg.angular.z = self.calculate_turn(scan_msg)
        twist_msg.linear.x = self.calculate_linear(scan_msg, twist_msg.angular.z)

        # Send velocity command to robot
        self.vel_publisher.publish(twist_msg)

    def calculate_linear(self, scan_msg: LaserScan, turn_rad):
        # The poi distance from the lidar
        poi_distance = scan_msg.ranges[np.argmin(scan_msg.ranges)]

        # self.get_logger().info("distance: %s"%poi_distance)
        if poi_distance <= self.dist_threshold:
            linear_vel = 0.
        else:
            linear_vel = 0.1
        return linear_vel

    def calculate_turn(self, scan_msg: LaserScan):
        # self.get_logger().info("calculate_turn()")
        # The index of the poi in the lidar scan is the nearest range
        poi_ind = float(np.argmin(scan_msg.ranges))
        # self.get_logger().info("poi_ind: %s"%poi_ind)

        # Move poi_ind from [0,360] frame to [-180,180] frame
        poi_ind_adjusted = poi_ind - 180
        # self.get_logger().info("poi_ind_adjusted: %s"%poi_ind_adjusted)
        
        # Calculate angle difference (degrees)
        if poi_ind_adjusted >= 0:
            angle_diff = poi_ind_adjusted-180
        else:
            angle_diff = 180+poi_ind_adjusted
        # self.get_logger().info("angle_diff: %s"%angle_diff)
        
        # Threshold (degrees) for when the robot can stop the turn and just drive straight
        diff_threshold = 2

        # If we're within the threshold, then stop turning
        if abs(angle_diff) <= diff_threshold:
            turn_rad = 0.
        # Else outside the threshold. Calculate turn
        else:
            # Calculate turn
            turn_rad = angle_diff*self.turn_p

            # Bound based on max velocity
            if turn_rad > self.max_angular_z:
                turn_rad = self.max_angular_z
            elif turn_rad < -self.max_angular_z:
                turn_rad = - self.max_angular_z

        # self.get_logger().info('Angular z: %s' % turn_rad)
        return turn_rad

def main(args=None):
    rclpy.init(args=args)
    driver = Driver()

    rclpy.spin(driver)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
