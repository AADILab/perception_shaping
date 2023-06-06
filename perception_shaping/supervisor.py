""" This runs a trained supervisor from a specified (or default) file containining the parameters of the supervisor network
"""

# Supervisor input:
# [x_robot, y_robot, heading_robot, x_poi, y_poi, x_hazzard, y_hazzard]

# Get x_robot, y_robot, heading_robot from pose in gazebo simulator
# Get x_poi, y_poi from cylinder position in gazebo simulator (maybe I can write a node that publishes the cylinder's location? Maybe hard-code it for the first pass?)
# Get x_hazzard, y_hazzard from gazebo simulator as well (same thing of publishing the position. Maybe I can put a really low cylinder as the hazzard)

import rclpy
from rclpy.node import Node

# Library for getting pose of robot
from nav_msgs.msg import Odometry

# Library for handling lidar data
from sensor_msgs.msg import LaserScan

# Numpy and scipy
import numpy as np
from scipy.spatial.transform import Rotation

from copy import deepcopy

POI_POSITION = [2,2]
HAZZARD_POSITION = [1,1]

class Supervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        # Subscriber for turtlebot pose
        self.turtlebot_pose_subscriber = self.create_subscription(
            Odometry,
            '/turtlebot_pose',
            self.turtlebot_pose_callback,
            10
        )
        self.turtlebot_pose_subscriber

        # Subsciber for lidar scan
        self.clean_scan_subscriber = self.create_subscription(
            LaserScan,
            '/clean_scan',
            self.clean_scan_callback,
            10
        )
        self.clean_scan_subscriber
    
        # Publisher for shaped lidar scan
        self.shaped_scan_publisher = self.create_publisher(
            LaserScan,
            '/shaped_scan',
            10
        )

        # Set up publisher to publish every timer_period seconds
        self.timer_period = 0.1 # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Variables for holding messages
        self.turtlebot_pose_msg = None
        self.clean_scan_msg = None


    def turtlebot_pose_callback(self, msg: Odometry):
        self.turtlebot_pose_msg = msg
    
    def clean_scan_callback(self, msg: LaserScan):
        self.clean_scan_msg = msg
    
    def extract_2d_pose(self, msg: Odometry):
        # Get position
        position_msg = msg.pose.pose.position
        
        # Grab the orientation out of the odometry message as a quaternion
        orientation_msg = msg.pose.pose.orientation
        quaternion = np.array([orientation_msg.w, orientation_msg.x, orientation_msg.y, orientation_msg.z])

        # Turn the quaternion into an angle in a 2D plane (relative to world frame) using scipy rotation library
        scipy_rotation = Rotation.from_quat(quaternion)
        euler_angles = scipy_rotation.as_euler('xyz', degrees=False)

        # Return the position and angle in 2D plane
        return position_msg.x, position_msg.y, -(euler_angles[0]-np.pi)


    def aggregate_supervisor_input(self):
        # Get the turtlebot pose as x,y,theta
        turtlebot_x, turtlebot_y, turtlebot_angle = self.extract_2d_pose(self.turtlebot_pose_msg)
        # self.get_logger().info("%s %s %s"%(turtlebot_x, turtlebot_y, turtlebot_angle))
        # self.get_logger().info("{:.2} {:.2} {:.3}".format(turtlebot_x, turtlebot_y, turtlebot_angle))

        # Get the poi position and hazzard position
        poi_x, poi_y = POI_POSITION
        hazzard_x, hazzard_y = HAZZARD_POSITION

        # Put it all together
        return np.array([turtlebot_x, turtlebot_y, turtlebot_angle, poi_x, poi_y, hazzard_x, hazzard_y])

    def compute_counterfactual(self, supervisor_input):
        # Computes the counterfactual lidar scan that we add to the actual scan to get the shaped scan

        # Until we integrate the supervisor, just return a null counterfactual (all zeros)
        # Return a list rather than a numpy array because the ranges in the message is a list
        # Should be faster to add two lists rather than convert the message list to a numpy array, add numpy arrays, then convert back to a list
        # We can experiment and see though
        return list(np.zeros((360)))

    
    def timer_callback(self):
        """Run the supervisor and publish the shaped scan
        """

        # Don't run this if necessary messages have not been received
        if self.turtlebot_pose_msg is None or self.clean_scan_msg is None:
            self.get_logger().info("Waiting for pose and scan messages.")
            return None

        # Aggregate messages and information into supervisor input
        supervisor_input = self.aggregate_supervisor_input()

        # Compute the counterfactual scan
        counterfactual_scan = self.compute_counterfactual(supervisor_input)

        # Add counterfactual scan to actual scan to get the shaped scan
        actual_scan = self.clean_scan_msg.ranges
        # shaped_scan = actual_scan + counterfactual_scan # This is what we would use if the scans were np arrays
        shaped_scan = [actual_range+counterfactual_range for actual_range, counterfactual_range in zip(actual_scan, counterfactual_scan)]

        # Pack shaped scan into a message
        shaped_scan_msg = deepcopy(self.clean_scan_msg)
        shaped_scan_msg.ranges = list(shaped_scan)

        # Publish the shaped scan
        self.shaped_scan_publisher.publish(shaped_scan_msg)

def main(args=None):
    rclpy.init(args=args)
    supervisor = Supervisor()

    rclpy.spin(supervisor)

    supervisor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
