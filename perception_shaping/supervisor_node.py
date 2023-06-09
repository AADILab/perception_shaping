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

import yaml

# Useful standard libraries
from copy import deepcopy
from typing import Optional
import pickle
import os

class Supervisor:
    def __init__(self, n_inp=9, n_out=360, n_hid=30):
        self.turtlebot_scan_dist = 3.5  # Turtlebot's scan distance (meters)
        self.n_inputs = n_inp  # Number of nodes in input layer
        self.n_outputs = n_out  # Number of nodes in output layer
        self.n_hidden = n_hid  # Number of nodes in hidden layer
        self.weights = {}
        self.input_layer = np.zeros(self.n_inputs)
        self.hidden_layer = np.zeros(self.n_hidden)
        self.output_layer = np.zeros(self.n_outputs)

    def scan_environment(self, turtlebot, target, hazard):
        """
        Supervisor collects information about environment
        """
        # Turtlebot data
        self.input_layer[0] = turtlebot[0]  # Turtlebot x position
        self.input_layer[1] = turtlebot[1]  # Turtlebot y position
        self.input_layer[2] = turtlebot[2]  # Turtlebot heading

        # Target data
        self.input_layer[3] = target[0]  # Target center x-coordinate
        self.input_layer[4] = target[1]  # Target center y-coordinate
        self.input_layer[5] = target[2]  # Target radius

        # Hazard sata
        self.input_layer[6] = hazard[0]  # Hazard center x-coordinate
        self.input_layer[7] = hazard[1]  # Hazard center y-coordinate
        self.input_layer[8] = hazard[2]  # Hazard radius

    # Supervisor NN ------------------------------------------------------------------------------------------------
    def get_weights(self, nn_weights):
        """
        Apply chosen network weights to supervisor's neural network
        """
        self.weights['l1'] = nn_weights['L1'][0:self.n_inputs, :]
        self.weights['l1_bias'] = nn_weights['L1'][self.n_inputs, :]  # Biasing weights
        self.weights['l2'] = nn_weights['L2'][0:self.n_hidden, :]
        self.weights['l2_bias'] = nn_weights['L2'][self.n_hidden, :]  # Biasing weights

    def get_nn_outputs(self):
        """
        Run NN to generate counterfactual for worker agents
        """
        self.hidden_layer = np.dot(self.input_layer, self.weights['l1']) + self.weights['l1_bias']
        self.hidden_layer = self.sigmoid(self.hidden_layer)

        self.output_layer = np.dot(self.hidden_layer, self.weights['l2']) + self.weights['l2_bias']
        self.output_layer = self.sigmoid(self.output_layer)

    def run_supervisor_nn(self, turtlebot, target, hazard):
        """
        Run neural network using state information, return counterfactual state
        """
        self.scan_environment(turtlebot, target, hazard)
        self.get_nn_outputs()

        # Convert NN outputs to a LiDAR distance
        counterfactual = np.multiply(self.output_layer, -1.5*self.turtlebot_scan_dist)

        return counterfactual

    # Activation Functions -------------------------------------------------------------------------------------------
    def tanh(self, inp):  # Tanh function as activation function
        """
        tanh neural network activation function
        """
        tanh = (2 / (1 + np.exp(-2 * inp))) - 1

        return tanh

    def sigmoid(self, inp):  # Sigmoid function as activation function
        """
        sigmoid neural network activation function
        """
        sig = 1 / (1 + np.exp(-inp))

        return sig


POI_POSITION =     [7.7,  7.7]
HAZZARD_POSITION = [6.75, 6.7]
POI_RADIUS = 0.3048
HAZZARD_RADIUS = 0.75

class SupervisorNode(Node):
    def __init__(self, policy_filename: Optional[str]):
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

        # Variable for which supervisor policy to load in. 
        # The default is to load in no supervisor, and just use the null counterfactual
        self.policy_filename = policy_filename
        if self.policy_filename is not None:
            self.supervisor = self.load_network(self.policy_filename)
        else:
            self.supervisor = None

        self.save_trajectory = False
        self.path_name = "/root/foxy_ws/src/perception_shaping/logs/log.yaml"
        self.log_file = open(self.path_name, 'w')
        self.log_counter = 0

        self.xs = []
        self.ys = []
        self.thetas = []
        self.ts = []
        self.log_data = {
            "xs": self.xs,
            "ys": self.ys,
            "thetas": self.thetas,
            "ts": self.ts
        }

    def load_network(self, policy_filename):
        sp = Supervisor()
        with open(policy_filename, 'rb') as weights_file:
            weights = pickle.load(weights_file)
        sp.get_weights(weights)
        return sp

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
        self.get_logger().info("%s %s %s"%(turtlebot_x, turtlebot_y, turtlebot_angle))
        self.get_logger().info("{:.2} {:.2} {:.3}".format(turtlebot_x, turtlebot_y, turtlebot_angle))

        # Get the poi position and hazzard position
        poi_x, poi_y = POI_POSITION
        hazzard_x, hazzard_y = HAZZARD_POSITION

        # Put it all together
        # Convert turtlebot angle from radians to degrees for feeding into the supervisor, which is trained on degrees
        turtlebot_angle_degrees = turtlebot_angle*57.2958
        # Adjust the degrees so that we have degrees mapping from [-180,180] instead of [0,360]
        if turtlebot_angle_degrees < 180: turtlebot_angle_degrees += 360
        turtlebot_angle_degrees -= 360

        supervisor_input = np.array([turtlebot_x, turtlebot_y, turtlebot_angle_degrees, poi_x, poi_y, hazzard_x, hazzard_y])
        self.get_logger().info(f"supervisor_input: {list(supervisor_input)}")
        return supervisor_input

    def compute_counterfactual(self, supervisor_input):
        # Computes the counterfactual lidar scan that we add to the actual scan to get the shaped scan

        if self.supervisor is None:
            # Until we integrate the supervisor, just return a null counterfactual (all zeros)
            # Return a list rather than a numpy array because the ranges in the message is a list
            # Should be faster to add two lists rather than convert the message list to a numpy array, add numpy arrays, then convert back to a list
            # We can experiment and see though
            return list(np.zeros((360)))

        else:
            # Then we actually run the network
            counterfactual = self.supervisor.run_supervisor_nn(turtlebot=supervisor_input[0:3], target=list(supervisor_input[3:5])+[POI_RADIUS], hazard=list(supervisor_input[5:7])+[HAZZARD_RADIUS])
            self.get_logger().info(f"counterfactual: {list(counterfactual)}")
            return list(counterfactual)
    
    def timer_callback(self):
        """Run the supervisor and publish the shaped scan
        """

        # Don't run this if necessary messages have not been received
        if self.turtlebot_pose_msg is None or self.clean_scan_msg is None:
            # self.get_logger().info("Waiting for pose and scan messages.")
            return None

        # Aggregate messages and information into supervisor input
        supervisor_input = self.aggregate_supervisor_input()

        # Compute the counterfactual scan
        counterfactual_scan = self.compute_counterfactual(supervisor_input)

        # Add counterfactual scan to actual scan to get the shaped scan
        actual_scan = self.clean_scan_msg.ranges
        # shaped_scan = actual_scan + counterfactual_scan # This is what we would use if the scans were np arrays
        shaped_scan = [actual_range+counterfactual_range for actual_range, counterfactual_range in zip(actual_scan, counterfactual_scan)]

        shaped_scan = [0.1 if shaped_range < 0 else shaped_range for shaped_range in shaped_scan]

        # self.get_logger().info(f"clean_scan: {actual_scan}")
        # self.get_logger().info(f"shaped_scan: {shaped_scan}")

        # Pack shaped scan into a message
        shaped_scan_msg = deepcopy(self.clean_scan_msg)
        shaped_scan_msg.ranges = list(shaped_scan)

        # Publish the shaped scan
        self.shaped_scan_publisher.publish(shaped_scan_msg)

        if self.save_trajectory:
            # Record the pose of the robot at this timestep
            self.xs.append(float(supervisor_input[0]))
            self.ys.append(float(supervisor_input[1]))
            self.thetas.append(float(supervisor_input[2]))
            # self.ts.append(self.get_clock().now())
            # os.remove(self.path_name)
            with open(self.path_name+str(self.log_counter)+".yaml", 'w') as self.log_file:
                yaml.dump(self.log_data, self.log_file, default_flow_style=False)
            self.log_counter+=1

def main(args=None):
    rclpy.init(args=args)
    policy_filename = "/root/foxy_ws/src/perception_shaping/supervisor_policies/policy_000.pkl"
    # policy_filename = None
    supervisor_node = SupervisorNode(policy_filename=policy_filename)

    rclpy.spin(supervisor_node)

    supervisor_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
