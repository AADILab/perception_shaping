""" This script gets the ground truth pose of the turtlebot from the /tf topic in gazebo and publishes it as an Odometry message to /turtlebot_pose
"""

import rclpy
from rclpy.node import Node

# Messages for receiving transforms and publishing odometry
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class GazeboPosePublisher(Node):
    def __init__(self):
        super().__init__('gazebo_pose_publisher')
        # Subscriber for tf messages
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.tf_subscriber

        # Publisher for odometry
        self.pose_publisher = self.create_publisher(Odometry, '/turtlebot_pose', 10)

    def tf_callback(self, msg: TFMessage):
        # Make sure this contains the turtlebot pose information
        has_turtlebot_pose = False
        turtlebot_transform: TransformStamped = None
        for transform in msg.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_footprint":
                has_turtlebot_pose = True
                turtlebot_transform = transform
                break

        # Take the pose and repackage it into an odometry message
        if has_turtlebot_pose:
            # Create odometry message. Populate it with the header and child frame information
            odom_msg = Odometry()
            odom_msg.header = turtlebot_transform.header
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.child_frame_id = turtlebot_transform.child_frame_id

            # Populate the position information
            # Need to do x,y,z seperately because the position message type is different for odom vs transform
            odom_msg.pose.pose.position.x = turtlebot_transform.transform.translation.x
            odom_msg.pose.pose.position.y = turtlebot_transform.transform.translation.y
            odom_msg.pose.pose.position.z = turtlebot_transform.transform.translation.z

            # Populate the angle information.
            # Can do this directly because both messages use the quaternion message type for orientation
            odom_msg.pose.pose.orientation = turtlebot_transform.transform.rotation

            # Note the covariance matrix is not populated because we don't need it for perception shaping

            # Publish the new odometry
            self.pose_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    gazebo_pose_publisher = GazeboPosePublisher()

    rclpy.spin(gazebo_pose_publisher)

    gazebo_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
