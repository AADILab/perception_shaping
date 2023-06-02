""" Clean up the lidar scan of infs and other noise
"""

import rclpy
from rclpy.node import Node

# Library for handling lidar data
from sensor_msgs.msg import LaserScan

class ScanCleaner(Node):
    def __init__(self):
        super().__init__('scan_cleaner')
        # Publisher for cleaned scan
        self.clean_scan_publisher = self.create_publisher(LaserScan, '/clean_scan', 10)

        # Subscriber for raw scan
        self.raw_scan_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10
            )
        self.raw_scan_subscriber
    
    def scan_callback(self, msg: LaserScan):
        # self.get_logger().info("heartbeat")
        # self.get_logger().info('\nMin angle: "%s" \nMax angle: "%s"' % (msg.angle_min, msg.angle_max))
        # self.get_logger().info(str(msg.ranges))

        # Edit header with metadata for this callback
        msg.header.stamp = self.get_clock().now().to_msg()
        # Keep the sequence id and frame_id consistent with the raw scan

        # Make all sensed ranges within the max and min range
        for i in range(len(msg.ranges)):
            if msg.ranges[i] > msg.range_max:
                msg.ranges[i] = msg.range_max
            elif msg.ranges[i] < msg.range_min:
                msg.ranges[i] = msg.range_min
        # self.get_logger().info(str(msg.ranges))
        # msg.ranges[msg.ranges>msg.range_max] = msg.range_max
        # msg.ranges[msg.ranges<msg.range_min] = msg.range_min

        self.clean_scan_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_cleaner = ScanCleaner()

    rclpy.spin(scan_cleaner)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scan_cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
