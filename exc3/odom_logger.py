#!/usr/bin/env python3
# odom logging
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # open and write the data into the csv file
        self.csv_file = open('odom_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y', 'theta'])

        # store latest odom values
        self.latest_x = None
        self.latest_y = None
        self.latest_theta = None

        # timer for logging every 0.25 seconds
        self.timer = self.create_timer(0.25, self.log_to_csv)

    def odom_callback(self, msg):
        # update the latest odom values
        self.latest_x = msg.pose.pose.position.x
        self.latest_y = msg.pose.pose.position.y
        # You probably want yaw (rotation about z), not just quaternion.z
        self.latest_theta = msg.pose.pose.orientation.z  

    def log_to_csv(self):
        if self.latest_x is not None:
            timestamp = self.get_clock().now().to_msg()
            self.csv_writer.writerow([timestamp, self.latest_x, self.latest_y, self.latest_theta])

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
