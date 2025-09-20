import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Obstacle avoidance node started")

    def lidar_callback(self, msg: LaserScan):
        if not msg.ranges:
            return

        twist = Twist()

        # Check if all distances are safe (>1.0m)
        all_clear = all(r > 1.0 for r in msg.ranges if r > 0.0)

        if all_clear:
            # Move forward
            twist.linear.x = 0.75
            twist.angular.z = 0.0
        else:
            # Obstacle detected: rotate in place to right
            twist.linear.x = 0.0
            twist.angular.z = -0.5

        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
