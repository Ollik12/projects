import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# adding the position and orientation 
from nav_msgs.msg import Odometry
import math
import sys
import threading

# "GoForward" class inherits from the base class "Node"

# making just very basic navigation based on the goforward.py to position
class GoToGoal(Node):

    def __init__(self, x_goal, y_goal, theta_goal):
        # Initialize the node
        super().__init__('go_to_goal')
        # Initialize the publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # lets make the subscriber for the /odom topic
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # initial position for the robot
        self.x = 0
        self.y = 0
        self.rot = 0
        # goal from input
        self.x_goal = x_goal
        self.y_goal = y_goal
        self.theta_goal = theta_goal
        # adding a bool value so we send only one command on goal reached
        self.goal_reached = False

        timer_period = 0.1 # seconds
        # Initialize a timer that executes callback function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
    def odom_callback(self, msg):
        # updating of the current position from pose and orientation from the /odom
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        
        q = msg.pose.pose.orientation  # get the orientation quaternion from odometry message

        # Compute components for rotation around z
        # i looked this up from some : "https://oduerr.github.io/gesture/ypr_calculations.html"

        # Formula is rotz = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2)) where w is the scalar in the
        # quaternion. so when we have rotation around x adn y as 0 we get simplified:
        self.rot = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z)) # store robot’s current heading around z-axis
        # note all q.x and q.y are zeros

    def timer_callback(self):
        # Create an object of msg type Twist() 
        # and lets control movement with the object
        move_cmd = Twist()
        # difference in current and goal
        deltax = self.x_goal - self.x
        deltay = self.y_goal - self.y

        # lets calc the distance:
        distance = math.sqrt(deltax**2 + deltay**2)
        # final orientation.
        angle_to_goal = math.atan2(deltay, deltax)

        # Heading error for final orientation
        theta_error = self.theta_goal - self.rot
        # angle normalisation
        theta_error = math.atan2(math.sin(theta_error), math.cos(theta_error))
        # position tolerances:
        position_offset = 0.1
        angle_offset = 0.1

        # control for cmd_vel commands
        if distance < position_offset:
            # close to target -> no more movement -> check for orientation
            if abs(theta_error) < angle_offset:
                # goal reached
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 0.0
                # add data to the logger that we reached the goal
                if (self.goal_reached == False):
                    self.get_logger().info(f"Goal reached: ({self.x_goal}, {self.y_goal}, {self.theta_goal})")
                    self.goal_reached = True
            else: 
                # position correct -> change orientation
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = 1.5 * theta_error
        else:
            # control towards goal because we are not close enough
            # logging the distance to goal.
            self.get_logger().info(f"Distance to target: ({distance:.3f} m.)")
            move_cmd.linear.x = 0.25
            move_cmd.angular.z = 1.5 * (angle_to_goal - self.rot)

        # and publish the result
        self.publisher_.publish(move_cmd)
        
    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
        self.publisher_.publish(Twist())
    	
def main(args=None):
    rclpy.init(args=args)

    # Creaation of the node with the initial goal of (0.0,0.0,0.0)
    cmd_publisher = GoToGoal(0.0, 0.0, 0.0)

    # Run ROS spin in a separate thread so we can use input() in the main thread
    thread = threading.Thread(target=rclpy.spin, args=(cmd_publisher,))
    thread.start()

    try:
        while True:
            # ask for new coordinates
            print("\nEnter new goal coordinates (or 'q' to quit):")
            x_goal = input("x_goal: ")
            if x_goal.lower() == 'q':
                break
            y_goal = input("y_goal: ")
            if y_goal.lower() == 'q':
                break
            theta_goal = input("theta_goal (in radians): ")
            if theta_goal.lower() == 'q':
                break

            try:
                cmd_publisher.x_goal = float(x_goal)
                cmd_publisher.y_goal = float(y_goal)
                cmd_publisher.theta_goal = float(theta_goal)
                cmd_publisher.get_logger().info(
                    f"New goal set: ({x_goal}, {y_goal}, {theta_goal})"
                )
                # set the goal reached to false so we can print the result we reach again.
                cmd_publisher.goal_reached = False
            except ValueError:
                print("Invalid input, please enter numeric values.")

    except KeyboardInterrupt:
        pass
    finally:
        cmd_publisher.stop_turtlebot()
        cmd_publisher.destroy_node()
        rclpy.shutdown()
        thread.join()
    	
if __name__ == '__main__':
    main()
