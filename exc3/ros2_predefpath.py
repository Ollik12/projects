import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# adding the position and orientation 
from nav_msgs.msg import Odometry
import math
import sys
# for reading the csv file:
import csv

# "GoForward" class inherits from the base class "Node"

# making just very basic navigation based on the goforward.py to position
class GoToGoal(Node):

    def __init__(self, waypoints):
        # Initialize the node
        super().__init__('to_to_goal')
        # Initialize the publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # lets make the subscriber for the /odom topic
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # initial position for the robot
        self.x = 0
        self.y = 0
        self.rot = 0
        self.current_id = 0
        # goal from input
        self.waypoints = waypoints
        timer_period = 0.1 # seconds
        # Initialize a timer that executes callback function every 0.1 seconds
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
        # check that is there waypoints left
        if self.current_id >= len(self.waypoints):
            self.stop_turtlebot()
            self.get_logger().info("All waypoints reached")
            return

        # if not done - get the current goal (x,y,theta)
        self.x_goal = self.waypoints[self.current_id][0]
        self.y_goal = self.waypoints[self.current_id][1]
        self.theta_goal = self.waypoints[self.current_id][2]


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
                # add data to the logger
                self.get_logger().info(f"Waypoint {self.current_id+1} reached: ({self.x_goal}, {self.y_goal}, {self.theta_goal})")
                self.current_id += 1
            else:
                # position correct -> change orientation
                move_cmd.linear.x = 0.0
                move_cmd.angular.z = max(min(0.5 * theta_error, 1.0), -1.0)
        else:
            # control towards goal because we are not close enough
            move_cmd.linear.x = min(0.25, 0.25 * distance / 0.5)
            move_cmd.angular.z = max(min(1.5 * (angle_to_goal - self.rot), 1.0), -1.0)

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

    # with command-line we give the file of csv file
    if len(sys.argv) < 2:
        print("python3 ros2goxy.py path_to_csv_file")
        return

    csv_file = sys.argv[1]

    waypoints = []
    with open(csv_file, 'r') as f:
        reader = csv.reader(f)
        # lets skip the header row and read the rest of the rows
        next(reader)
        for row in reader:
            # reading the waypoints as floats and appending to the list so control works
            waypoints.append((float(row[1]), float(row[2]), float(row[3])))


    try:
        # start the gotogoal
        cmd_publisher = GoToGoal(waypoints)
        # continue until interrupted
        rclpy.spin(cmd_publisher)
    except KeyboardInterrupt:
        # execute shutdown function
        cmd_publisher.stop_turtlebot()
        # clear the node
        cmd_publisher.destroy_node()
        rclpy.shutdown()
    	
if __name__ == '__main__':
    main()
