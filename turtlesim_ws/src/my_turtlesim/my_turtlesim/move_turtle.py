# Import Python standard libraries
import sys
import math

# Import ROS2 Python API
import rclpy
from rclpy.node import Node

# Import message definitions
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

# Turtle will stop rotating in place when its angle is within this threshold of the desired angle. Units: Radians
ANGULAR_THRESHOLD = 0.001

# Constant multiplier for angular velocity when turtle is rotating in place
ANGULAR_GAIN = 1.0

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Initialize publisher for turtle velocity
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Initialize subscriber for turtle pose
        self.turtle_pose = None
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.got_turtle_pose, 10)

    def got_turtle_pose(self, turtle_pose: Pose):
        """
        Callback when turtle pose is received.

        Args:
            turtle_pose (Pose): Position of turtle.
        """
        self.turtle_pose = turtle_pose

    def run(self):
        """
        Function that runs this node and calls task planning.
        """

        # Wait until turtle pose is received
        while self.turtle_pose is None:
            rclpy.spin_once(self)

        # Test `rotate_to_global_theta`
        thetas = [0, math.pi/2, math.pi, -math.pi/2, 0, -math.pi/2, -math.pi, math.pi/2, 0]
        for theta in thetas:
            self.rotate_to_global_theta(theta)

    """
    Task #1: Draw Square
        - (5.5, 5.5), (7.5, 5.5), (7.5, 7.5), (5.5, 7.5)
    """

    def draw_square(self):
        """
        Move to (5.5, 5.5)
        Move to (7.5, 5.5)
        Move to (7.5, 7.5)
        Move to (5.5, 7.5)
        """
        pass

    def move_to_global_pos_and_stop(self, x: float, y: float):
        """
        Move turtle to desired x, y position.

        Args:
            x (float): X-coordinate of desired position.
            y (float): Y-coordinate of desired position.
        """

        """
        Need: Turtle's pose (x, y, theta)
        Need to subscribe to /turtle1/pose (DONE)

        Algorithm:
        1. Compute theta that turtle needs to be at to face (x, y)
        2. Rotate to computed theta
        3. Set velocity to positive linear x to move forward until turtle reaches (x, y)
        """
        pass

    def rotate_to_global_theta(self, theta: float):
        """
        Rotate turtle to theta. Does not change x, y position of turtle.

        Args:
            theta (float): The angle to rotate the turtle to. Must be in radians.
        """
        # Compute amount to rotate
        delta_theta = self.convert_0_2pi_to_neg_pi_pi(theta - self.turtle_pose.theta)

        twist = Twist()

        # Loop while turtle isn't within a threshold of the desired theta
        while rclpy.ok() and abs(delta_theta) > ANGULAR_THRESHOLD:

            # Update angular velocity to be proportional to amount left to rotate
            twist.angular.z = delta_theta * ANGULAR_GAIN

            # Publish velocity
            self.vel_publisher.publish(twist)

            # Allow ROS to process published message
            rclpy.spin_once(self)

            # Update remaining angle
            delta_theta = self.convert_0_2pi_to_neg_pi_pi(theta - self.turtle_pose.theta)

        # Stop turtle
        self.vel_publisher.publish(Twist())


    def convert_0_2pi_to_neg_pi_pi(self, theta: float):
        """
        Convert an angle to range [-pi, pi].

        Args:
            theta (float): Angle in radians to convert.
        """
        return (theta + math.pi) % (2 * math.pi) - math.pi



def main():
    """
    Main function to create and run MoveTurtle node. Called by `ros2 run my_turtlesim move_turtle`.
    """
    rclpy.init(args=sys.argv)
    move_turtle = MoveTurtle()

    try:
        move_turtle.run()
    except KeyboardInterrupt:
        pass
    finally:
        move_turtle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
