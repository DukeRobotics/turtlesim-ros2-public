import sys
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

ANGULAR_THRESHOLD = 0.001
ANGULAR_GAIN = 1

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Initialize publisher for turtle velocity
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        self.turtle_pose = None
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.got_turtle_pose, 10)

        self.rate = self.create_rate(20)

    def got_turtle_pose(self, turtle_pose: Pose):
        """
        Callback when turtle pose is received.

        Args:
            turtle_pose (Pose): Position of turtle.
        """
        self.turtle_pose = turtle_pose

        # Logging
        # print("message")
        # self.get_logger().info(str(self.turtle_pose))

    def run(self):
        # Publish velocity to move turtle forward
        # rclpy.spin(self)
        # twist = Twist()
        # twist.linear.x = 1.0

        # while rclpy.ok():
        #     self.vel_publisher.publish(twist)
        #     rclpy.spin_once(self)
        #     self.rate.sleep()

        while self.turtle_pose is None:
            rclpy.spin_once(self)

        self.rotate_to_global_theta(-math.pi)

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
        What is the high-level algorithm to move to an x,y position?

        Need: Turtle's pose (x, y, theta)
        Need to subscribe to /turtle1/pose

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

        # theta could be any value (not nececessarily betwen -pi and pi)

        """
        1. Compute delta_theta (the angle that we need to rotate from the current position)
        2. Loop while delta_theta > threshold:
            1. Publish angular Z velocity to rotate turtle
            2. Update the detla_theta
        3. Stop turtle by publishing zero velocity
        """

        # Compute amount to rotate
        delta_theta = self.convert_0_2pi_to_neg_pi_to_pi(theta - self.turtle_pose.theta)

        twist = Twist()

        # Loop while turtle isn't within a threshold of the desired theta
        while rclpy.ok() and abs(delta_theta) > ANGULAR_THRESHOLD:
            twist.angular.z = delta_theta * ANGULAR_GAIN

            # Publish velocity
            self.vel_publisher.publish(twist)

            rclpy.spin_once(self)

            # Update remaining angle
            delta_theta = self.convert_0_2pi_to_neg_pi_to_pi(theta - self.turtle_pose.theta)

            self.rate.sleep()

        # Stop turtle
        self.vel_publisher.publish(Twist())


    def convert_0_2pi_to_neg_pi_to_pi(self, theta: float):
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
