# Import Python standard libraries
import sys
import math
from typing import Tuple, Optional

# Import ROS2 Python API
import rclpy
from rclpy.node import Node

# Import message definitions
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import SetPen

# Turtle will stop rotating in place when its angle is within this threshold of the desired angle. Units: Radians
ANGULAR_THRESHOLD = 0.001

# Constant multiplier for angular velocity when turtle is rotating in place
ANGULAR_GAIN = 4.0

# Turtle will stop moving to new position when the distance to the new position is within this threshold.
LINEAR_THRESHOLD = 0.001

# Constant multiplier for linear velocity when turtle is moving to a new position
LINEAR_GAIN = 3.0

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')

        # Initialize publisher for turtle velocity
        self.vel_publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # Initialize subscriber for turtle pose
        self.turtle_pose = None
        self.pose_subscriber = self.create_subscription(Pose, 'turtle1/pose', self.got_turtle_pose, 10)

        # Initialize client for clear service
        self.clear_client = self.create_client(Empty, 'clear')
        while not self.clear_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Clear service not available, waiting...')

        # Initialize client for set pen service
        self.set_pen_client = self.create_client(SetPen, 'turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('SetPen service not available, waiting...')

        # Intialize pen color and width
        self.pen_color = (178, 184, 255)  # RGB
        self.pen_width = 5

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

        # Clear all drawings off canvas
        self.clear()

        self.pen_color = (255, 0, 0)
        self.pen_width = 10
        self.set_pen(*self.pen_color, self.pen_width, False)
        self.draw_reg_polygon(4, 3)

    # ---------------------- TASK PLANNING ----------------------

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
        positions = [(7.5, 5.5), (7.5, 7.5), (5.5, 7.5), (5.5, 5.5)]
        for position in positions:
            self.move_to_global_pos_and_stop(*position)

    """
    Task #2: Draw a regular polygon
    """

    def draw_reg_polygon(self, n: int, side_length: float):
        """
        Draw a regular polygon with n sides and side_length, starting at the turtle's current position and orientation.

        Args:
            n (int): Number of sides in polygon.
            side_length (float): Length of each side of the polygon.
        """
        for _ in range(n):
            self.move_to_local_pos_and_stop(side_length, 0)
            self.rotate_to_local_theta(2 * math.pi / n)


    """
    TODO: COMPLETE THIS INDEPENDENTLY

    Task #3: Draw something cool
        - Incorporate straight lines and arcs
        - Use different pen colors or widths
    """


    # ---------------------- CONTROLS ----------------------

    def move_to_global_pos_and_stop(self, x: float, y: float):
        """
        Move turtle to desired x, y position.

        Args:
            x (float): X-coordinate of desired position.
            y (float): Y-coordinate of desired position.
        """

        # Rotate turtle to face goal
        desired_theta = math.atan2(y - self.turtle_pose.y, x - self.turtle_pose.x)
        self.rotate_to_global_theta(desired_theta)

        twist = Twist()

        # Compute distance to goal
        dist_x, _ = self.global_to_local_vector(x, y)

        # Loop while turtle isn't within a threshold of the goal
        while rclpy.ok() and abs(dist_x) > LINEAR_THRESHOLD:

            # Update linear velocity to be proportional to distance to goal
            twist.linear.x = dist_x * LINEAR_GAIN

            # Publish velocity
            self.vel_publisher.publish(twist)

            # Allow ROS to process published message
            rclpy.spin_once(self)

            # Update distance to goal
            dist_x, _ = self.global_to_local_vector(x, y)

        # Stop turtle
        self.vel_publisher.publish(Twist())

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

    def move_to_local_pos_and_stop(self, x: float, y: float):
        """
        Move x units forward/backward and y units left/right relative to turtle's current pose.

        Args:
            x (float): Units to move forward/backward
            y (float): Units to move left/right
        """
        global_x, global_y = self.local_to_global_vector(x, y)
        self.move_to_global_pos_and_stop(global_x, global_y)


    def rotate_to_local_theta(self, theta: float):
        """
        Rotate turtle by theta radians relative to its current orientation.

        Args:
            theta (float): Angle to rotate turtle by.
        """
        global_theta = self.turtle_pose.theta + theta
        self.rotate_to_global_theta(global_theta)


    # TODO: COMPLETE THIS INDEPENDENTLY
    def draw_arc(self, center_x: float, center_y: float, radius: float, start_theta: float, end_theta: float):
        """
        Draw an arc (part of a circle) with the given center, radius, and start and end angles.

        Angles are defined in radians, with 0 radians corresponding to the global positive x-axis and increasing in the counterclockwise direction.

        Args:
            center_x (float): X-coordinate of center of circle containing the arc.
            center_y (float): Y-coordinate of center of circle containing the arc.
            radius (float): Radius of circle containing arc.
            start_theta (float): Staring angle of arc. Must be between 0 and 2pi.
            end_theta (float): Ending angle of arc. Must be between 0 and 2pi.
        """

        """
        Hints:
            1. Structed similarly to move_to_global_pos_and_stop and rotate_to_global_theta
            2. v = omega * r
                v = linear velocity
                omega = angular velocity
                r = radius
            3. Tune your gain to draw the arc as fast as possible without overshooting.
        """
        pass


    def clear(self):
        """
        Clear the canvas.
        """
        request = Empty.Request()
        future = self.clear_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def set_pen(self, r: int, g: int, b: int, width: int, off: bool):
        """
        Set pen color and width.

        Args:
            r (int): Red value. Must be between 0 and 255.
            g (int): Green value. Must be between 0 and 255.
            b (int): Blue value. Must be between 0 and 255.
            width (int): Width of line drawn by pen. Must be between 0 and 255.
            off (bool): Wether the pen is off or on.
        """
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = int(off)
        future = self.set_pen_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

    def set_pen_off(self):
        """
        Set pen off so turtle doesn't draw when it moves.
        """
        self.set_pen(0, 0, 0, 0, True)

    def set_pen_on(self):
        """
        Set pen on. Update the color and width to self.pen_color and self.pen_width.
        """
        self.set_pen(*self.pen_color, self.pen_width, False)

    def relocate_turtle(self, x: float, y: float, theta: Optional[float] = None):
        """
        Move turtle to a new position and orientation with the pen off. Turn pen back on after relocation.

        Args:
            x (float): X-coordinate of new position.
            y (float): Y-coordinate of new position.
            theta (Optional[float]): Orientation of turtle at the new position. Defaults to None, in which case orientation is not changed.
        """
        self.set_pen_off()
        self.move_to_global_pos_and_stop(x, y)
        if theta is not None:
            self.rotate_to_global_theta(theta)
        self.set_pen_on()

    def convert_0_2pi_to_neg_pi_pi(self, theta: float):
        """
        Convert an angle to range [-pi, pi].

        Args:
            theta (float): Angle in radians to convert.
        """
        return (theta + math.pi) % (2 * math.pi) - math.pi


    def global_to_local_vector(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert a vector in global coordinates into local coordinates.

        Args:
            x (float): X-coordinate in global frmae
            y (float): Y-coordinate in global frame

        Returns:
            tuple (float, float): X, Y coordinates in local frame
        """

        # X - d
        translated_x = x - self.turtle_pose.x
        translated_y = y - self.turtle_pose.y

        # A^-1 (X - d)
        rotated_x = translated_x * math.cos(-self.turtle_pose.theta) - translated_y * math.sin(-self.turtle_pose.theta)
        rotated_y = translated_x * math.sin(-self.turtle_pose.theta) + translated_y * math.cos(-self.turtle_pose.theta)

        return rotated_x, rotated_y

    def local_to_global_vector(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert a vector in local coordinates to global coordinates.

        Args:
            x (float): X-coordinate in local frmae
            y (float): Y-coordinate in local frame

        Returns:
            tuple (float, float): X, Y coordinates in global frame
        """

        # X = Ax + d
        global_x = x * math.cos(self.turtle_pose.theta) - y * math.sin(self.turtle_pose.theta) + self.turtle_pose.x
        global_y = x * math.sin(self.turtle_pose.theta) + y * math.cos(self.turtle_pose.theta) + self.turtle_pose.y

        return global_x, global_y


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
