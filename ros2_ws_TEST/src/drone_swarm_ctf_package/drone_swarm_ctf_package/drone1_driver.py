import math
from controller import Supervisor
import rclpy
from geometry_msgs.msg import Point


class Drone1Driver:
    """
    Webots plugin + ROS node for drone1.

    - Publishes /drone1/position (Point)
    - Subscribes to /drone2/position (Point) (just to satisfy "exchange info")
    - Controls translation: up -> forward -> down
    """

    def init(self, webots_node, properties):
        # Webots handles
        print("Er inne i driver 1")

        self._robot = webots_node.robot
        self._time_step = int(self._robot.getBasicTimeStep())
        self._translation_field = self._robot.getField('translation')

        # Internal state
        self._phase = "takeoff"   # "takeoff", "forward", "land", "done"
        self._target_altitude = 1.0
        self._target_forward = 1.0
        self._vertical_speed = 0.5   # m/s
        self._forward_speed = 0.5    # m/s

        # ROS node
        rclpy.init(args=None)
        self._node = rclpy.create_node('drone1_node')

        self._pos_pub = self._node.create_publisher(Point, '/drone1/position', 10)
        self._other_pos_sub = self._node.create_subscription(
            Point, '/drone2/position', self._other_position_callback, 10
        )

        self._other_last_position = None

    def _other_position_callback(self, msg: Point):
        self._other_last_position = msg

    def step(self):
        # Spin ROS callbacks
        rclpy.spin_once(self._node, timeout_sec=0.0)

        dt = self._time_step / 1000.0

        # Read current position [x, y, z]
        pos = list(self._translation_field.getSFVec3f())
        x, y, z = pos

        # Simple state machine for drone1
        if self._phase == "takeoff":
            # go up until z >= target_altitude
            z += self._vertical_speed * dt
            if z >= self._target_altitude:
                z = self._target_altitude
                self._phase = "forward"

        elif self._phase == "forward":
            # move forward in x
            x += self._forward_speed * dt
            if x >= self._target_forward:
                x = self._target_forward
                self._phase = "land"

        elif self._phase == "land":
            z -= self._vertical_speed * dt
            if z <= 0.1:
                z = 0.1
                self._phase = "done"

        # If "done": no change

        # Apply new position
        self._translation_field.setSFVec3f([x, y, z])

        # Publish our position
        p = Point()
        p.x, p.y, p.z = x, y, z
        self._pos_pub.publish(p)

""" def main():
    from controller import Supervisor
    robot = Supervisor()
    driver = Drone1Driver()
    driver.init(robot, {})
    while robot.step(robot.getBasicTimeStep()) != -1:
        driver.step() """