import rclpy
from geometry_msgs.msg import Point


class Drone2Driver:
    """
    Webots plugin + ROS node for drone2.

    - Publishes /drone2/position (Point)
    - Subscribes to /drone1/position (Point)
    - When drone1.z >= 1.0, start vertical oscillation between 0.1 and 1.0.
    """

    def init(self, webots_node, properties):
        # Webots handles
        print("Er inne i driver 2")
        self._robot = webots_node.robot
        self._time_step = int(self._robot.getBasicTimeStep())
        self._translation_field = self._robot.getField('translation')

        # Internal state
        self._phase = "waiting"   # "waiting", "up", "down"
        self._low_alt = 0.1
        self._high_alt = 1.0
        self._vertical_speed = 0.5

        # Store last known position of drone1
        self._drone1_position = None

        # ROS node
        rclpy.init(args=None)
        self._node = rclpy.create_node('drone2_node')

        self._pos_pub = self._node.create_publisher(Point, '/drone2/position', 10)
        self._drone1_sub = self._node.create_subscription(
            Point, '/drone1/position', self._drone1_position_callback, 10
        )

    def _drone1_position_callback(self, msg: Point):
        self._drone1_position = msg

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0.0)

        dt = self._time_step / 1000.0

        pos = list(self._translation_field.getSFVec3f())
        x, y, z = pos

        # Transition from waiting → oscillation when drone1 at 1m
        if self._phase == "waiting":
            if self._drone1_position is not None and self._drone1_position.z >= self._high_alt - 1e-2:
                self._phase = "up"  # start by going up from current altitude

        elif self._phase == "up":
            z += self._vertical_speed * dt
            if z >= self._high_alt:
                z = self._high_alt
                self._phase = "down"

        elif self._phase == "down":
            z -= self._vertical_speed * dt
            if z <= self._low_alt:
                z = self._low_alt
                self._phase = "up"

        # Apply updated position
        self._translation_field.setSFVec3f([x, y, z])

        # Publish our position
        p = Point()
        p.x, p.y, p.z = x, y, z
        self._pos_pub.publish(p)

""" def main():
    from controller import Supervisor
    robot = Supervisor()
    driver = Drone2Driver()
    driver.init(robot, {})
    while robot.step(robot.getBasicTimeStep()) != -1:
        driver.step() """