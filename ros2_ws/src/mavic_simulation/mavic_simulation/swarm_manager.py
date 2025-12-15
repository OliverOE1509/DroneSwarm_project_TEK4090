import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32
import functools
import math

class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')

        #self.declare_parameter('NDrones', 2)
        self.declare_parameter('loop_freq_hz', 2.0)

        self.n_drones = int(self.get_parameter('NDrones').value)
        freq_hz = float(self.get_parameter('loop_freq_hz').value)

        self.drone_positions = {}   # drone_id -> (x,y,z)
        self.flag_pos = None
        self.current_leader = None

        # --- Subscribers ---
        self.flag_sub = self.create_subscription(
            PointStamped,
            '/flag/gps',
            self._flag_cb,
            10
        )

        self._subs = []
        for i in range(1, self.n_drones + 1):
            topic = f"/Mavic_2_PRO_{i}/gps"
            cb = functools.partial(self._gps_cb, drone_id=i)
            self._subs.append(
                self.create_subscription(PointStamped, topic, cb, 10)
            )

        # --- Publisher ---
        self.leader_pub = self.create_publisher(Int32, '/swarm/leader', 10)

        self.timer = self.create_timer(1.0 / freq_hz, self.timer_callback)

    def _flag_cb(self, msg):
        self.flag_pos = (msg.point.x, msg.point.y, msg.point.z)

    def _gps_cb(self, msg, drone_id):
        self.drone_positions[drone_id] = (
            msg.point.x, msg.point.y, msg.point.z
        )

    def compute_cost(self, pos):
        dx = self.flag_pos[0] - pos[0]
        dy = self.flag_pos[1] - pos[1]
        dz = self.flag_pos[2] - pos[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def timer_callback(self):
        if self.flag_pos is None:
            return
        if len(self.drone_positions) < self.n_drones:
            return

        costs = {
            i: self.compute_cost(pos)
            for i, pos in self.drone_positions.items()
        }

        leader = min(costs, key=lambda i: (costs[i], i))

        if leader != self.current_leader:
            self.current_leader = leader
            msg = Int32()
            msg.data = leader
            self.leader_pub.publish(msg)
            self.get_logger().info(
                f"Leader elected: Drone {leader}, costs={costs} IN swarm_manager.py"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
