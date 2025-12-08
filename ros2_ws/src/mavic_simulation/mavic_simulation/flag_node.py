import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import functools
import numpy as np

class PublisherNode(Node):
    '''A bit unsure if we implement the respawn logic here or in the launch file'''

    def __init__(self):
        super().__init__('publisher_node')
        self.declare_parameter('NDrones', 4)
        n_drones = self.get_parameter('NDrones').value

        self.drone_positions = {  } # drone_id: (x,y,z)
        self._subs = []
        for i in range(1, n_drones +1):
            topic = f"/Mavic_2_PRO_{i}/gps"
            cb = functools.partial(self._gps_cb, drone_id=i)
            sub = self.create_subscription(
                PointStamped,
                topic,
                cb,
                10
            )
            self._subs.append(sub)
        self.flag_pug = self.create_publisher(PointStamped, '/flag/gps', 10)
        self.flag_x = 5.0
        self.flag_y = 3.0
        self.flag_z = 10.0
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def respawn_flag(self, radius = 2.0):
        flag_vec = np.array([self.flag_x, self.flag_y, self.flag_z])
        for drone_id, pos in self.drone_positions.items():
            drone_vec = np.array(pos)
            dist = np.linalg.norm(flag_vec - drone_vec)
            if dist < radius:
                # Respawn
                self.flag_x = np.random.uniform(-10, 10)
                self.flag_y = np.random.uniform(-10, 10)
                self.flag_z = np.random.uniform(5, 15)
                self.get_logger().info(f'Flag respawned to: x={self.flag_x}, y={self.flag_y}, z={self.flag_z} due to proximity to drone {drone_id}')
                break


    def _gps_cb(self, msg, drone_id):
        self.drone_positions[drone_id] = (float(msg.point.x), float(msg.point.y), float(msg.point.z))

    def timer_callback(self):
        self.respawn_flag()
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.point.x = self.flag_x
        msg.point.y = self.flag_y
        msg.point.z = self.flag_z
        self.flag_pug.publish(msg)
        self.get_logger().info(f'Publishing flag position at: x={self.flag_x}, y={self.flag_y}, z={self.flag_z} | Drone positions: {self.drone_positions}')



def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()