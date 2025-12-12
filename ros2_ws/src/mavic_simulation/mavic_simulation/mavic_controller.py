import functools, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped, PointStamped, Vector3
from std_msgs.msg import Float32
from webots_ros2_msgs.msg import FloatStamped

STATE_TOPICS  = {
    "/Mavic_2_PRO_ID/compass/bearing": FloatStamped,        #The drone's yaw/angle relative to the north_vector in degrees. 
    "/Mavic_2_PRO_ID/compass/north_vector": Vector3Stamped, #A vector pointing towards the north pole relative to the drone's body frame.
    "/Mavic_2_PRO_ID/gps": PointStamped,              #The drone's position in the global coordinate system.
    "/Mavic_2_PRO_ID/gps/speed": Float32,             #The magnitude of the drone's translational velocity vector in the global coordinate system.
    "/Mavic_2_PRO_ID/gps/speed_vector": Vector3,      #The drone's translational velocity vector in the global coordinate system
}
#TODO: FLAG_TOPIC = {}
STATE_TOPIC_NAMES = [i.split("/")[-1]  for i in STATE_TOPICS.keys()]
CONTROL_TOPICS = {
    "/cmd_vel_Mavic_2_PRO_ID": Twist #Desired translational and rotational velocities in the body frame coordinate system. 
}

class MP2Controller(Node):

    def __init__(self):
        #Init Node
        super().__init__('Mavic_2_PRO_ID_Controller')
        
        # Declare params with typed defaults
        self.declare_parameter('NDrones', 4)
        self.declare_parameter('MavicID', 1)
        self.declare_parameter('loop_freq_hz', 10.0)
        # Get values
        n_drones = int(self.get_parameter('NDrones').value)
        my_id = int(self.get_parameter('MavicID').value)
        freq_hz = int(self.get_parameter('loop_freq_hz').value)
        # keep references so subscriptions aren't GC'd
        self._subs = []
        # latest messages storage: { drone_id: {topic_name: msg}  }
        self.latest = {}
        # create subscriptions for all drone state topics.
        for i in range(1, n_drones + 1):
            for tmpl, msg_type in STATE_TOPICS.items():
                topic = tmpl.replace("ID", str(i))
                cb = functools.partial(self._generic_cb, topic=topic, drone_id=i)

                sub = self.create_subscription(
                    msg_type,
                    topic,
                    cb,
                    10  # qos depth
                )
                self._subs.append(sub)  # keep ref
        
        # create publisher for the drone control topic.
        for tmpl, msg_type in CONTROL_TOPICS.items():
            topic = tmpl.replace("ID", str(my_id))
            self.cmd_vel = self.create_publisher(msg_type, topic, 10)
        
        # create control loop. 
        self.timer = self.create_timer(1.0/freq_hz, self._ctrl_loop)
        

    def _generic_cb(self, msg, topic: str, drone_id: int):
        """
        Generic callback used for all subscriptions.
        Stores the latest message under self.latest[drone_id][topic].
        """
        pyval = self._unpack_msg(msg)
        self.latest.setdefault(drone_id, {})[topic] = pyval

    def _unpack_msg(self, msg):
        """Deterministic unpacker for common messages."""
        # Exact type checks first
        if isinstance(msg, Float32):
            return float(msg.data)

        if isinstance(msg, FloatStamped):
            # handle common field names robustly
            for fld in ("data", "value", "float", "data_float"):
                if hasattr(msg, fld):
                    try:
                        return float(getattr(msg, fld))
                    except Exception:
                        pass
            # fallback to attempt numeric attributes on msg
        if isinstance(msg, PointStamped):
            return (float(msg.point.x), float(msg.point.y), float(msg.point.z))

        if isinstance(msg, Vector3Stamped):
            return (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))

        if isinstance(msg, Vector3):
            return (float(msg.x), float(msg.y), float(msg.z))

        # Generic heuristics (best-effort)
        if hasattr(msg, 'point') and hasattr(msg.point, 'x'):
            return (float(msg.point.x), float(msg.point.y), float(msg.point.z))

        if hasattr(msg, 'vector') and hasattr(msg.vector, 'x'):
            return (float(msg.vector.x), float(msg.vector.y), float(msg.vector.z))

        if all(hasattr(msg, f) for f in ('x', 'y', 'z')):
            try:
                return (float(msg.x), float(msg.y), float(msg.z))
            except Exception:
                pass

        if hasattr(msg, 'data'):
            try:
                return float(msg.data)
            except Exception:
                pass
        return msg
    
    def _get_latest(self, id, topic):
        pass

    def _ctrl_loop(self):
        ctrl = Twist()
        ctrl.linear = Vector3(x=0.5, y=0.0, z=0.0)
        ctrl.angular = Vector3(x=0.0, y=0.0, z=math.radians(10.0))
        self.cmd_vel.publish(ctrl)
        self.get_logger().info(f'Linear: {ctrl.linear}. Angular: {ctrl.angular}')
        self.get_logger().info(f'Latest: {self.latest}')
        


def main(args=None):
    rclpy.init(args=args)
    mp2c = MP2Controller()
    rclpy.spin(mp2c)

    mp2c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()