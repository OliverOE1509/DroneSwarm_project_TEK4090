from geometry_msgs.msg import Twist, Vector3Stamped, PointStamped, Vector3
from webots_ros2_msgs.msg import FloatStamped
from std_msgs.msg import Float32, Int32
import functools, math, rclpy
from rclpy.node import Node
import numpy as np


STATE_TOPICS  = {
    "/Mavic_2_PRO_ID/compass/bearing": FloatStamped,        #The drone's yaw/angle relative to the north_vector in degrees. 
    "/Mavic_2_PRO_ID/compass/north_vector": Vector3Stamped, #A vector pointing towards the north pole relative to the drone's body frame.
    "/Mavic_2_PRO_ID/gps": PointStamped,              #The drone's position in the global coordinate system.
    "/Mavic_2_PRO_ID/gps/speed": Float32,             #The magnitude of the drone's translational velocity vector in the global coordinate system.
    "/Mavic_2_PRO_ID/gps/speed_vector": Vector3,      #The drone's translational velocity vector in the global coordinate system
}
FLAG_TOPIC = {
    "/flag/gps": PointStamped  #The target position of the flag in the global coordinate system.
}
CONTROL_TOPICS = {
    "/cmd_vel_Mavic_2_PRO_ID": Twist #Desired translational and rotational velocities in the body frame coordinate system. 
}

class MP2Controller(Node):

    def __init__(self):
        #Init Node
        super().__init__('Mavic_2_PRO_ID_Controller')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Declare params with typed defaults
        self.declare_parameter('NDrones', 1)
        self.declare_parameter('MavicID', 1)
        self.declare_parameter('loop_freq_hz', 10.0)

        # Get values
        n_drones = int(self.get_parameter('NDrones').value)
        my_id = int(self.get_parameter('MavicID').value)
        freq_hz = int(self.get_parameter('loop_freq_hz').value)
        self.my_id = my_id
        self.n_drones = n_drones
        self.flag_pos = None

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

        # Create subscription to flag position
        self.flag_sub = self.create_subscription(
            PointStamped,
            "/flag/gps",
            self._flag_cb,
            10
        )

        # create control loop. 
        self.timer = self.create_timer(1.0/freq_hz, self._ctrl_loop)
    
    def _flag_cb(self, msg):
        self.flag_pos = self._unpack_msg(msg)
    
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
    
    def help_debug(self, msg):
        return self.get_logger().info(f'{msg}')

    def _get_latest(self, topic, drone_id):
        for tmpl in STATE_TOPICS.keys():
            if tmpl.endswith(f'/{topic}'):
                top = tmpl.replace("ID", str(drone_id))
                return self.latest.get(drone_id, {}).get(top, None)

    def _norm3(self, v):
        return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

    def _unit3(self, v, eps=1e-9):
        n = self._norm3(v)
        if n < eps:
            return (0.0, 0.0, 0.0), 0.0
        return (v[0]/n, v[1]/n, v[2]/n), n
    
    def _dist(self, x1, x2):
        return self._norm3(np.asarray(x1) - np.asarray(x2))

    def angle_between(self, v1, v2):
        v1 = np.asarray(v1, dtype=float)
        v2 = np.asarray(v2, dtype=float)

        dot = np.dot(v1, v2)
        n1 = np.linalg.norm(v1)
        n2 = np.linalg.norm(v2)

        # guard against numerical drift
        cos_theta = dot / (n1 * n2)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)

        return np.arccos(cos_theta)
    
    def rotate_vector_in_plane(self, v1, v2, theta, eps=1e-8):
        v1 = np.asarray(v1, dtype=float)
        v2 = np.asarray(v2, dtype=float)

        # debug theta
        self.help_debug(f'theta: {theta}')

        n1 = self._norm3(v1)
        if n1 < eps:
            raise ValueError("v1 must be non-zero")

        e1 = v1 / n1

        # orthogonalize v2 against v1
        v2o = v2 - np.dot(e1, v2) * e1
        n2 = self._norm3(v2o)
        e2 = v2o / n2

        return e1 * np.cos(theta) + e2 * np.sin(theta)


    def _attraction_velocity(self, zeta, target_pos, agent_pos):
        tp = np.asarray(target_pos)
        ap = np.asarray(agent_pos)
        return (tp-ap)*zeta
       
    def _repulsion_velocity(self, eta, cutoff, obstacle_pos, agent_pos):
        op = np.asarray(obstacle_pos)
        ap = np.asarray(agent_pos)
        ir = op-ap
        r = self._norm3(ir)
        
        if r > cutoff or r < 1e-6:
            return np.asarray([0.0,0.0,0.0])
        
        return eta * ( (1/cutoff) - (1/r) ) * (1/r**3) * ir
    
           
    def _get_lin_vel_vector(self, zeta, eta, cutoff):
        v = np.asarray([0.0,0.0,0.0])
        att = True
        my_pos = self._get_latest("gps", self.my_id)
        for i in range(1, self.n_drones + 1):
            if i != self.my_id:
                i_pos = self._get_latest("gps", i)
                v += self._repulsion_velocity(eta, cutoff, i_pos, my_pos) 
                if self._dist(i_pos, self.flag_pos) <= cutoff:
                    att = False
        if att:
            v += self._attraction_velocity(zeta, self.flag_pos, my_pos)
        return v
    
    def _constrain_lin_vel_vector(self, v):
        curr_v = self._get_latest("speed_vector", self.my_id)
        v_xy_max = 1.0
        v_z_max  = 1.0
        dtheta_max = np.pi/2 #Radians. 
        dv_max = 1 
        vx ,vy, vz = v

        #Enforce max velocity along z-axis
        if abs(vz) > v_z_max:
            vz = np.sign(vz)*v_z_max

        #Enforce max velocity in the xy-plane
        xynorm = math.sqrt(vx**2 + vy**2)
        if xynorm > v_xy_max:
            adj = v_xy_max/xynorm
            vx = vx * adj
            vy = vy * adj 

        v = np.asarray([vx,vy,vz])
        if self._norm3(curr_v) < 1e-6:
            return v

        #Enforce max change in velocity
        dv = v - np.asarray(curr_v)
        dv_norm = self._norm3(dv)
        if dv_norm > dv_max:
            v = np.asarray(curr_v) + dv * (dv_max / dv_norm)

        #Enforce max change in direction of travel
        if self.angle_between(v, curr_v) > dtheta_max:
            v = self.rotate_vector_in_plane(curr_v, v, dtheta_max)

        return v
        
    def _world_to_body(self, v):
        north_vec = self._get_latest("north_vector", self.my_id)
        nx, ny = north_vec[0], north_vec[1]
        psi = math.atan2(nx, ny)  # world→body yaw
        c = math.cos(psi)
        s = math.sin(psi)
        vbx =  c * v[0] + s * v[1]
        vby = -s * v[0] + c * v[1]
        vbz = v[2]  # unchanged (flat assumption)
        return np.asarray([vbx,vby,vbz])
    
    def _get_angular(self, v_b, gamma):
        vx, vy = v_b[0], v_b[1]

        # If no horizontal motion, do not rotate
        if vx == 0.0 and vy == 0.0:
            return 0.0

        # Yaw error in radians
        theta_err = math.atan2(vy, vx)

        # Scale by gain
        w_z = gamma * theta_err

        return w_z
    
    def debug(self, v_b, v, w):
        
        flag_msg = f'flag: {self.flag_pos}'
        control_input_body = f'BODY id: {self.my_id}, v_b: {v_b}, angular: {w}'
        control_input_world = f'WORLD id: {self.my_id}, v: {v}'
        gps_msg = f'id: {self.my_id}, gps: {self._get_latest("gps", self.my_id)}'
        vel_msg = f'id: {self.my_id}, vel: {self._get_latest("speed_vector", self.my_id)}'
        timestamp = f'{self.get_clock().now()}'
        self.get_logger().debug(timestamp)
        self.get_logger().debug(flag_msg)
        self.get_logger().debug(control_input_body)
        self.get_logger().debug(control_input_world)
        self.get_logger().debug(gps_msg)
        self.get_logger().debug(vel_msg)
    

    def _ctrl_loop(self): 

        if self.flag_pos is None:
            return
        #Get the desired linear velocity vector in world frame coordinates
        v = self._get_lin_vel_vector(zeta=0.5, eta=2, cutoff=1.0)
        #Constrain the velocity vector
        v = self._constrain_lin_vel_vector(v)
        #Convert velocity vector to body frame
        v_b = self._world_to_body(v)
        #Get angular velocity 
        w = self._get_angular(v_b, gamma=1)
        # log inputs and outputs
        self.debug(v_b, v, w)
        #Publish control
        ctrl = Twist()
        ctrl.linear = Vector3(x=float(v_b[0]), y=float(v_b[1]), z=float(v_b[2]))
        ctrl.angular = Vector3(x=0.0, y=0.0, z=float(w))
        self.cmd_vel.publish(ctrl)
        
        
def main(args=None):
    rclpy.init(args=args)
    mp2c = MP2Controller()
    rclpy.spin(mp2c)
    mp2c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()