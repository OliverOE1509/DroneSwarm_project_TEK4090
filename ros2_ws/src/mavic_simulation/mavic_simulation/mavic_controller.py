import functools, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3Stamped, PointStamped, Vector3
from std_msgs.msg import Float32
from webots_ros2_msgs.msg import FloatStamped
import numpy as np
from std_msgs.msg import Int32

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
STATE_TOPIC_NAMES = [i.split("/")[-1]  for i in STATE_TOPICS.keys()]
CONTROL_TOPICS = {
    "/cmd_vel_Mavic_2_PRO_ID": Twist #Desired translational and rotational velocities in the body frame coordinate system. 
}

class MP2Controller(Node):

    def __init__(self):
        #Init Node
        super().__init__('Mavic_2_PRO_ID_Controller')
        
        # Declare params with typed defaults
        self.declare_parameter('NDrones', 1)
        self.declare_parameter('MavicID', 1)
        self.declare_parameter('loop_freq_hz', 10.0)
        # Get values
        n_drones = int(self.get_parameter('NDrones').value)
        my_id = int(self.get_parameter('MavicID').value)
        freq_hz = int(self.get_parameter('loop_freq_hz').value)

        # Attempt at making swarm formation, as function of # of drones
        #self.formation_offsets = self.spherical_offsets(n_drones, radius = 2.5)

        self.declare_parameter('z_ref', 1.5)  # fallback
        self.z_ref = float(self.get_parameter('z_ref').value)
        self._z_ref_initialized = False 

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

        # I create a leader ID (1 to N) to identify which drone is the leader at each consensus step.
        # The leader will be the one touching the flag
        self.leader_id = None
        # After an event (either time based or flag respawned), the swarm will re-calculate consensus, and new leader will emerge
        self.last_consensus_time = self.get_clock().now()
        self.flag_pos = None

        self.F_previous = (0.0, 0.0, 0.0)
        self.alpha = 0.3
        self.F_max = 8.0

        self.z_ref_target = None
        self.z_ref_current = None
        self.z_transition_rate = 0.5  # m/s max transition rate

        self.k_att_base = 2.0  # Base attraction gain
        self.k_att_current = self.k_att_base
        self.rho_g = 2.0  # Goal proximity threshold
        self.lambda_gain = 1.5  # Moderate gain within goal proximity
        self.tau_gain = 2.0  # Amplified gain when far but no obstacles

        # Create subscription to flag position
        self.flag_sub = self.create_subscription(
            PointStamped,
            "/flag/gps",
            self._flag_cb,
            10
        )

        # Subscription for swarm manager
        self.leader_sub = self.create_subscription(
            Int32,
            '/swarm/leader',
            self._leader_cb,
            10
        )
        
        # create control loop. 
        self.timer = self.create_timer(1.0/freq_hz, self._ctrl_loop)
    
    def _flag_cb(self, msg):
        self.flag_pos = (
            float(msg.point.x),
            float(msg.point.y),
            float(msg.point.z)
        )
    
    def _leader_cb(self, msg):
        self.leader_id = msg.data
    
    def _get_my_position(self):
        my_id = int(self.get_parameter('MavicID').value)
        gps_topic = f'/Mavic_2_PRO_{my_id}/gps'
        return self.latest.get(my_id, {}).get(gps_topic, None)
    
    def _get_drone_position(self, drone_id):
        gps_topic = f'/Mavic_2_PRO_{drone_id}/gps'
        return self.latest.get(drone_id, {}).get(gps_topic, None)

    def _generic_cb(self, msg, topic: str, drone_id: int):
        """
        Generic callback used for all subscriptions.
        Stores the latest message under self.latest[drone_id][topic].
        """
        pyval = self._unpack_msg(msg)
        self.latest.setdefault(drone_id, {})[topic] = pyval
        if len(self.latest[drone_id]) > 10:  # Keep only 10 latest topics
            keys = list(self.latest[drone_id].keys())
            for key in keys[:-10]:  # Remove oldest
                del self.latest[drone_id][key]

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


    def _norm3(self, v):
        return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

    def _unit3(self, v, eps=1e-9):
        n = self._norm3(v)
        if n < eps:
            return (0.0, 0.0, 0.0), 0.0
        return (v[0]/n, v[1]/n, v[2]/n), n

    def attraction_velocity(self, pos, goal, v_att_max=2.0, d0=3.0, slow_radius=1.5):
        # vector to goal
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        dz = goal[2] - pos[2]
        u, d = self._unit3((dx, dy, dz))

        # base speed curve: v = v_att_max * tanh(d/d0)
        v = v_att_max * math.tanh(d / max(1e-6, d0))

        # soften attraction close to goal to prevent pile-up
        # scale -> 0 as d->0, scale ~1 outside slow_radius
        if d < slow_radius:
            v *= (d / max(1e-6, slow_radius))

        return (v * u[0], v * u[1], v * u[2])

    def repulsion_velocity(self, pos_i, pos_j, R=2.0, v_rep_max=1.2, p=2.0):
        # vector away from j
        rx = pos_i[0] - pos_j[0]
        ry = pos_i[1] - pos_j[1]
        rz = pos_i[2] - pos_j[2]
        u, d = self._unit3((rx, ry, rz))
        if d <= 1e-6 or d >= R:
            return (0.0, 0.0, 0.0)

        # smooth bounded gain in [0, v_rep_max]:
        # g = v_rep_max * ( (1/d - 1/R) / (1/(eps) - 1/R) )^p  (normalized)
        # simpler and robust:
        s = (R - d) / R              # 0 at boundary, 1 at d=0
        g = v_rep_max * (s ** p)     # shape exponent p>=1

        return (g * u[0], g * u[1], g * u[2])
    
    def density_scale(self, pos_i, neighbor_positions, R=2.0, beta=0.4):
        # N_eff counts nearby drones with distance weighting
        N_eff = 0.0
        for pj in neighbor_positions:
            dx = pos_i[0] - pj[0]
            dy = pos_i[1] - pj[1]
            dz = pos_i[2] - pj[2]
            d = np.sqrt(dx*dx + dy*dy + dz*dz)
            if 1e-6 < d < R:
                N_eff += np.exp(-d / (0.5*R))
        return 1.0 + beta * N_eff

    
    def yaw_gain(self, d_goal, k_far=1.2, k_near=0.4, d_switch=2.0):
        # smooth interpolation using tanh
        a = math.tanh(d_goal / max(1e-6, d_switch))
        return k_near + (k_far - k_near) * a


    def _ctrl_loop(self):
        my_id = int(self.get_parameter('MavicID').value)
        n_drones = int(self.get_parameter('NDrones').value)
        freq_hz = int(self.get_parameter('loop_freq_hz').value)

        v_xy_max = 2.0
        v_z_max  = 1.0

        pos = self._get_my_position()
        if pos is None or self.flag_pos is None:
            return
        if not self._z_ref_initialized:
            self.z_ref = pos[2]
            self._z_ref_initialized = True

        if not hasattr(self, '_z_ref_initialized') or not self._z_ref_initialized:
            self.z_ref = pos[2]
            self._z_ref_initialized = True
        goal = self.flag_pos

        # ---------- Compass / orientation ----------
        north_topic = f"/Mavic_2_PRO_{my_id}/compass/north_vector"
        north_vec = self.latest.get(my_id, {}).get(north_topic, None)
        if north_vec is None:
            return

        nx, ny, _ = north_vec
        nrm = math.hypot(nx, ny)
        if nrm < 1e-6:
            return
        nx /= nrm
        ny /= nrm

        # ============================================================
        # 1) ATTRACTION (WORLD FRAME, BOUNDED)
        # ============================================================
        dx = goal[0] - pos[0]
        dy = goal[1] - pos[1]
        dz = goal[2] - pos[2]
        d_goal = np.sqrt(dx*dx + dy*dy + dz*dz)

        # direction
        if d_goal > 1e-6:
            ux, uy, uz = dx/d_goal, dy/d_goal, dz/d_goal
        else:
            ux, uy, uz = 0.0, 0.0, 0.0

        v_att_max = 2.0      # [m/s]
        d0 = 3.0             # saturation distance
        slow_radius = 1.5    # brake near goal

        #under_repulsion = False
        #for pj in neighbour_positions:
        #    d = np.sqrt((pos[0]-pj[0])**2 + (pos[1]-pj[1])**2 + (pos[2]#-pj[2])**2)
        #    if d < 5.0:  # Repulsion influence radius
        #        under_repulsion = True
        #        break

        # Adaptive gain (simplified version of Eq. 18)
        """ if under_repulsion:
            k_att = self.k_att_base  # Constant when near obstacles
        elif d_goal < self.rho_g:
            k_att = self.lambda_gain * self.k_att_base  # Moderate near goal
        else:
            # Inverse scaling with distance (prevents excessive force when far)
            k_att = min(self.tau_gain * self.k_att_base / max(d_goal, 0.1), 
                        self.tau_gain * self.k_att_base) """

        # Apply adaptive gain to attraction
        v_att = v_att_max * math.tanh(d_goal / d0)
        if d_goal < slow_radius:
            v_att *= d_goal / slow_radius

        vwx = v_att * ux
        vwy = v_att * uy
        vwz = 1 * v_att * uz   # softer vertical motion

        # ============================================================
        # 2) REPULSION (WORLD FRAME, BOUNDED)
        # ============================================================
        R = 5.0           # influence radius
        v_rep_max = 1.2   # max repulsion speed
        p = 2.0           # shape exponent

        neighbour_positions = []
        for j in range(1, n_drones + 1):
            if j == my_id:
                continue
            pj = self.latest.get(j, {}).get(f"/Mavic_2_PRO_{j}/gps", None)
            if pj is not None:
                neighbour_positions.append(pj)
        
        scale = self.density_scale(pos, neighbour_positions, R=R, beta=0.4)
        scale = min(scale, 3.0)  # cap max scaling

        for pj in neighbour_positions:
            if my_id == self.leader_id:
                continue # To avoid making a formation around a flag, so repulsion only applies to followers
            rx = pos[0] - pj[0]
            ry = pos[1] - pj[1]
            rz = pos[2] - pj[2]
            d = np.sqrt(rx*rx + ry*ry + rz*rz)

            if d < 1e-6 or d >= R:
                continue

            ux = rx / d
            uy = ry / d
            uz = rz / d

            s = (R - d) / R
            if my_id == self.leader_id:
                # Leader repels less strongly from followers
                rep_strength = v_rep_max * 0.5
            elif j == self.leader_id:  # This follower is repelling from leader
                # Followers repel more strongly from leader
                rep_strength = v_rep_max * 0.8
            else:
                # Follower-follower repulsion
                rep_strength = v_rep_max
            
            g = rep_strength * (s ** p) * scale

            vwx += g * ux
            vwy += g * uy
            vwz += g * uz

        F_world = (vwx, vwy, vwz)
        F_smooth = (
            self.alpha * self.F_previous[0] + (1 - self.alpha) * F_world[0],
            self.alpha * self.F_previous[1] + (1 - self.alpha) * F_world[1],
            self.alpha * self.F_previous[2] + (1 - self.alpha) * F_world[2]
        )
        self.F_previous = F_smooth

        force_mag = math.sqrt(F_smooth[0]**2 + F_smooth[1]**2 + F_smooth[2]**2)
        if force_mag > self.F_max:
            scale = self.F_max / force_mag
            F_smooth = (
                F_smooth[0] * scale,
                F_smooth[1] * scale,
                F_smooth[2] * scale
            )
        
        # Update velocities with smoothed force
        vwx, vwy, vwz = F_smooth

        # ============================================================
        # 3) WORLD → BODY TRANSFORM
        # ============================================================
        # north_vector = world +Y in body frame
        # rotation angle body->world:
        psi = math.atan2(nx, ny)
        c = np.cos(psi)
        s = np.sin(psi)

        vbx =  c * vwx + s * vwy
        vby = -s * vwx + c * vwy

        k_p_z = 1.2 # climb rate per meter error
        k_d_z = 0.6 # damping using measured vertical speed
        vz_max_hold = 1.0
        z_ref = self.z_ref
        if my_id == self.leader_id:
            self.z_ref_target = goal[2]
        else:
            if self.leader_id is not None:
                leader_pos = self._get_drone_position(self.leader_id)
                if leader_pos is not None:
                    self.z_ref_target = leader_pos[2]
                else:
                    self.z_ref_target = self.z_ref
            else:
                self.z_ref_target = self.z_ref

        if self.z_ref_current is None:
            self.z_ref_current = pos[2]
        
        dz = self.z_ref_target - self.z_ref_current
        max_dz = self.z_transition_rate * (1.0 / freq_hz)  # Max change per timestep
        if abs(dz) > max_dz:
            dz = math.copysign(max_dz, dz)
        self.z_ref_current += dz

        # Use smoothed reference for control
        z_err = self.z_ref_current - pos[2]
        vz_meas = 0.0
        sv_topic = f"/Mavic_2_PRO_{my_id}/gps/speed_vector"
        sv = self.latest.get(my_id, {}).get(sv_topic, None)
        if sv is not None:
            vz_meas = float(sv[2])

        vwz_hold = k_p_z * z_err - k_d_z * vz_meas
        vwz_hold = max(-vz_max_hold, min(vz_max_hold, vwz_hold))

        if my_id == self.leader_id:
            # Leader: go to flag in Z
            vwz = max(-v_z_max, min(v_z_max, vwz))
        else:
            # Followers: STRICT altitude hold only
            vwz = vwz_hold

        # ============================================================
        # 4) SATURATION
        # ============================================================

        vxy = math.hypot(vbx, vby)
        if vxy > v_xy_max:
            scale = v_xy_max / vxy
            vbx *= scale
            vby *= scale

        vwz = max(-v_z_max, min(v_z_max, vwz))

        # ============================================================
        # 5) YAW CONTROL (GAIN-SCHEDULED, STABLE)
        # ============================================================

        vxy = math.hypot(vbx, vby)

        # Heading error only meaningful if we are actually moving
        if vxy > 0.15:
            theta_err = math.atan2(vby, vbx)   # body-frame heading error
        else:
            theta_err = 0.0

        # ---------- Gain scheduling ----------
        k_yaw_max = 1.2        # maximum yaw rate gain
        theta0    = 0.6        # [rad] where yaw gain saturates
        v0        = 0.8        # [m/s] speed at which yaw fully activates
        yaw_max   = 0.8        # [rad/s] hard safety limit

        angle_gain = math.tanh(abs(theta_err) / theta0)
        speed_gain = min(1.0, vxy / v0)
        yaw_rate = k_yaw_max * angle_gain * speed_gain * math.copysign(1.0, theta_err)
        yaw_rate = max(-yaw_max, min(yaw_max, yaw_rate))

        # ============================================================
        # 6) COMMAND
        # ============================================================
        if not all(math.isfinite(x) for x in [vbx, vby, vwz, yaw_rate]):
            self.get_logger().error(f"Drone {self.my_id}: Non-finite control values! Resetting to zero.")
            vbx, vby, vwz, yaw_rate = 0.0, 0.0, 0.0, 0.0

        self.get_logger().info(
            f"[DEBUG] id={my_id} pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
            f"leader={self.leader_id} "
            f"goal=({goal[0]:.2f},{goal[1]:.2f},{goal[2]:.2f}) "
            #f"close_to={close_drones}"
        )
        ctrl = Twist()
        ctrl.linear = Vector3(x=float(vbx), y=float(vby), z=float(vwz))
        ctrl.angular = Vector3(x=0.0, y=0.0, z=float(yaw_rate))
        self.cmd_vel.publish(ctrl)
        
        
def main(args=None):
    rclpy.init(args=args)
    mp2c = MP2Controller()
    rclpy.spin(mp2c)
    mp2c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()