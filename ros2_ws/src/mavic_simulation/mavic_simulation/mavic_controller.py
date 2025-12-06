import functools
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
    "/cmd_vel_Mavic_2_PRO_ID": Twist #Desired translational and rotational velocities in the global coordinate system. 
}

class MP2Controller(Node):

    def __init__(self):
        #Init Node
        super().__init__('Mavic_2_PRO_ID_Controller')
        
        # Declare params with typed defaults
        self.declare_parameter('NDrones', 4)
        self.declare_parameter('MavicID', 1)
        # Get values
        n_drones = int(self.get_parameter('NDrones').value)
        my_id = int(self.get_parameter('MavicID').value)

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
        freq = 10.0 #Hz
        self.timer = self.create_timer(1.0/freq, self._ctrl_loop)
        

    def _generic_cb(self, msg, topic: str, drone_id: int):
        """
        Generic callback used for all subscriptions.
        Stores the latest message under self.latest[drone_id][topic].
        """
        self.latest.setdefault(drone_id, {})[topic] = msg


        
    def _ctrl_loop(self):
        ctrl = Twist()
        
        #self.cmd_vel.publish(ctrl)
        #self.get_logger().info('Publishing: "%s"' % ctrl.data)
        


def main(args=None):
    rclpy.init(args=args)
    mp2c = MP2Controller()
    rclpy.spin(mp2c)

    mp2c.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()