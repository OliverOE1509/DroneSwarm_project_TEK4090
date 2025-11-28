from controller import Robot, GPS, InertialUnit, Compass, DistanceSensor, Emitter, Receiver
import math
import json

class AdvancedDroneController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.drone_id = self.robot.getName()
        
        # Initialize all sensors
        self.initialize_sensors()
        self.initialize_communication()
        
        # Mission parameters
        self.target_positions = [
            [25, -25, 2],   # Red flag
            [-25, 25, 2],   # Blue flag  
        ]
        
        # Control state
        self.current_target = 0
        self.mission_complete = False
        self.obstacle_detected = False
        
        # Performance metrics
        self.start_time = self.robot.getTime()
        self.distance_traveled = 0
        self.last_position = self.get_position()
        
        print(f"{self.drone_id}: Controller initialized with full sensor suite")

    def initialize_sensors(self):
        """Initialize all drone sensors - safely handle missing devices"""
        drone_num = self.drone_id.split('_')[1]
        
        # Navigation sensors
        self.gps = self.robot.getDevice("gps_" + drone_num)
        self.imu = self.robot.getDevice("imu_" + drone_num)
        self.compass = self.robot.getDevice("compass_" + drone_num)
        
        # Proximity sensors - safely initialize only available ones
        self.front_sensor = self.robot.getDevice("front_sensor_" + drone_num)
        
        # Try to get optional sensors, but don't crash if they don't exist
        self.back_sensor = self.get_optional_device("back_sensor_" + drone_num)
        self.left_sensor = self.get_optional_device("left_sensor_" + drone_num) 
        self.right_sensor = self.get_optional_device("right_sensor_" + drone_num)
        
        # Enable all available sensors
        self.gps.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.front_sensor.enable(self.timestep)
        
        if self.back_sensor:
            self.back_sensor.enable(self.timestep)
        if self.left_sensor:
            self.left_sensor.enable(self.timestep)
        if self.right_sensor:
            self.right_sensor.enable(self.timestep)

    def get_optional_device(self, device_name):
        """Safely get a device that might not exist"""
        try:
            return self.robot.getDevice(device_name)
        except:
            print(f"Warning: Device {device_name} not found on {self.drone_id}")
            return None

    def initialize_communication(self):
        """Initialize communication devices"""
        drone_num = self.drone_id.split('_')[1]
        self.emitter = self.robot.getDevice("emitter_" + drone_num)
        self.receiver = self.robot.getDevice("receiver_" + drone_num)
        self.receiver.enable(self.timestep)

    def get_position(self):
        """Get current drone position"""
        return self.gps.getValues()

    def get_orientation(self):
        """Get drone orientation as quaternion"""
        return self.imu.getQuaternion()

    def get_heading(self):
        """Get compass heading in radians"""
        north = self.compass.getValues()
        return math.atan2(north[0], north[1])

    def get_obstacle_distances(self):
        """Get distances to obstacles in all directions"""
        distances = {
            'front': self.front_sensor.getValue(),
            'back': 500.0,  # Default max distance
            'left': 500.0,
            'right': 500.0
        }
        
        # Only include sensors that exist
        if self.back_sensor:
            distances['back'] = self.back_sensor.getValue()
        if self.left_sensor:
            distances['left'] = self.left_sensor.getValue()
        if self.right_sensor:
            distances['right'] = self.right_sensor.getValue()
            
        return distances

    def calculate_distance(self, pos1, pos2):
        """Calculate 3D distance between positions"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

    def avoid_obstacles(self, desired_direction, obstacles):
        """Simple obstacle avoidance behavior"""
        safe_direction = list(desired_direction)
        
        # Adjust direction based on obstacle proximity
        if obstacles['front'] < 1.0:  # Too close to front obstacle
            safe_direction[0] *= 0.5  # Reduce forward speed
            if obstacles['left'] > obstacles['right']:
                safe_direction[1] = 0.3  # Turn left
            else:
                safe_direction[1] = -0.3  # Turn right
                
        return safe_direction

    def navigate_to_target(self, target_position):
        """Advanced navigation with obstacle avoidance"""
        current_pos = self.get_position()
        heading = self.get_heading()
        obstacles = self.get_obstacle_distances()
        
        # Calculate direction to target
        dx = target_position[0] - current_pos[0]
        dy = target_position[1] - current_pos[1]
        dz = target_position[2] - current_pos[2]
        
        # Calculate desired heading
        target_heading = math.atan2(dy, dx)
        heading_error = target_heading - heading
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
            
        # Calculate control outputs
        distance_to_target = self.calculate_distance(current_pos, target_position)
        
        # Base control values
        forward_vel = min(2.0, distance_to_target * 0.5)  # P-controller
        vertical_vel = dz * 0.8  # Altitude control
        yaw_vel = heading_error * 1.5  # Heading control
        
        # Apply obstacle avoidance
        control_vector = [forward_vel, yaw_vel, vertical_vel]
        safe_control = self.avoid_obstacles(control_vector, obstacles)
        
        return safe_control

    def update_metrics(self):
        """Update performance metrics"""
        current_pos = self.get_position()
        distance = self.calculate_distance(current_pos, self.last_position)
        if not math.isnan(distance):
            self.distance_traveled += distance
        self.last_position = current_pos

    def print_status(self):
        """Print comprehensive drone status"""
        pos = self.get_position()
        obstacles = self.get_obstacle_distances()
        
        print(f"\n=== {self.drone_id} Status ===")
        print(f"Position: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        print(f"Heading: {math.degrees(self.get_heading()):.1f}°")
        print(f"Target: {self.current_target + 1}")
        print(f"Obstacles - F:{obstacles['front']:.1f} L:{obstacles['left']:.1f} R:{obstacles['right']:.1f}")
        print(f"Distance traveled: {self.distance_traveled:.1f}m")
        print(f"Mission time: {self.robot.getTime() - self.start_time:.1f}s")

    def run(self):
        """Main control loop"""
        status_counter = 0
        
        while self.robot.step(self.timestep) != -1 and not self.mission_complete:
            # Update metrics
            self.update_metrics()
            
            # Navigation
            target_pos = self.target_positions[self.current_target]
            control = self.navigate_to_target(target_pos)
            
            # Check if target reached
            current_pos = self.get_position()
            if self.calculate_distance(current_pos, target_pos) < 2.0:
                print(f"{self.drone_id}: Reached target {self.current_target + 1}!")
                self.current_target = (self.current_target + 1) % len(self.target_positions)
            
            # Print status periodically
            if status_counter % 100 == 0:
                self.print_status()
                
            status_counter += 1

# Create and run the controller
controller = AdvancedDroneController()
controller.run()