from controller import Robot, GPS, InertialUnit, Compass, Camera, Motor
import math

class MavicController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Get devices
        self.gps = self.robot.getDevice("gps")
        self.imu = self.robot.getDevice("inertial unit")
        self.compass = self.robot.getDevice("compass")
        self.camera = self.robot.getDevice("camera")
        
        # Get motors - Mavic has 4 motors
        self.motors = []
        for i in range(4):
            motor = self.robot.getDevice(f"motor{i+1}")
            self.motors.append(motor)
        
        # Enable devices
        self.gps.enable(self.timestep)
        self.imu.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.camera.enable(self.timestep)
        
        # Target positions (you can modify these)
        self.targets = [
            [1, 1, 10],    # x, y, z
            [-1, -1, 15],
            [2, 0, 12]
        ]
        self.current_target = 0
        
        # PID gains (you'll need to tune these)
        self.kp_xy = 0.5   # Position control
        self.kp_z = 1.0    # Altitude control  
        self.kp_yaw = 2.0  # Heading control
        
        print("Mavic 2 Pro controller initialized!")

    def get_position(self):
        """Get current drone position [x, y, z]"""
        return self.gps.getValues()

    def get_orientation(self):
        """Get roll, pitch, yaw from IMU"""
        return self.imu.getRollPitchYaw()

    def get_heading(self):
        """Get compass heading in radians"""
        north = self.compass.getValues()
        return math.atan2(north[0], north[1])

    def calculate_distance(self, pos1, pos2):
        """Calculate distance to target"""
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(pos1, pos2)))

    def position_control(self, target_pos):
        """Simple position controller for Mavic"""
        current_pos = self.get_position()
        orientation = self.get_orientation()
        heading = self.get_heading()
        
        # Calculate errors
        error_x = target_pos[0] - current_pos[0]
        error_y = target_pos[1] - current_pos[1] 
        error_z = target_pos[2] - current_pos[2]
        
        # Calculate target heading
        target_heading = math.atan2(error_y, error_x)
        heading_error = target_heading - heading
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Base control outputs
        forward = error_x * self.kp_xy
        lateral = error_y * self.kp_xy  
        vertical = error_z * self.kp_z
        yaw = heading_error * self.kp_yaw
        
        # Convert to motor commands (simplified)
        # Real quadcopter mixing would be more complex
        base_thrust = 60.0  # Base motor speed
        
        # Simple mixing (you may need to adjust signs based on motor layout)
        motor_commands = [
            base_thrust + vertical - forward + lateral + yaw,  # motor1
            base_thrust + vertical - forward - lateral - yaw,  # motor2  
            base_thrust + vertical + forward - lateral + yaw,  # motor3
            base_thrust + vertical + forward + lateral - yaw   # motor4
        ]
        
        return motor_commands

    def run(self):
        """Main control loop"""
        step_count = 0
        
        while self.robot.step(self.timestep) != -1:
            current_pos = self.get_position()
            target_pos = self.targets[self.current_target]
            
            # Calculate motor commands
            motor_commands = self.position_control(target_pos)
            
            # Apply motor commands
            for i, motor in enumerate(self.motors):
                motor.setVelocity(motor_commands[i])
            
            # Check if target reached
            distance = self.calculate_distance(current_pos, target_pos)
            if distance < 1.0:  # Within 1 meter
                print(f"Reached target {self.current_target + 1}!")
                self.current_target = (self.current_target + 1) % len(self.targets)
            
            # Print status every 100 steps
            if step_count % 100 == 0:
                print(f"Position: [{current_pos[0]:.1f}, {current_pos[1]:.1f}, {current_pos[2]:.1f}]")
                print(f"Target: {self.current_target + 1}, Distance: {distance:.1f}m")
                print("---")
            
            step_count += 1

# Create and run controller
controller = MavicController()
controller.run()