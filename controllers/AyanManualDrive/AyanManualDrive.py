from controller import Supervisor, Keyboard

class drive_controller(Supervisor):
    def __init__(self):
        super().__init__()
        
        # Get the time step of the current world
        self.timestep = int(self.getBasicTimeStep())
        
        # Initialize keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # Get the motors using the correct names from the device list
        self.left_steer = self.getDevice("left_steer")
        self.right_steer = self.getDevice("right_steer")
        self.left_front_wheel = self.getDevice("left_front_wheel")
        self.right_front_wheel = self.getDevice("right_front_wheel")
        
        # Get brake devices
        self.left_front_brake = self.getDevice("left_front_brake")
        self.right_front_brake = self.getDevice("right_front_brake")
        self.left_rear_brake = self.getDevice("left_rear_brake")
        self.right_rear_brake = self.getDevice("right_rear_brake")
        
        # Set up wheel motors
        self.left_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setPosition(float('inf'))
        
        # Initialize velocities to 0
        self.left_front_wheel.setVelocity(0)
        self.right_front_wheel.setVelocity(0)
        
        # Release all brakes
        self.left_front_brake.setDampingConstant(0)
        self.right_front_brake.setDampingConstant(0)
        self.left_rear_brake.setDampingConstant(0)
        self.right_rear_brake.setDampingConstant(0)
        
        # Constants
        self.max_speed = 30.0  # Maximum wheel speed
        self.max_steering_angle = 0.5  # Maximum steering angle in radians
        self.movement_threshold = 0.01  # Threshold to consider the car as "stuck"
        self.if_crashed = False  # Initialize if_crashed to False
        self.crash_delay = 1.0  # Time (in seconds) required to trigger crash detection
        self.time_stuck = 0.0  # Counter for time spent stuck
        
        # Reference to the robot node for position
        self.robot_node = self.getFromDef("car")
        if self.robot_node is None:
            raise ValueError("Robot node with DEF name 'car' not found.")
        
        # Store the previous position for movement detection
        self.previous_position = self.getPos()
        self.vertical_roads = [
            self.getFromDef('Road_0'),  # Add additional road segments as needed
            self.getFromDef('Road_9'),
            self.getFromDef('Road_10'),
            self.getFromDef('Road_11'),
            self.getFromDef('Road_12'),
            self.getFromDef('Road_13'),

        ]
        self.horizontal_roads = [
            self.getFromDef('Road_1'),
            self.getFromDef('Road_2'),
            self.getFromDef('Road_3'),
            self.getFromDef('Road_4'),
            self.getFromDef('Road_5'),
            self.getFromDef('Road_6'),
            self.getFromDef('Road_7'),
            self.getFromDef('Road_8'),

        ]
        self.road_intersections = [
            self.getFromDef('Road_14'),
            self.getFromDef('Road_15'),
            self.getFromDef('Road_16'),
            self.getFromDef('Road_17'),
        ]
        
        # Camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)


    def is_on_road(self):
        car_position = self.getPos()  # Using self for the car's position
        for road in self.vertical_roads:
            center = road.getPosition()
            length = 15
            width = 7
            # Check if car is within the bounding box of the road segment
            if (center[0] <= car_position[0] <= center[0] + length and
                center[1] - width / 2 <= car_position[1] <= center[1] + width / 2):
                return True
        for road in self.horizontal_roads:
            center = road.getPosition()
            length = 7
            width = 15
            # Check if car is within the bounding box of the road segment
            if (center[0] - length / 2 <= car_position[0] <= center[0] + length / 2 and
                center[1] <= car_position[1] <= center[1] + width):
                return True
                 
        for road in self.road_intersections:
            center = road.getPosition()
            length = 19
            width = 19
            # Check if car is within the bounding box of the road segment
            if (center[0] - length / 2 <= car_position[0] <= center[0] + length / 2 and
                center[1] - width / 2 <= car_position[1] <= center[1] + width / 2):
                return True
        return False

    
    def getPos(self):
        """Return current position of the robot."""
        return self.robot_node.getPosition()
    
    def drive(self, speed, steering):
        """Apply speed and steering to the car."""
        # Apply steering
        self.left_steer.setPosition(steering)
        self.right_steer.setPosition(steering)
        
        # Set wheel velocities
        self.left_front_wheel.setVelocity(speed)
        self.right_front_wheel.setVelocity(speed)
        
        # Apply brakes if no speed is requested
        brake_torque = 1000 if speed == 0 else 0
        self.left_front_brake.setDampingConstant(brake_torque)
        self.right_front_brake.setDampingConstant(brake_torque)
        self.left_rear_brake.setDampingConstant(brake_torque)
        self.right_rear_brake.setDampingConstant(brake_torque)
    
    def detect_crash(self, keys):
        """Detect if the car is crashed or stuck based on movement threshold and duration."""
        # Check if any arrow key is pressed
        if Keyboard.UP in keys or Keyboard.DOWN in keys or Keyboard.LEFT in keys or Keyboard.RIGHT in keys:
            current_position = self.getPos()
            # Calculate the difference between the current and previous position
            movement = [abs(current_position[i] - self.previous_position[i]) for i in range(3)]
            
            # Check if movement is below threshold
            if all(m < self.movement_threshold for m in movement):
                self.time_stuck += self.timestep / 1000.0  # Increase the stuck timer
                if self.time_stuck >= self.crash_delay:
                    self.if_crashed = True
                    print("Crash detected: The car is stuck or minimally moving.")
            else:
                # Reset the stuck timer if the car is moving
                self.time_stuck = 0.0
                self.if_crashed = False
            
            # Update the previous position
            self.previous_position = current_position
        else:
            # Reset the timer if no arrow key is pressed
            self.time_stuck = 0.0
            self.if_crashed = False
    
    def run(self):
        while self.step(self.timestep) != -1:
            # Get keyboard input
            keys = []
            key = self.keyboard.getKey()
            while key != -1:
                keys.append(key)
                key = self.keyboard.getKey()
            
            # Initialize speed and steering
            speed = 0.0
            steering = 0.0
            
            # Handle keyboard input
            if Keyboard.UP in keys:
                speed = self.max_speed
            elif Keyboard.DOWN in keys:
                speed = -self.max_speed
            
            if Keyboard.LEFT in keys:
                steering = -self.max_steering_angle
            elif Keyboard.RIGHT in keys:
                steering = self.max_steering_angle
            
            # Drive the car
            self.drive(speed, steering)
            # Detect crash
            self.detect_crash(keys)
            
            print("Is on road: " + str(self.is_on_road()))
            # Print crash status
            if self.if_crashed:
                print("if_crashed is True")

# Create the robot controller object and run it
controller = drive_controller()
controller.run()
