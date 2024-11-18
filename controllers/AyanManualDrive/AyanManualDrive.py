from controller import Supervisor, Keyboard
from model import getInstructions
import time
import random

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
        self.speed = 0.0
        self.steering_angle = 0.0
        self.max_speed = 30.0  # Maximum wheel speed
        self.max_steering_angle = 0.5  # Maximum steering angle in radians
        self.movement_threshold = 0.01  # Threshold to consider the car as "stuck"
        self.if_crashed = False  # Initialize if_crashed to False
        self.crash_delay = 1.0  # Time (in seconds) required to trigger crash detection
        self.time_stuck = 0.0  # Counter for time spent stuck
        self.manual_drive = False
        self.time_start = 0.0
        
        # Reference to the robot node for position
        self.robot_node = self.getFromDef("car")
        if self.robot_node is None:
            raise ValueError("Robot node with DEF name 'car' not found.")
        
        # Store the previous position for movement detection
        self.previous_position = self.getPos()
        self.vertical_roads = [
            self.getFromDef('Road_0'),
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
        
        self.roads = []
        for road in self.vertical_roads:
            self.roads.append(road)
        for road in self.horizontal_roads:
            self.roads.append(road)
        for road in self.road_intersections:
            self.roads.append(road)

        # Camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.end_obj = self.getFromDef("redEnd")
        self.end_pos = self.end_obj.getPosition()
        self.distance_away = 0
        


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
            bigLength = 19
            bigWidth = 19
            length = 7
            width = 7
            # Check if car is within the bounding box of the road segment
            if ((center[0] - length / 2 <= car_position[0] <= center[0] + length / 2 or
                center[1] - width / 2 <= car_position[1] <= center[1] + width / 2) and 
                (center[0] - bigLength / 2 <= car_position[0] <= center[0] + bigLength / 2 and
                center[1] - bigWidth / 2 <= car_position[1] <= center[1] + bigWidth / 2)):
                return True
        return False

    
    def get_random_road_position(self):
        """
        Selects a random road segment and returns its translation position.
        """
        selected_road = random.choice(self.roads)
        road_position = selected_road.getField("translation").getSFVec3f()
        return road_position

    def teleport_object(self, object_def, position):
        """
        Teleports a specific object to the given position.
        """
        obj = self.getFromDef(object_def)
        if obj:
            obj.getField("translation").setSFVec3f(position)
            
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
   
    def reset(self):
        self.robot_node.resetPhysics()
        carPos = self.get_random_road_position()
        endPos = self.get_random_road_position()
        while carPos == endPos:
            carPos = self.get_random_road_position()
            endPos = self.get_random_road_position()
        self.teleport_object("car", carPos)
        self.teleport_object("redEnd", endPos)
        self.distance_away = self.shortestPath(carPos, endPos)
        self.end_pos = endPos
        self.start_time = time.time()
        self.robot_node.getField("rotation").setSFRotation([0, 0, 1, random.randint(0, 6)])
    
    def reachedEndPosition(self):
        car_pos = self.getPos()
        if ((self.end_pos[0] - 5 <= car_pos[0] <= self.end_pos[0] + 5) and
            (self.end_pos[1] - 5 <= car_pos[1] <= self.end_pos[1] + 5)):
            return True
        return False
        
    def shortestPath(self, carPos, endPos):
        if carPos[0] == endPos[0]:
            for road in self.road_intersections:
                if carPos[0] == road.getField("translation").getSFVec3f()[0]:
                    return abs(carPos[0] - endPos[0]) + abs(carPos[1] - endPos[1])
            return abs(carPos[0] - endPos[0]) + abs(carPos[1] - endPos[1]) + 30
        if carPos[1] == endPos[1]:
            for road in self.road_intersections:
                if carPos[1] == road.getField("translation").getSFVec3f()[1]:
                    return abs(carPos[0] - endPos[0]) + abs(carPos[1] - endPos[1])
            return abs(carPos[0] - endPos[0]) + abs(carPos[1] - endPos[1]) + 30
        return abs(carPos[0] - endPos[0]) + abs(carPos[1] - endPos[1])
        
        
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
        self.reset()
        while self.step(self.timestep) != -1:
            # Get keyboard input
            keys = []
            key = self.keyboard.getKey()
            currTime = time.time() - self.start_time
            if not self.is_on_road() or currTime > 30:
                self.reset()
            if self.reachedEndPosition():
                self.reset()
            
            while key != -1:
                keys.append(key)
                self.manual_drive = True
                key = self.keyboard.getKey()
            
            if (self.manual_drive):
                self.speed = 0;
                self.steering_angle = 0;
                if Keyboard.UP in keys:
                    self.speed = self.max_speed
                elif Keyboard.DOWN in keys:
                    self.speed = -self.max_speed
                
                if Keyboard.LEFT in keys:
                    self.steering_angle = -self.max_steering_angle
                elif Keyboard.RIGHT in keys:
                    self.steering_angle = self.max_steering_angle
            else:
                self.speed, self.steering_angle = getInstructions(self.speed, self.steering_angle,
                    self.getPos(), self.camera.getImage(), self.end_pos, currTime)


            self.drive(self.speed, self.steering_angle)


# Create the robot controller object and run it
controller = drive_controller()
controller.run()
