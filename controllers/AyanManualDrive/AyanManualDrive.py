# AyanManualDrive.py
from controller import Supervisor, Keyboard
from model import getInstructions
import time
import random
<<<<<<< HEAD
=======
import sys
import pickle
import neat
import numpy as np
import cv2
import os
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1

class AyanManualDrive(Supervisor):
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
        
        # Define road segments (adjust according to your world setup)
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
        
<<<<<<< HEAD
        self.roads = []
        for road in self.vertical_roads:
            self.roads.append(road)
        for road in self.horizontal_roads:
            self.roads.append(road)
        for road in self.road_intersections:
            self.roads.append(road)
=======
        self.roads = self.vertical_roads + self.horizontal_roads + self.road_intersections
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1

        # Camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.end_obj = self.getFromDef("redEnd")
        self.end_pos = self.end_obj.getPosition()
        self.distance_away = 0
<<<<<<< HEAD
        
=======
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1

        # Initialize NEAT network
        self.net = None
        genome_file = None
        config_file = None
        
        # Parse command-line arguments for genome and config files
        if '--genome' in sys.argv:
            genome_index = sys.argv.index('--genome') + 1
            if genome_index < len(sys.argv):
                genome_file = sys.argv[genome_index]
        if '--config' in sys.argv:
            config_index = sys.argv.index('--config') + 1
            if config_index < len(sys.argv):
                config_file = sys.argv[config_index]
        
        if genome_file and config_file:
            print(f"Received genome file: {genome_file}")
            print(f"Received config file: {config_file}")
            try:
                self.net = self.initialize_neat_network(config_file, genome_file)
                print(f"Neural network successfully initialized with genome: {genome_file} and config: {config_file}")
            except Exception as e:
                self.net = None
                print(f"Error initializing neural network: {e}")
                print("Neural network not initialized.")
        else:
            self.net = None
            print("Genome and/or config file not provided. Using default manual drive controls.")
        
    def initialize_neat_network(self, config_file, genome_file):
        """
        Initializes the NEAT neural network using the provided genome and configuration files.

        Parameters:
        - config_file (str): Path to the NEAT configuration file.
        - genome_file (str): Path to the genome pickle file.

        Returns:
        - net (neat.nn.FeedForwardNetwork): Initialized neural network.
        """
        # Load genome
        with open(genome_file, 'rb') as f:
            genome = pickle.load(f)

        # Load configuration
        config = neat.Config(
            neat.DefaultGenome,
            neat.DefaultReproduction,
            neat.DefaultSpeciesSet,
            neat.DefaultStagnation,
            config_file
        )

        # Create neural network
        net = neat.nn.FeedForwardNetwork.create(genome, config)
        return net

    def is_on_road(self):
        car_position = self.getPos()  # [x, y, z]
        for road in self.roads:
            center = road.getPosition()
            translation = road.getField("translation").getSFVec3f()
            # Define road boundaries based on your world setup
            # Example for horizontal roads:
            if road in self.horizontal_roads:
                road_length = 15  # Adjust as per your world
                road_width = 7
                if (center[0] - road_length / 2 <= car_position[0] <= center[0] + road_length / 2 and
                    center[1] - road_width / 2 <= car_position[1] <= center[1] + road_width / 2):
                    return True
            # Example for vertical roads:
            elif road in self.vertical_roads:
                road_length = 15
                road_width = 7
                if (center[0] - road_width / 2 <= car_position[0] <= center[0] + road_width / 2 and
                    center[1] - road_length / 2 <= car_position[1] <= center[1] + road_length / 2):
                    return True
            # Example for intersections:
            elif road in self.road_intersections:
                road_length = 19
                road_width = 19
                if (center[0] - road_length / 2 <= car_position[0] <= center[0] + road_length / 2 and
                    center[1] - road_width / 2 <= car_position[1] <= center[1] + road_width / 2):
                    return True
        return False

<<<<<<< HEAD
    
=======
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1
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
<<<<<<< HEAD
            
=======

>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1
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
<<<<<<< HEAD
        
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
        
=======
            
    def shortestPath(self, carPos, endPos):
        return np.linalg.norm(np.array(carPos[:2]) - np.array(endPos[:2]))
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1
        
    def detect_crash(self, keys):
        """Detect if the car is crashed or stuck based on movement threshold and duration."""
        # Check if any arrow key is pressed
        if any(key in keys for key in [Keyboard.UP, Keyboard.DOWN, Keyboard.LEFT, Keyboard.RIGHT]):
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
    
    def preprocess_image(self, image_data):
        """
        Converts image data from Webots camera to a normalized grayscale flattened list.
        """
        # Convert image data to a numpy array
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, 4))
        # Convert BGRA to Grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
        # Resize image
        image = cv2.resize(image, (10, 10))  # Adjust size as needed for input dimensionality
        # Flatten and normalize
        image = image.flatten() / 255.0
        return image.tolist()
    
    def calculate_fitness(self):
        """
        Calculates fitness based on distance traveled towards the end position.
        """
        current_pos = self.getPos()
        distance = self.shortestPath(current_pos, self.end_pos)
        fitness = 1.0 / (distance + 1.0)  # Inverse relation to distance
        return fitness
    
    def run(self):
        self.reset()
        while self.step(self.timestep) != -1:
            # Get keyboard input
            keys = []
            key = self.keyboard.getKey()
            currTime = time.time() - self.start_time
            if not self.is_on_road() or currTime > 30:
<<<<<<< HEAD
                self.reset()
            if self.reachedEndPosition():
=======
                fitness = self.calculate_fitness()
                print(f"Fitness: {fitness}")
                self.reset()
            if self.reachedEndPosition():
                fitness = self.calculate_fitness()
                print(f"Fitness: {fitness}")
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1
                self.reset()
            
            while key != -1:
                keys.append(key)
                self.manual_drive = True
                key = self.keyboard.getKey()
            
<<<<<<< HEAD
<<<<<<< HEAD
            # Initialize speed and steering
            speed = 0.0
            steering = 0.0
            
            
            file = open("humanControls.txt", "a")
            keys_pressed = set()
            # Handle keyboard input
            if Keyboard.UP in keys:
                speed = self.max_speed
                # Key is pressed
               

                if Keyboard.UP not in keys_pressed:
                    print(f"Key pressed: {Keyboard.UP}")
                    keys_pressed.add(Keyboard.UP)
          
                # Key is released
                for pressed_key in list(keys_pressed):
                        # print(f"Key released: {pressed_key}")
                        file.write("UP " + str(speed))
                        keys_pressed.remove(pressed_key)       
                    
            elif Keyboard.DOWN in keys:
                speed = -self.max_speed
                file.write("DOWN " + str(speed))
               
            if Keyboard.LEFT in keys:
                steering = -self.max_steering_angle
                file.write("LEFT " + str(steering))
               
            elif Keyboard.RIGHT in keys:
                steering = self.max_steering_angle
                file.write("RIGHT " + str(steering))

            file.close()
            
            # Drive the car
            self.drive(speed, steering)
            # Detect crash
            self.detect_crash(keys)
            
            print("Is on road: " + str(self.is_on_road()))
            # Print crash status
            if self.if_crashed:
                print("if_crashed is True")
=======
            if (self.manual_drive):
                self.speed = 0;
                self.steering_angle = 0;
=======
            if self.manual_drive:
                self.speed = 0.0
                self.steering_angle = 0.0
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1
                if Keyboard.UP in keys:
                    self.speed = self.max_speed
                elif Keyboard.DOWN in keys:
                    self.speed = -self.max_speed
                
                if Keyboard.LEFT in keys:
                    self.steering_angle = -self.max_steering_angle
                elif Keyboard.RIGHT in keys:
                    self.steering_angle = self.max_steering_angle
            else:
<<<<<<< HEAD
                self.speed, self.steering_angle = getInstructions(self.speed, self.steering_angle,
                    self.getPos(), self.camera.getImage(), self.end_pos)

=======
                # Preprocess image
                preprocessed_image = self.preprocess_image(self.camera.getImage())
                
                # Get instructions from the neural network via model.getInstructions
                self.speed, self.steering_angle = getInstructions(
                    self.speed,
                    self.steering_angle,
                    self.getPos(),
                    preprocessed_image,
                    self.end_pos,
                    currTime,
                    self.net  # Pass the neural network as a parameter
                )
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1

            # Apply drive commands
            self.drive(self.speed, self.steering_angle)

<<<<<<< HEAD
>>>>>>> 89e5f798357f826104c4787bb3b0a954975142d7
=======
            # Detect crashes
            self.detect_crash(keys)

            # Optional: Handle crashes
            if self.if_crashed:
                fitness = self.calculate_fitness()
                print(f"Fitness: {fitness}")
                self.reset()
>>>>>>> eab5459ba35f583bbbd595edba3a5b0d4fb997b1

# Create the robot controller object and run it
if __name__ == "__main__":
    controller = AyanManualDrive()
    controller.run()
