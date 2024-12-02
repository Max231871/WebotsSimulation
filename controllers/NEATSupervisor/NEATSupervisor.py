import neat
import os
import pickle
from controller import Supervisor
import numpy as np
import cv2
import random


class NEATSupervisor(Supervisor):
    def __init__(self):
        super().__init__()
        self.time_step = int(self.getBasicTimeStep())

        # Initialize the robot node first
        self.robot_node = self.getFromDef("car")
        if self.robot_node is None:
            raise ValueError("Robot node with DEF name 'car' not found.")

        self.end_obj = self.getFromDef("redEnd")
        if self.end_obj is None:
            raise ValueError("End node with DEF name 'redEnd' not found.")
        self.green_start = self.getFromDef("greenStart")
        if self.green_start is None:
            raise ValueError("Start node with DEF name 'greenStart' not found.")

        self.roads = []
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
        self.roads.extend(self.vertical_roads)
        self.roads.extend(self.horizontal_roads)
        self.roads.extend(self.road_intersections)
        
        self.previous_position = self.getPos()
        self.time_stuck = 0.0
        self.movement_threshold = 0.01
        self.crash_delay = 1.0
        self.fitness = 0.0
        self.max_simulation_time = 15  # seconds
        self.shortest_path = self.shortestPath(self.previous_position, self.end_obj.getField("translation").getSFVec3f())
        self.speed = 0.0
        self.steering_angle = 0.0
        self.distance_travelled = 0.0
        self.initialize_devices()

    def initialize_devices(self):
        self.left_steer = self.getDevice("left_steer")
        self.right_steer = self.getDevice("right_steer")
        self.left_front_wheel = self.getDevice("left_front_wheel")
        self.right_front_wheel = self.getDevice("right_front_wheel")

        self.camera = self.getDevice("camera")
        if not self.camera:
            raise ValueError("Camera device not found.")
        self.camera.enable(self.time_step)

        if not all([self.left_steer, self.right_steer, self.left_front_wheel, self.right_front_wheel]):
            raise ValueError("One or more motor devices not found.")

        self.left_front_wheel.setPosition(float('inf'))
        self.right_front_wheel.setPosition(float('inf'))
        self.left_front_wheel.setVelocity(0)
        self.right_front_wheel.setVelocity(0)

    def preprocess_camera_data(self):
        """Preprocess the camera image and return a flattened grayscale array."""
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = np.frombuffer(self.camera.getImage(), dtype=np.uint8).reshape((height, width, 4))
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)  # Convert BGRA to grayscale
        resized_image = cv2.resize(gray_image, (10, 10))  # Resize to reduce dimensionality
        flattened_image = resized_image.flatten() / 255.0  # Normalize pixel values
        return flattened_image

    def preprocess_inputs(self):
        input_list = self.preprocess_camera_data().tolist()
        # input_list.extend(self.getPos())
        # input_list.extend(self.end_obj.getPosition())
        input_list.append(self.speed / 30.0)
        input_list.append(self.steering_angle / 0.5)
        return np.array(input_list)
        
    def get_random_road_position(self):
        """Selects a random road segment and returns its translation position."""
        if not self.roads:
            return [0, 0, 0]  # Default position to avoid crashing
        selected_road = random.choice(self.roads)
        road_position = selected_road.getField("translation").getSFVec3f()
        return road_position

    def teleport_object(self, obj, position):
        """Teleports a specific object to the given position."""
        if obj:
            obj.getField("translation").setSFVec3f(position)

    def reset_simulation_state(self):
        """Manually reset the simulation state."""
        print("Resetting simulation state...")
        # Reset velocities
        self.left_front_wheel.setVelocity(0)
        self.right_front_wheel.setVelocity(0)

        # Reset internal states
        self.previous_position = self.getPos()
        self.time_stuck = 0.0
        self.fitness = 0.0
        self.speed = 0.0
        self.steering_angle = 0.0
        self.distance_travelled = 0.0
        # Teleport the car back to the green start position
        start_position = self.green_start.getField("translation").getSFVec3f()
        self.teleport_object(self.robot_node, start_position)
        self.robot_node.resetPhysics()
        self.shortest_path = self.shortestPath(start_position, self.end_obj.getField("translation").getSFVec3f())

        for _ in range(10):  # Step multiple times to apply changes
            self.step(self.time_step)

    def getPos(self):
        """Get the current position of the car."""
        return self.robot_node.getPosition()

    def detect_crash(self):
        current_position = self.getPos()
        if current_position is None:
            print("Position unavailable during crash detection.")
            return True

        movement = [abs(current_position[i] - self.previous_position[i]) for i in range(3)]
        self.previous_position = current_position

        if all(m < self.movement_threshold for m in movement):
            self.time_stuck += self.time_step / 1000.0
            if self.time_stuck > self.crash_delay:
                print("Car is stuck!")
                return True
        else:
            self.time_stuck = 0.0

        return False

    def calculate_distance(self):
        """
        Calculate fitness based on:
        - Speed
        - Distance from the goal
        """
        current_pos = self.getPos()
        distance_to_goal = np.linalg.norm(
            np.array(self.end_obj.getField("translation").getSFVec3f()[:2]) -
            np.array(current_pos[:2])
        )
        return 1.0 / (distance_to_goal + 0.01)  # Fitness increases as distance decreases

    def set_controls(self, outputs):
        max_speed = 30.0
        max_steering_angle = 0.5

        self.speed = outputs[0] * max_speed
        self.steering_angle = outputs[1] * max_steering_angle

        self.left_front_wheel.setVelocity(self.speed)
        self.right_front_wheel.setVelocity(self.speed)

        self.left_steer.setPosition(self.steering_angle)
        self.right_steer.setPosition(self.steering_angle)
    
    def reached_end(self):
        carPos = self.getPos()
        endPos = self.end_obj.getField("translation").getSFVec3f()
        if (endPos[0] - 2 <= carPos[0] <= endPos[0] + 2 and
           endPos[1] - 2 <= carPos[1] <= endPos[1] + 2):
           return True
        return False

    def evaluate_genome(self, genome, config):
        """Evaluate a single genome's fitness."""
        self.net = neat.nn.FeedForwardNetwork.create(genome, config)
        self.start_time = self.getTime()
        self.fitness = 0.0
        reached = False
        timeCounter = 0.0
        onRoadCounter = 0.0
        self.max_simulation_time = self.shortest_path / 4.0
        while self.step(self.time_step) != -1:
            current_time = self.getTime()
            if self.is_on_road():
                onRoadCounter += 1
            timeCounter += 1
            if current_time - self.start_time > self.max_simulation_time:
                break
            if self.reached_end():
                reached = True
                break
            inputs = self.preprocess_inputs()
            outputs = self.net.activate(inputs)
            self.set_controls(outputs)

            # Update fitness continuously
        if reached:
            self.fitness = 50 + 40 * ((self.getTime() - self.start_time) / (self.shortest_path / 30.0)) - 20 * ((timeCounter - onRoadCounter) / timeCounter)
        else:
            self.fitness = -3 * ((timeCounter - onRoadCounter) / timeCounter) + 30 * self.calculate_distance()
        print(f"Fitness: {self.fitness}")
        return self.fitness
        
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
        
    def is_on_road(self):
        car_position = self.getPos()  # [x, y, z]
        for road in self.roads:
            center = road.getField("translation").getSFVec3f()
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

def eval_genomes(genomes, config):
    print("Starting a new generation...")
    # Teleport redEnd and greenStart at the start of each generation
    print("Teleporting objects for new generation...")
    new_red_end_position = supervisor.get_random_road_position()
    new_green_start_position = supervisor.get_random_road_position()
    while new_red_end_position == new_green_start_position:
        new_red_end_position = supervisor.get_random_road_position()
        new_green_start_position = supervisor.get_random_road_position()

    supervisor.teleport_object(supervisor.end_obj, new_red_end_position)
    supervisor.teleport_object(supervisor.green_start, new_green_start_position)

    # Evaluate each genome
    for genome_id, genome in genomes:
        try:
            print(f"Evaluating genome {genome_id}...")
            supervisor.reset_simulation_state()  # Reset simulation for each genome
            fitness = supervisor.evaluate_genome(genome, config)
            genome.fitness = fitness
            print(f"Genome {genome_id} fitness: {fitness}")
        except Exception as e:
            print(f"Error evaluating genome {genome_id}: {e}")
            genome.fitness = float('-inf')


def run_neat():
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, 'config-feedforward.txt')
    if not os.path.exists(config_path):
        print(f"Configuration file not found at {config_path}")
        return

    config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                         neat.DefaultSpeciesSet, neat.DefaultStagnation,
                         config_path)

    p = neat.Population(config)
    p.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    p.add_reporter(stats)

    winner = p.run(eval_genomes, n=500)
    with open('best_genome.pkl', 'wb') as f:
        pickle.dump(winner, f)


if __name__ == '__main__':
    # Create a single global supervisor instance
    supervisor = NEATSupervisor()
    run_neat()