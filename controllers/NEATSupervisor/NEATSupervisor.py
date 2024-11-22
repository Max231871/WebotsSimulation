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
        self.collect_roads()

        self.previous_position = self.getPos()
        self.time_stuck = 0.0
        self.movement_threshold = 0.01
        self.crash_delay = 1.0
        self.fitness = 0.0
        self.max_simulation_time = 3  # seconds

        self.initialize_devices()

    def collect_roads(self):
        """Collects all road segments dynamically from the world."""
        road_type = "Road"
        index = 0
        while True:
            road_def = f"{road_type}_{index}"
            road = self.getFromDef(road_def)
            if road:
                self.roads.append(road)
                index += 1
            else:
                break

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

        # Teleport the car back to the green start position
        start_position = self.green_start.getField("translation").getSFVec3f()
        self.teleport_object(self.robot_node, start_position)
        self.robot_node.resetPhysics()

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

    def calculate_fitness(self):
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
        fitness = 1.0 / (distance_to_goal + 1.0)  # Fitness increases as distance decreases
        fitness += self.left_front_wheel.getVelocity() / 30.0  # Normalize speed into fitness
        return fitness

    def set_controls(self, outputs):
        max_speed = 30.0
        max_steering_angle = 0.5

        speed = outputs[0] * max_speed
        steering_angle = outputs[1] * max_steering_angle

        self.left_front_wheel.setVelocity(speed)
        self.right_front_wheel.setVelocity(speed)

        self.left_steer.setPosition(steering_angle)
        self.right_steer.setPosition(steering_angle)

    def evaluate_genome(self, genome, config):
        """Evaluate a single genome's fitness."""
        self.net = neat.nn.FeedForwardNetwork.create(genome, config)
        self.start_time = self.getTime()
        self.fitness = 0.0

        while self.step(self.time_step) != -1:
            current_time = self.getTime()
            if current_time - self.start_time > self.max_simulation_time:
                break

            inputs = self.preprocess_camera_data()
            outputs = self.net.activate(inputs)
            self.set_controls(outputs)

            if self.detect_crash():
                break

            # Update fitness continuously
            self.fitness = self.calculate_fitness()

        print(f"Fitness: {self.fitness}")
        return self.fitness


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

    winner = p.run(eval_genomes, n=4)
    with open('best_genome.pkl', 'wb') as f:
        pickle.dump(winner, f)


if __name__ == '__main__':
    # Create a single global supervisor instance
    supervisor = NEATSupervisor()
    run_neat()
