# run_neat.py
import neat
import os
import pickle
import subprocess
import sys
import multiprocessing

# Paths
local_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(local_dir, 'config-feedforward.txt')
controller_script = os.path.join(local_dir, 'controller', 'drive_controller.py')
world_file = os.path.join(local_dir, 'worlds', 'your_world.wbt')
genome_dir = os.path.join(local_dir, 'genomes')

# Ensure genomes directory exists
if not os.path.exists(genome_dir):
    os.makedirs(genome_dir)

def run_simulation(genome_file, config_file):
    """
    Runs a Webots simulation with the given genome.
    """
    # Use absolute paths to avoid path-related issues
    genome_file = os.path.abspath(genome_file)
    config_file = os.path.abspath(config_file)
    world_file_abs = os.path.abspath(world_file)
    controller_script_abs = os.path.abspath(controller_script)

    # Launch Webots with the genome and config as arguments
    webots_command = [
        'webots',
        '--mode', 'batch',
        '--no-rendering',  # Run in headless mode for efficiency
        world_file_abs,
        '--controller', controller_script_abs,
        '--',  # Separator between Webots arguments and controller arguments
        '--genome', genome_file,
        '--config', config_file
    ]

    try:
        # Start Webots process
        process = subprocess.Popen(webots_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate(timeout=60)  # Set a timeout to prevent hangs

        # Parse fitness from controller's output
        fitness = parse_fitness(stdout, stderr)

    except subprocess.TimeoutExpired:
        process.kill()
        stdout, stderr = process.communicate()
        fitness = 0.0  # Assign a low fitness due to timeout

    return fitness

def parse_fitness(stdout, stderr):
    """
    Parses fitness value from controller's output.
    Assumes that the controller prints 'Fitness: <value>' upon completion.
    """
    fitness = 0.0
    try:
        for line in stdout.decode().split('\n'):
            if 'Fitness:' in line:
                fitness = float(line.strip().split(':')[1])
                break
    except Exception as e:
        print(f"Error parsing fitness: {e}")
        fitness = 0.0  # Default fitness if parsing fails
    return fitness

def evaluate_genome(genome, config):
    """
    Evaluates a single genome by running a simulation and assigning fitness.
    """
    genome_id = genome.key
    genome_file = os.path.join(genome_dir, f'genome_{genome_id}.pkl')
    with open(genome_file, 'wb') as f:
        pickle.dump(genome, f)

    # Run simulation
    fitness = run_simulation(genome_file, config_path)

    # Assign fitness
    genome.fitness = fitness

def run_simulation_wrapper(genome, config):
    """
    Wrapper function to match ParallelEvaluator's expected signature.
    """
    genome_id = genome.key
    genome_file = os.path.join(genome_dir, f'genome_{genome_id}.pkl')
    with open(genome_file, 'wb') as f:
        pickle.dump(genome, f)

    # Run simulation and get fitness
    fitness = run_simulation(genome_file, config_path)

    return fitness

def run_neat():
    """
    Sets up and runs the NEAT algorithm.
    """
    # Load configuration
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        config_path
    )

    # Create the population
    population = neat.Population(config)

    # Add reporters to show progress in the terminal
    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)
    # population.add_reporter(neat.Checkpointer(10))  # Optional: Checkpoint every 10 generations

    # Use ParallelEvaluator for parallel simulation
    pe = neat.ParallelEvaluator(multiprocessing.cpu_count(), run_simulation_wrapper)

    # Run NEAT with parallel evaluation
    winner = population.run(pe.evaluate, n=50)  # Adjust generations as needed

    # Save the best genome
    with open('best_genome.pkl', 'wb') as f:
        pickle.dump(winner, f)

    print('\nBest genome:\n{!s}'.format(winner))

if __name__ == '__main__':
    run_neat()
