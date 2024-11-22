# create_genome.py
import neat
import pickle
import os

def create_genome(config_file, genome_id, genome_file_path):
    """
    Creates a genome based on the NEAT configuration and saves it to a file.

    Parameters:
    - config_file (str): Path to the NEAT configuration file.
    - genome_id (int): Unique identifier for the genome.
    - genome_file_path (str): Path where the genome file will be saved.
    """
    # Load configuration
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        config_file
    )

    # Initialize a genome
    genome = neat.DefaultGenome(genome_id)
    genome.configure_new(config)

    # Save the genome to a pickle file
    with open(genome_file_path, 'wb') as f:
        pickle.dump(genome, f)

    print(f"Genome {genome_id} created and saved to {genome_file_path}.")

if __name__ == "__main__":
    # Define paths
    local_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(local_dir, 'config-feedforward.txt')
    genome_dir = os.path.join(local_dir, 'genomes')

    # Ensure the genomes directory exists
    if not os.path.exists(genome_dir):
        os.makedirs(genome_dir)

    # Create a genome with ID 0
    genome_id = 0
    genome_file = os.path.join(genome_dir, f'genome_{genome_id}.pkl')
    create_genome(config_path, genome_id, genome_file)
