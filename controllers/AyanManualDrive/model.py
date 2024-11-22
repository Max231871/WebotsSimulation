# model.py
import neat
import numpy as np

def getInstructions(speed, steering, position, image, endPos, time_step, net):
    """
    Determines speed and steering commands using the neural network.

    Parameters:
    - speed (float): Current speed of the BMX5.
    - steering (float): Current steering angle.
    - position (list): [x, y, z] position coordinates.
    - image (list): Preprocessed image data.
    - endPos (list): [x, y, z] target position coordinates.
    - time_step (float): Elapsed time in seconds.
    - net (neat.nn.FeedForwardNetwork): NEAT neural network.

    Returns:
    - speed_cmd (float): Speed command.
    - steering_cmd (float): Steering command.
    """
    if net is None:
        print("Neural network not initialized.")
        return 0.0, 0.0  # Default commands

    # Normalize inputs based on expected ranges
    normalized_speed = speed / 30.0  # Assuming max speed is 30
    normalized_steering = steering / 0.5  # Assuming max steering angle is 0.5 radians
    normalized_position = [position[0] / 100.0, position[1] / 100.0, position[2] / 100.0]  # Adjust based on environment
    normalized_endPos = [endPos[0] / 100.0, endPos[1] / 100.0, endPos[2] / 100.0]
    normalized_time = time_step / 1000.0  # Normalize time

    # Combine all inputs
    inputs = [
        normalized_speed,
        normalized_steering,
        normalized_position[0],
        normalized_position[1],
        normalized_position[2],
        normalized_endPos[0],
        normalized_endPos[1],
        normalized_endPos[2],
        normalized_time
    ] + image  # Assuming image is already preprocessed and normalized

    # Ensure input length matches the network's expectation
    if len(inputs) != net.num_inputs:
        print(f"Input size mismatch: expected {net.num_inputs}, got {len(inputs)}")
        return 0.0, 0.0  # Default commands

    # Activate the neural network
    outputs = net.activate(inputs)

    # Map outputs based on indices
    speed_cmd = outputs[0]      # First output neuron for speed
    steering_cmd = outputs[1]   # Second output neuron for steering

    return speed_cmd, steering_cmd
