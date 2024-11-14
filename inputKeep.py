import os

# Static file path
FILE_PATH = "data_store.txt"


def inputKeep(variable_name, data):
    # Create the file if it doesn't exist
    if not os.path.exists(FILE_PATH):
        with open(FILE_PATH, 'w') as file:
            pass  # Create an empty file

    # Read the current file content
    with open(FILE_PATH, 'r') as file:
        lines = file.readlines()

    # Check if the variable already exists
    variable_found = False
    with open(FILE_PATH, 'w') as file:
        for line in lines:
            if line.startswith(f"{variable_name}:"):
                # Replace the existing value
                file.write(f"{variable_name}: {data}\n")
                variable_found = True
            else:
                # Keep other lines as is
                file.write(line)

        # If the variable was not found, append it
        if not variable_found:
            file.write(f"{variable_name}: {data}\n")
