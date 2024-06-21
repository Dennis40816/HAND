import os
import sys
import subprocess
from SCons.Script import Import

# Import the PlatformIO build environment
Import('env')

def print_colored(text, color_code):
    """
    Print text with the given color code.
    """
    print(f"\033[{color_code}m{text}\033[0m")

def install_requirements():
    # Use the PlatformIO provided PROJECT_DIR environment variable to get the project root directory
    project_dir = env.get("PROJECT_DIR")
    
    print_colored("Checking hand scripts package requirements...", 36)
    
    if project_dir is None:
        print_colored("PROJECT_DIR environment variable is not set. Please ensure this script is run within a PlatformIO environment.", 31)  # Red color for error
        return

    requirements_path = os.path.join(project_dir, 'requirements.txt')
    
    if not os.path.exists(requirements_path):
        print_colored("requirements.txt not found, skipping dependency installation.", 33)  # Yellow color for warning
        return
    
    # Command to run
    command = [sys.executable, '-m', 'pip', 'install', '-r', requirements_path]
    
    try:
        # Run the command and redirect output to the terminal
        result = subprocess.run(command, stdout=sys.stdout, stderr=sys.stderr, text=True)
        
        # Check if the command was successful
        if result.returncode != 0:
            raise subprocess.CalledProcessError(result.returncode, command)
            
    except subprocess.CalledProcessError as e:
        print_colored(f"An error occurred while installing requirements: {e}", 31)  # Red color for error

# Execute the installation
install_requirements()
