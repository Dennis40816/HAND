# Import the PlatformIO environment modules
Import("env")
Import("projenv")

# Import necessary Python modules
import time
import os
from termcolor import colored

# List of supported platforms
SUPPORTED_PLATFORMS = ["espidf", "dummy"]
SUPPORTED_BOARD = ["hand"]

def get_relative_path():
    """
    Get the relative path of the VL53L1X library from the project root.
    """
    proj_root = projenv["PROJECT_DIR"]
    # Assuming the script is in the root of the VL53L1X library
    vl53l1x_path = os.path.join(proj_root, "lib", "devices", "VL53L1X")
    relative_path = os.path.relpath(vl53l1x_path, proj_root)
    return relative_path

def display_log_level(env):
    """
    Display the log level for VL53L1X from the CPPDEFINES.

    Args:
        env (Environment): The PlatformIO environment object.
    """
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, tuple) and item[0] == "LOG_LOCAL_LEVEL":
            print(f"VL53L1X log level: {item[1]}")
            time.sleep(0.1)
            break

def configure_src_filter(env, platform, board):
    """
    Configure the source filter and include paths based on the provided platform and board.

    Args:
        env (Environment): The PlatformIO environment object.
        platform (str): The target platform.
        board (str): The target board.
    """
    if platform and platform not in SUPPORTED_PLATFORMS:
        warning_message = f"Warning: Unsupported platform: {platform}. Falling back to 'platform/dummy'."
        print(colored(warning_message, "red"))
        time.sleep(1)
        platform = "dummy"

    if board and board not in SUPPORTED_BOARD:
        warning_message = f"Warning: Unsupported board: {board}. Won't use source code in bsp directory"
        print(colored(warning_message, "red"))
        time.sleep(1)
        board = None

    # Define source filters
    src_filter = ["+<*>", "-<platform/>", "-<bsp/>", "-<example/>"]

    if platform:
        src_filter.append(f"+<platform/{platform}/>")
    
    if board and platform:  # Only include bsp if both platform and board are specified
        src_filter.append(f"+<bsp/{board}/>")

    # Apply the source filter to the environment
    # env.Replace(SRC_FILTER=src_filter)

    # Get the project root and relative path
    proj_root = projenv["PROJECT_DIR"]
    relative_path = get_relative_path()

    # Define the include paths based on src_filter
    include_paths = []
    if platform:
        include_paths.append(os.path.join(relative_path, "platform", platform))
    if board and platform:
        include_paths.append(os.path.join(relative_path, "bsp", board))

    # Add include paths relative to the project root
    for path in include_paths:
        absolute_path = os.path.join(proj_root, path)
        env.Append(CPPPATH=[absolute_path])
    
    # print("Source filter:", src_filter)
    # print("Include paths:", include_paths)
    # print(env.Dump())

# Retrieve the LIB_PLATFORM and LIB_BSP_BOARD definitions
lib_platform = None
lib_bsp_board = None

for item in env.get("CPPDEFINES", []):
    if isinstance(item, tuple):
        if item[0] == "LIB_PLATFORM":
            lib_platform = item[1]
        elif item[0] == "LIB_BSP_BOARD":
            lib_bsp_board = item[1]

# Display log level information
display_log_level(env)

# Configure the source filter if the platform is defined
if lib_platform:
    configure_src_filter(env, lib_platform, lib_bsp_board)
else:
    print("Warning: LIB_PLATFORM is not defined in build flags.")