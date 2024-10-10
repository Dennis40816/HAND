# Import the PlatformIO environment modules
Import("env")

# Import necessary Python modules
import time
from termcolor import colored

# List of supported platforms
SUPPORTED_PLATFORMS = ["espidf", "stm32", "dummy"]

def display_log_level(env):
    """
    Display the log level for BOS1901 from the CPPDEFINES.
    
    Args:
        env (Environment): The PlatformIO environment object.
    """
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, tuple) and item[0] == "LOG_LOCAL_LEVEL":
            print(f"BOS1901 log level: {item[1]}")
            time.sleep(0.1)
            break

def configure_src_filter(env, platform):
    """
    Configure the source filter based on the provided platform.
    
    Args:
        env (Environment): The PlatformIO environment object.
        proj_env (Environment): The project-specific environment object.
        platform (str): The target platform.
    """
    if platform not in SUPPORTED_PLATFORMS:
        warning_message = f"Warning: Unsupported platform: {platform}. Falling back to 'platform/dummy'."
        print(colored(warning_message, "red"))
        time.sleep(3)
        platform = "dummy"
        
    # Define source filters
    exclude_filter = "-<platform/>"
    exclude_example = "-<example/>"
    include_filter = f"+<platform/{platform}/*>"

    src_filter = ["+<*>", exclude_filter, exclude_example, include_filter]

    # Apply the source filter to the environment
    env.Replace(SRC_FILTER=src_filter)

# Retrieve the LIB_PLATFORM definition
lib_platform = None
for item in env.get("CPPDEFINES", []):
    if isinstance(item, tuple) and item[0] == "LIB_PLATFORM":
        lib_platform = item[1]
        break

# Display log level information
display_log_level(env)

# Configure the source filter if the platform is defined
if lib_platform:
    configure_src_filter(env, lib_platform)
else:
    print("Warning: LIB_PLATFORM is not defined in build flags.")
