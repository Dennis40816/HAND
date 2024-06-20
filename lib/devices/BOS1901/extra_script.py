# Import the PlatformIO environment modules
Import("env", "projenv")

# Import necessary Python modules
import time
from termcolor import colored

# List of supported platforms
SUPPORTED_PLATFORMS = ["espidf", "stm32", "dummy"]

def display_log_level(env):
    """
    Display the log level for TCA6408A from the CPPDEFINES.
    
    Args:
        env (Environment): The PlatformIO environment object.
    """
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, tuple) and item[0] == "LOG_LOCAL_LEVEL":
            print(f"BOS1901 log level: {item[1]}")
            time.sleep(1)
            break

def check_duplicate_define(env, define):
    """
    Check for duplicate macro definitions in the CPPDEFINES.
    
    Args:
        env (Environment): The PlatformIO environment object.
        define (str): The macro to check for duplication.
        
    Returns:
        bool: True if a duplicate is found, otherwise False.
    """
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, tuple) and item[0] == define:
            return True
        elif item == define:
            return True
    return False

def configure_src_filter(env, proj_env, platform):
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
        time.sleep(5)
        platform = "dummy"

    platform_upper = platform.upper()
    cpp_define = f"-DLIB_USE_{platform_upper}_PLATFORM"

    # Check for duplicate definitions before adding, Important both global and projenv should append
    global_env = DefaultEnvironment()
    if check_duplicate_define(global_env, cpp_define):
        print(colored(f"Warning: Duplicate macro definition detected: {cpp_define}. Ignore!", "yellow"))
    else:
        global_env.Prepend(BUILD_FLAGS=[cpp_define])

    if check_duplicate_define(proj_env, cpp_define):
        print(colored(f"Warning: Duplicate macro definition detected in project environment: {cpp_define}. Ignore!", "yellow"))
    else:
        proj_env.Prepend(BUILD_FLAGS=[cpp_define])
        
    env.Prepend(BUILD_FLAGS=[cpp_define])

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
display_log_level(projenv)

# Configure the source filter if the platform is defined
if lib_platform:
    configure_src_filter(env, projenv, lib_platform)
else:
    print("Warning: LIB_PLATFORM is not defined in build flags.")
