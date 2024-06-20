# see https://docs.platformio.org/en/latest/scripting/construction_environments.html
# this is library’s isolated environment
# Import("env")

global_env = DefaultEnvironment()

# Import necessary modules
import inspect
import os
import time
from termcolor import colored

# Get the current library path
current_script_path = inspect.getfile(lambda: None)
current_library_path = os.path.dirname(current_script_path)


def custom_info(env):
    for item in env.get("CPPDEFINES", []):
        if isinstance(item, tuple) and item[0] == "LOG_LOCAL_LEVEL":
            print(f"BOS1901 log level: {item[1]}")
            time.sleep(1)
            break


SUPPORTED_PLATFORMS = ["espidf", "stm32", "dummy"]


def set_src_filter(env, global_env, platform, library_path):
    if platform not in SUPPORTED_PLATFORMS:
        warning_message = f"Warning: Unsupported platform: {platform}. Falling back to 'platform/dummy'."
        colored_warning_message = colored(warning_message, "red")
        print(colored_warning_message)
        time.sleep(5)
        platform = "dummy"

    # Convert platform to upper case for CPPDEFINE
    platform_upper = platform.upper()

    cpp_define = f"LIB_USE_{platform_upper}_PLATFORM"

    # Add cpp_define into global_env CPPDEFINES
    global_env.Append(CPPDEFINES=[cpp_define])

    # Adjust paths based on the library path
    exclude_filter = f"-<{library_path}/platform/>"
    exclude_example = f"-<{library_path}/example/>"
    include_filter = f"+<{library_path}/platform/{platform}/*>"

    # Combine to source filter
    src_filter = [
        f"+<{library_path}/*>",
        exclude_filter,
        exclude_example,
        include_filter,
    ]

    env.Replace(SRC_FILTER=src_filter)


# Get LIB_PLATFORM
lib_platform = None
for item in global_env.get("CPPDEFINES", []):
    if isinstance(item, tuple) and item[0] == "LIB_PLATFORM":
        lib_platform = item[1]
        break

custom_info(global_env)

if lib_platform:
    set_src_filter(global_env, global_env, lib_platform, current_library_path)
else:
    print("Warning: LIB_PLATFORM is not defined in build flags.")
