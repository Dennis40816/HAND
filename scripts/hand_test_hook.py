import signal
import time
import sys
from hand_test_wifi import (
    set_test_environment,
)  # Import the function from the new script

# Global variable to track if script is interrupted
interrupted = False


def signal_handler(sig, frame):
    global interrupted
    print("Signal received, exiting...")
    interrupted = True
    sys.exit(0)


# Register signal handler
signal.signal(signal.SIGINT, signal_handler)


def print_colored(text, color_code):
    """
    Print text with the given color code.
    """
    print(f"\033[{color_code}m{text}\033[0m")


# Register the script to run before the build process
Import("env", "projenv")
print(" ")

# Start section with green color
print_colored("############################################################", "32")
print(" ")
print_colored("hand_test_hook running...", "32")
print(" ")
print(" Global ")
global_env = DefaultEnvironment()
for item in global_env.get("CPPDEFINES", []):
    print(item)

## debug
# print(global_env.Dump())
# time.sleep(5)

print(" ")
print(" Project ")
print(projenv)
for item in projenv.get("CPPDEFINES", []):
    print(item)
print(" ")

print(" ")
print(" ENV ")
for item in env.get("CPPDEFINES", []):
    print(item)
print(" ")

print_colored("Obtaining current build machine local IP address...", "32")

# Call the environment setup function
set_test_environment(projenv)

# End section with green color
print(" ")
print_colored("############################################################", "32")
print(" ")
