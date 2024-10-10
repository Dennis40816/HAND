import socket
import getpass
import time
import wifi_ssid  # Assuming wifi_ssid is a custom module to get SSID


def print_colored(text, color_code):
    """
    Print text with the given color code.
    """
    print(f"\033[{color_code}m{text}\033[0m")


def get_own_ip():
    """
    Retrieve the current device's IP address.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception as e:
        ip = "127.0.0.1"  # Use loopback address if error occurs
    finally:
        s.close()
    return ip


def get_user_input():
    """
    Prompt user for WiFi password.
    """
    ssid = wifi_ssid.get_current_ssid()
    if not ssid:
        ssid = input("Enter WiFi SSID: ")
    else:
        print_colored(f"Detected WiFi SSID: {ssid}", "36")
        
    time.sleep(1)
    password = getpass.getpass("Enter WiFi Password: ")
    return ssid, password


def add_cpp_define(env, key, value):
    """
    Add a define to CPPDEFINES if it doesn't already exist.
    """
    existing_defines = [
        define[0] if isinstance(define, tuple) else define
        for define in env.get("CPPDEFINES", [])
    ]
    if key not in existing_defines:
        env.Append(CPPDEFINES=[(key, env.StringifyMacro(value))])
        print_colored(f"Added {key}={value} to CPPDEFINES", "36")
    else:
        print_colored(f"{key} is already defined in CPPDEFINES", "36")


def check_and_define(env, macro_name, default_value_function, description):
    """
    Check if a macro is defined in CPPDEFINES. If not, define it using a default value function.
    Print instructions for user to define it in build_flags if needed.
    """
    existing_defines = [
        define[0] if isinstance(define, tuple) else define
        for define in env.get("CPPDEFINES", [])
    ]
    if macro_name not in existing_defines:
        default_value = default_value_function()
        add_cpp_define(env, macro_name, default_value)
        print_colored(
            f'Hint: You can define {macro_name} in build_flags as -D{macro_name}="{default_value}"',
            "33",
        )
        print_colored(f"{description}", "36")
    else:
        print_colored(f"{macro_name} is already defined, skipping.", "36")


def set_test_environment(env):
    """
    Set up test environment by checking and defining necessary macros.
    """
    # Check and define TEST_HAND_TCP_ECHO_SERVER_IP
    check_and_define(
        env,
        "TEST_HAND_TCP_ECHO_SERVER_IP",
        get_own_ip,
        "This sets the local machine's IP address for the TCP Echo Server.",
    )

    # Check if SSID and password are defined
    ssid_defined = any(
        define[0] == "TEST_HAND_WIFI_SSID" for define in env.get("CPPDEFINES", [])
    )
    password_defined = any(
        define[0] == "TEST_HAND_WIFI_PASSWORD" for define in env.get("CPPDEFINES", [])
    )

    if not ssid_defined and not password_defined:
        # If neither SSID nor password is defined, prompt for both
        ssid, password = get_user_input()
        add_cpp_define(env, "TEST_HAND_WIFI_SSID", ssid)
        add_cpp_define(env, "TEST_HAND_WIFI_PASSWORD", password)
        print_colored(
            "Hint: You can define both TEST_HAND_WIFI_SSID and TEST_HAND_WIFI_PASSWORD in build_flags to avoid manual input.",
            "33",
        )
    elif ssid_defined and not password_defined:
        # If SSID is defined but not password, prompt for password
        time.sleep(1)
        password = getpass.getpass("Enter WiFi Password: ")
        add_cpp_define(env, "TEST_HAND_WIFI_PASSWORD", password)
        print_colored(
            "Hint: You can define TEST_HAND_WIFI_PASSWORD in build_flags to avoid manual input.",
            "33",
        )
    elif not ssid_defined and password_defined:
        # If password is defined but not SSID, automatically detect SSID
        ssid = wifi_ssid.get_current_ssid()
        if not ssid:
            ssid = input("Enter WiFi SSID: ")
        add_cpp_define(env, "TEST_HAND_WIFI_SSID", ssid)
        print_colored(
            "Hint: You can define TEST_HAND_WIFI_SSID in build_flags to avoid manual input.",
            "33",
        )
    else:
        # Both SSID and password are defined
        print_colored(
            "Both TEST_HAND_WIFI_SSID and TEST_HAND_WIFI_PASSWORD are already defined, skipping.",
            "36",
        )
