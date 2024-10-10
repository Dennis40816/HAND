import sys
import subprocess
import re

def get_current_ssid():
    """
    Retrieve the current connected WiFi SSID.
    """
    ssid = None
    if sys.platform.startswith("win"):
        try:
            # 使用 mbcs 编码以处理可能的非 ASCII 字符
            result = subprocess.check_output(
                ["netsh", "wlan", "show", "interfaces"], encoding="mbcs"
            )
            # 输出结果用于调试
            # print("Command output:", result)

            # 使用正则表达式严格匹配 SSID 行，并提取 SSID 值
            match = re.search(r"SSID\s*:\s*(.*)", result)
            if match:
                ssid = match.group(1).strip()
                return ssid
            else:
                print("SSID not found in command output.")
                return None
        except Exception as e:
            print(f"Error getting SSID on Windows: {e}")
            return None
    elif sys.platform.startswith("darwin"):
        try:
            result = subprocess.check_output(
                [
                    "/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport",
                    "-I",
                ]
            ).decode()
            for line in result.split("\n"):
                if " SSID" in line:
                    ssid = line.split(":")[1].strip()
                    break
        except Exception as e:
            print(f"Error getting SSID on macOS: {e}")
    elif sys.platform.startswith("linux"):
        try:
            ssid = subprocess.check_output(["iwgetid", "-r"]).decode().strip()
        except Exception as e:
            print(f"Error getting SSID on Linux: {e}")
    return ssid

if __name__ == '__main__':
    ssid_name = get_current_ssid()
    print(f"current ssid name is: {ssid_name}")