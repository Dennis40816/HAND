import socket
import signal
import sys
import select

# 設置伺服器運行標誌
running = True

# 信號處理函數，用於關閉伺服器
def signal_handler(sig, frame):
    global running
    print("\nStopping TCP echo server...")
    running = False

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

# 註冊信號處理器
signal.signal(signal.SIGTERM, signal_handler)
signal.signal(signal.SIGINT, signal_handler)

# 設置伺服器端口
PORT = 6020

# 創建一個 TCP/IP socket
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', PORT))
    server_socket.listen()
    server_socket.setblocking(False)

    print(f"TCP echo server running on port {PORT}...")
    MY_IP = get_own_ip()
    print(f"Current IP is: {MY_IP}")

    while running:
        try:
            ready_to_read, _, _ = select.select([server_socket], [], [], 1)
            if ready_to_read:
                client_socket, client_address = server_socket.accept()
                client_socket.setblocking(False)
                print(f"Connection from {client_address}")

                with client_socket:
                    while True:
                        ready_to_read, _, _ = select.select([client_socket], [], [], 1)
                        if ready_to_read:
                            data = client_socket.recv(1024)
                            if not data:
                                break
                            client_socket.sendall(data)
        except OSError as e:
            print(f"OSError: {e}")
            break
        except Exception as e:
            print(f"Unexpected error: {e}")
            break

print("TCP echo server stopped.")
