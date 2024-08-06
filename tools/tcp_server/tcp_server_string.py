import socket
import selectors

HOST = "0.0.0.0"
PORT = 8055

# 创建一个选择器对象
sel = selectors.DefaultSelector()


def accept(sock, mask):
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False)
    sel.register(conn, selectors.EVENT_READ, read)


def read(conn, mask):
    data = conn.recv(1024)  # 从连接中读取数据
    if data:
        print(f"Received: {data.decode()}")
    else:
        print("Closing connection")
        sel.unregister(conn)
        conn.close()


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((HOST, PORT))
    sock.listen()
    print(f"Server started on {HOST}:{PORT}")
    sock.setblocking(False)

    sel.register(sock, selectors.EVENT_READ, accept)

    while True:
        events = sel.select()
        for key, mask in events:
            callback = key.data
            callback(key.fileobj, mask)


if __name__ == "__main__":
    main()
