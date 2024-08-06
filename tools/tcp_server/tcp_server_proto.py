import socket
import selectors
import hand_data_pb2  # Generated Protocol Buffers file
import binascii
import struct
import shutil

HOST = "0.0.0.0"
PORT = 8055

# Create a selector object
sel = selectors.DefaultSelector()

def print_full_line(char='#'):
    """Print a full line of the given character to fill the terminal width."""
    columns = shutil.get_terminal_size().columns
    print(char * columns)

# Data type mapping dictionary
data_type_map = {
    hand_data_pb2.UINT8: "UINT8",
    hand_data_pb2.UINT16: "UINT16",
    hand_data_pb2.INT32: "INT32",
    hand_data_pb2.INT64: "INT64",
    hand_data_pb2.FLOAT: "FLOAT",
    hand_data_pb2.DOUBLE: "DOUBLE"
}

# Source mapping dictionary
source_map = {
    hand_data_pb2.ESP32_S3_MINI_MAIN: "ESP32_S3_MINI_MAIN",
    hand_data_pb2.BQ27427_BATTERY: "BQ27427_BATTERY",
    hand_data_pb2.KX132_1211_SENSOR1: "KX132_1211_SENSOR1",
    hand_data_pb2.KX132_1211_SENSOR2: "KX132_1211_SENSOR2",
    hand_data_pb2.KX132_1211_SENSOR3: "KX132_1211_SENSOR3",
    hand_data_pb2.KX132_1211_SENSOR4: "KX132_1211_SENSOR4",
    hand_data_pb2.VL53L1X_SENSOR1: "VL53L1X_SENSOR1",
    hand_data_pb2.VL53L1X_SENSOR2: "VL53L1X_SENSOR2",
    hand_data_pb2.CH101_SENSOR1: "CH101_SENSOR1",
    hand_data_pb2.CH101_SENSOR2: "CH101_SENSOR2",
    hand_data_pb2.CH101_SENSOR3: "CH101_SENSOR3",
    hand_data_pb2.CH101_SENSOR4: "CH101_SENSOR4",
    hand_data_pb2.BOS1901_ACTUATOR1: "BOS1901_ACTUATOR1",
    hand_data_pb2.BOS1901_ACTUATOR2: "BOS1901_ACTUATOR2",
    hand_data_pb2.BOS1901_ACTUATOR3: "BOS1901_ACTUATOR3",
    hand_data_pb2.BOS1901_ACTUATOR4: "BOS1901_ACTUATOR4",
    hand_data_pb2.BMI323_IMU: "BMI323_IMU",
    hand_data_pb2.TCA6408A_CH101: "TCA6408A_CH101",
    hand_data_pb2.TCA6408A_OTHER: "TCA6408A_OTHER"
}

def accept(sock, mask):
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False)
    sel.register(conn, selectors.EVENT_READ, read)

def read(conn, mask):
    print(' ' * 2)
    print_full_line()
    data = conn.recv(4096)  # Read data from the connection
    if data:
        try:
            print(f'Msg len: {len(data)}')
            # print(binascii.hexlify(data))
            
            # Deserialize handmsg
            msg = hand_data_pb2.HandMsg()
            msg.ParseFromString(data)
            
            # # Print the deserialized message
            # print(f"Received message: {msg}")
            
            # Parse data messages
            if msg.msg_type == hand_data_pb2.DATA and msg.HasField("data_wrapper"):
                for data_msg in msg.data_wrapper.data_msgs:
                    print(' ')
                    data_type_str = data_type_map.get(data_msg.data_type, "Unknown")
                    source_str = source_map.get(data_msg.source, "Unknown")
                    print(f"Source: {source_str}")
                    print(f"Data Type: {data_type_str}")
                    print(f"Data Count: {data_msg.data_count}")
                    print(f"Timestamps: {data_msg.timestamps}")
                    
                    # Decode data based on data_type
                    data_format = ""
                    if data_msg.data_type == hand_data_pb2.UINT8:
                        data_format = f"{data_msg.data_count}B"
                    elif data_msg.data_type == hand_data_pb2.UINT16:
                        data_format = f"{data_msg.data_count}H"
                    elif data_msg.data_type == hand_data_pb2.INT32:
                        data_format = f"{data_msg.data_count}i"
                    elif data_msg.data_type == hand_data_pb2.INT64:
                        data_format = f"{data_msg.data_count}q"
                    elif data_msg.data_type == hand_data_pb2.FLOAT:
                        data_format = f"{data_msg.data_count}f"
                    elif data_msg.data_type == hand_data_pb2.DOUBLE:
                        data_format = f"{data_msg.data_count}d"
                    
                    if data_format:
                        decoded_data = struct.unpack(data_format, data_msg.data)
                        print(f"Decoded Data: {decoded_data}")
                    else:
                        print("Unknown data type")
                    
        except Exception as e:
            print(f"Failed to parse message: {e}")
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
