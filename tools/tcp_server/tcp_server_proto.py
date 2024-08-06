import socket
import selectors
import struct
import hand_data_pb2  # Generated Protocol Buffers file
import queue
import threading
import os
import re
import json
from google.protobuf.json_format import MessageToJson

HOST = "0.0.0.0"
PORT = 8055

# Create a selector object
sel = selectors.DefaultSelector()

# Data type mapping dictionary
data_type_map = {
    hand_data_pb2.UINT8: "UINT8",
    hand_data_pb2.UINT16: "UINT16",
    hand_data_pb2.INT32: "INT32",
    hand_data_pb2.INT64: "INT64",
    hand_data_pb2.FLOAT: "FLOAT",
    hand_data_pb2.DOUBLE: "DOUBLE",
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
    hand_data_pb2.TCA6408A_OTHER: "TCA6408A_OTHER",
}

log_queue = queue.Queue()

def get_max_index(log_dir):
    max_index = -1
    pattern = re.compile(r"hand_msg_(\d+)\.json")
    for filename in os.listdir(log_dir):
        match = pattern.match(filename)
        if match:
            index = int(match.group(1))
            if index > max_index:
                max_index = index
    return max_index

# log function
def log_writer_thread(log_dir="logs"):
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    index = get_max_index(log_dir) + 1
    filename = os.path.join(log_dir, f"hand_msg_{index:02d}.json")

    # use a+
    with open(filename, "a+") as log_file:
        log_file.write('[')
        while True:
            msg = log_queue.get()
            if msg is None:
                print('log recv None. Leaving log...')
                break
            log_file.write('\n')
            json.dump(msg, log_file, indent=2)
            log_file.write(',')
            log_file.flush()  # Flush the file buffer to ensure data is written to disk
        
        # leaving
        with open(filename, "rb+") as log_file:
            log_file.seek(-1, os.SEEK_END)
            if log_file.read(1) == b',':
                log_file.seek(-1, os.SEEK_END)
            log_file.truncate()
            log_file.write(b'\n]')
            log_file.flush()

def accept(sock, mask):
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False)
    sel.register(conn, selectors.EVENT_READ, read)


def read(conn, mask):
    print(" " * 2)
    print_full_line()

    # Read the first 5 bytes
    try:
        initial_data = conn.recv(5)
        if len(initial_data) < 5:
            print("Failed to read the initial 5 bytes")
            sel.unregister(conn)
            conn.close()
            return

        # Parse the first byte (tag) and the next 4 bytes (fixed32)
        tag = initial_data[0]
        remaining_bytes = (
            struct.unpack("<I", initial_data[1:5])[0] - 5
        )  # Subtract the initial 5 bytes

        print(f"Tag: {tag}")
        print(f"Remaining bytes: {remaining_bytes}")

        # Read the remaining data, ensuring we read all the bytes
        data = bytearray()
        while len(data) < remaining_bytes:
            try:
                packet = conn.recv(remaining_bytes - len(data))
                if not packet:
                    print("Connection closed before all data was received")
                    sel.unregister(conn)
                    conn.close()
                    return
                data.extend(packet)
            except BlockingIOError:
                continue

        # Combine initial_data and data to form the complete message
        complete_data = initial_data + data

        try:
            print(f"Msg len: {len(complete_data)}")
            # print(binascii.hexlify(complete_data))

            # Deserialize handmsg
            msg = hand_data_pb2.HandMsg()
            msg.ParseFromString(complete_data)

            # # Print the deserialized message
            # print(f"Received message: {msg}")
            
            msg_dict = MessageToJson(msg)
            tmp_json_obj = json.loads(msg_dict)

            # Parse data messages
            for index, data_msg in enumerate(msg.data_wrapper.data_msgs):
                print(" ")
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
                    tmp_json_obj["dataWrapper"]["dataMsgs"][index]["data"] = list(decoded_data)
                    
                else:
                    print("Unknown data type")

                        
            log_queue.put(tmp_json_obj)

        except Exception as e:
            print(f"Failed to parse message: {e}")
            sel.unregister(conn)
            conn.close()
    except Exception as e:
        print(f"Exception during read: {e}")
        sel.unregister(conn)
        conn.close()


def print_full_line(char="#"):
    """Print a full line of the given character to fill the terminal width."""
    import shutil

    columns = shutil.get_terminal_size().columns
    print(char * columns)


def main():
    # start logging thread
    log_thread = threading.Thread(target=log_writer_thread)
    log_thread.start()
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((HOST, PORT))
    sock.listen()
    print(f"Server started on {HOST}:{PORT}")
    sock.setblocking(False)

    sel.register(sock, selectors.EVENT_READ, accept)

    try:
        while True:
            events = sel.select(timeout=1)  # Set a timeout for select
            for key, mask in events:
                callback = key.data
                callback(key.fileobj, mask)
    except KeyboardInterrupt:
        print("\nServer is shutting down...")
    finally:
        sel.close()
        sock.close()
        log_queue.put(None)
        log_thread.join()
        print("Server closed")


if __name__ == "__main__":
    main()
