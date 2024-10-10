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
from datetime import datetime

# enable this when you want to debug
DEBUG_ENABLE = False


def debug_print(arg):
    if DEBUG_ENABLE:
        print(arg)


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
    hand_data_pb2.CH101_SIMPLE: "CH101_SIMPLE",
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
    message_num = 0

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    index = get_max_index(log_dir) + 1
    filename = os.path.join(log_dir, f"hand_msg_{index:02d}.json")

    # add a timer
    LOG_EVERY_N_MSGS = 10
    last_time = datetime.now()
    current_time = datetime.now()

    # use a+
    with open(filename, "a+") as log_file:
        log_file.write("[")

        # A forever loop for log queue
        while True:
            msg = log_queue.get()
            if msg is None:
                print("log recv None. Leaving log...")
                break
            message_num += 1
            log_file.write("\n")
            json.dump(msg, log_file, indent=2)
            log_file.write(",")
            log_file.flush()  # Flush the file buffer to ensure data is written to disk

            # let user know how many messages have been logged
            if message_num % LOG_EVERY_N_MSGS == 0:
                current_time = datetime.now()
                time_diff = (current_time - last_time).total_seconds()
                print(f"Logged {message_num} messages, {LOG_EVERY_N_MSGS} msgs took {time_diff:.2f}")
                last_time = current_time

        # leaving
        with open(filename, "rb+") as log_file:
            log_file.seek(-1, os.SEEK_END)
            if log_file.read(1) == b",":
                log_file.seek(-1, os.SEEK_END)
            log_file.truncate()
            log_file.write(b"\n]")
            log_file.flush()


def accept(sock, mask):
    conn, addr = sock.accept()
    print(f"Connected by {addr}")
    conn.setblocking(False)
    sel.register(conn, selectors.EVENT_READ, read)


def read(conn, mask):
    debug_print(" " * 2)
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

        debug_print(f"Tag: {tag}")
        debug_print(f"Remaining bytes: {remaining_bytes}")

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
            debug_print(f"Msg len: {len(complete_data)}")
            # print(binascii.hexlify(complete_data))

            # Deserialize handmsg
            msg = hand_data_pb2.HandMsg()
            msg.ParseFromString(complete_data)

            msg_dict = MessageToJson(msg)
            tmp_json_obj = json.loads(msg_dict)

            # Parse data messages
            for index, data_msg in enumerate(msg.data_wrapper.data_msgs):
                debug_print(" ")
                data_type_str = data_type_map.get(data_msg.data_type, "Unknown")
                source_str = source_map.get(data_msg.source, "Unknown")
                debug_print(f"Source: {source_str}")
                debug_print(f"Data Type: {data_type_str}")
                debug_print(f"Data Count: {data_msg.data_count}")
                debug_print(f"Timestamps: {data_msg.timestamps}")

                # Decode data
                decoded_data = decode_data(data_msg)
                tmp_json_obj["dataWrapper"]["dataMsgs"][index]["data"] = decoded_data

            log_queue.put(tmp_json_obj)

        except Exception as e:
            print(f"Failed to parse message: {e}")
            sel.unregister(conn)
            conn.close()
    except Exception as e:
        print(f"Exception during read: {e}")
        sel.unregister(conn)
        conn.close()


def decode_data(data_msg):
    """Decode data based on data_type using either base_decode or custom_decode."""
    simple_data_types = [
        hand_data_pb2.UINT8,
        hand_data_pb2.UINT16,
        hand_data_pb2.INT32,
        hand_data_pb2.INT64,
        hand_data_pb2.FLOAT,
        hand_data_pb2.DOUBLE,
    ]

    if data_msg.data_type in simple_data_types:
        data_format_map = {
            hand_data_pb2.UINT8: f"{data_msg.data_count}B",
            hand_data_pb2.UINT16: f"{data_msg.data_count}H",
            hand_data_pb2.INT32: f"{data_msg.data_count}i",
            hand_data_pb2.INT64: f"{data_msg.data_count}q",
            hand_data_pb2.FLOAT: f"{data_msg.data_count}f",
            hand_data_pb2.DOUBLE: f"{data_msg.data_count}d",
        }
        data_format = data_format_map.get(data_msg.data_type, "")
        return base_decode(data_format, data_msg.data)

    elif data_msg.data_type == hand_data_pb2.CH101_SIMPLE:
        return custom_decode(data_msg.data_type, data_msg.data)

    else:
        print(f"Unknown data type: {data_msg.data_type}")
        return []


def base_decode(data_format, data):
    try:
        decoded_data = struct.unpack(data_format, data)
        return list(decoded_data)
    except struct.error as e:
        print(f"Error in base_decode: {e}")
        return []


def custom_decode(data_type, data):
    if data_type == hand_data_pb2.CH101_SIMPLE:
        struct_format = "HHf"  # 對應 hand_chx01_simple_data_unit_t 中的 sample_num, amp, range
        unit_size = struct.calcsize(struct_format)
        
        # 初始化字典，每個 key 對應一個列表
        decoded_data = {
            "sample_num": [],
            "amp": [],
            "range": []
        }

        for i in range(0, len(data), unit_size):
            unit_data = struct.unpack(struct_format, data[i:i + unit_size])
            decoded_data["sample_num"].append(unit_data[0])
            decoded_data["amp"].append(unit_data[1])
            decoded_data["range"].append(unit_data[2])

        return [decoded_data]  # 返回包含一個字典的列表

    else:
        print(f"Unknown custom data type: {data_type}")
        return []



def print_full_line(char="#"):
    """Print a full line of the given character to fill the terminal width."""
    import shutil

    columns = shutil.get_terminal_size().columns
    debug_print(char * columns)


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
