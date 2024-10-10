import hand_data_pb2
import struct

def create_sample_data_message():
    # Create an instance of HandDataMsg
    data_msg1 = hand_data_pb2.HandDataMsg()
    
    # Set the fields
    # data_msg1.source = hand_data_pb2.HandChipInstance.VL53L1X_SENSOR1
    # data_msg1.data_type = hand_data_pb2.HandDataType.FLOAT
    # data_msg1.data_count = 10
    data_msg1.timestamps.extend([1627891234 + i for i in range(10)])
    
    # Convert float data to bytes and extend data field
    # float_data1 = [float(i) for i in range(10)]
    # byte_data = struct.pack('10f', *float_data1)
    # data_msg1.data = byte_data

    return data_msg1

def create_sample_hand_message():
  
    # Create an instance of HandMsg
    hand_msg = hand_data_pb2.HandMsg()
    
    # Set the fields
    hand_msg.direction = hand_data_pb2.HandMsgDirection.FROM_HAND
    hand_msg.msg_type = hand_data_pb2.HandMainMsgType.DATA
    hand_msg.chip_type = hand_data_pb2.HandChipType.VL53L1X
    
    # Create and add data messages
    data_msg1 = hand_data_pb2.HandDataMsg()
    data_msg1.source = hand_data_pb2.HandChipInstance.VL53L1X_SENSOR1
    data_msg1.data_type = hand_data_pb2.HandDataType.FLOAT
    data_msg1.data_count = 10
    data_msg1.timestamps.extend([1627891234 + i for i in range(10)])
    
    # Convert float data to bytes and extend data field
    float_data1 = [float(i) for i in range(10)]
    byte_data = struct.pack('10f', *float_data1)
    data_msg1.data = byte_data
    
    data_msg2 = hand_data_pb2.HandDataMsg()
    data_msg2.source = hand_data_pb2.HandChipInstance.VL53L1X_SENSOR2
    data_msg2.data_type = hand_data_pb2.HandDataType.FLOAT
    data_msg2.data_count = 10
    data_msg2.timestamps.extend([1627891234 + i for i in range(10)])
    
    # Convert float data to bytes and extend data field
    float_data2 = [float(i) for i in range(10, 20)]
    data_msg2.data = struct.pack('10f', *float_data2)
    
    hand_msg.data_wrapper.data_msgs.extend([data_msg1, data_msg2])
    return hand_msg

def main():
    # Create a sample message
    sample_msg = create_sample_hand_message()
    
    # Serialize the message to a string
    serialized_msg = sample_msg.SerializeToString()
    
    # Deserialize the message from the string
    deserialized_msg = hand_data_pb2.HandMsg()
    deserialized_msg.ParseFromString(serialized_msg)
    
    # Print the original and deserialized messages
    print("Original message:")
    print(sample_msg)
    print("\nSerialized message (in hex):")
    print(serialized_msg.hex())
    print(f'Serialized len: {len(serialized_msg)}')
    print("\nDeserialized message:")
    print(deserialized_msg)
    
    # Create a sample message
    data_msg = create_sample_data_message()
    
    # Serialize the message to a string
    serialized_data_msg = data_msg.SerializeToString()
    
    # Deserialize the message from the string
    deserialized_data_msg = hand_data_pb2.HandDataMsg()
    deserialized_data_msg.ParseFromString(serialized_data_msg)
    
    # Print the original and deserialized messages
    print("Original data message:")
    print(data_msg)
    print("\nSerialized data message (in hex):")
    print(serialized_data_msg.hex())
    print("\nDeserialized message:")
    print(deserialized_data_msg)

if __name__ == "__main__":
    main()
