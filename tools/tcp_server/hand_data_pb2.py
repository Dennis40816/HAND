# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: hand_data.proto
# Protobuf Python Version: 5.27.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    27,
    3,
    '',
    'hand_data.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0fhand_data.proto\"\xae\x01\n\x0bHandDataMsg\x12!\n\x06source\x18\x01 \x01(\x0e\x32\x11.HandChipInstance\x12 \n\tdata_type\x18\x02 \x01(\x0e\x32\r.HandDataType\x12\x12\n\ndata_count\x18\x03 \x01(\r\x12\x16\n\ttimestamp\x18\x04 \x01(\x03H\x00\x88\x01\x01\x12\x12\n\ntimestamps\x18\x05 \x03(\x03\x12\x0c\n\x04\x64\x61ta\x18\x06 \x01(\x0c\x42\x0c\n\n_timestamp\"B\n\rHandConfigMsg\x12!\n\x06target\x18\x01 \x03(\x0e\x32\x11.HandChipInstance\x12\x0e\n\x06\x63onfig\x18\x02 \x01(\x0c\"<\n\nHandCmdMsg\x12!\n\x06target\x18\x01 \x01(\x0e\x32\x11.HandChipInstance\x12\x0b\n\x03\x63md\x18\x02 \x01(\x0c\"2\n\x0fHandDataWrapper\x12\x1f\n\tdata_msgs\x18\x01 \x03(\x0b\x32\x0c.HandDataMsg\"8\n\x11HandConfigWrapper\x12#\n\x0b\x63onfig_msgs\x18\x01 \x03(\x0b\x32\x0e.HandConfigMsg\"/\n\x0eHandCmdWrapper\x12\x1d\n\x08\x63md_msgs\x18\x01 \x03(\x0b\x32\x0b.HandCmdMsg\"\x95\x02\n\x07HandMsg\x12\x13\n\x0b\x62ytes_count\x18\x01 \x01(\x07\x12$\n\tdirection\x18\x02 \x01(\x0e\x32\x11.HandMsgDirection\x12\"\n\x08msg_type\x18\x03 \x01(\x0e\x32\x10.HandMainMsgType\x12 \n\tchip_type\x18\x04 \x01(\x0e\x32\r.HandChipType\x12(\n\x0c\x64\x61ta_wrapper\x18\x05 \x01(\x0b\x32\x10.HandDataWrapperH\x00\x12,\n\x0e\x63onfig_wrapper\x18\x06 \x01(\x0b\x32\x12.HandConfigWrapperH\x00\x12&\n\x0b\x63md_wrapper\x18\x07 \x01(\x0b\x32\x0f.HandCmdWrapperH\x00\x42\t\n\x07\x63ontent*.\n\x10HandMsgDirection\x12\r\n\tFROM_HAND\x10\x00\x12\x0b\n\x07TO_HAND\x10\x01*0\n\x0fHandMainMsgType\x12\n\n\x06\x43ONFIG\x10\x00\x12\x08\n\x04\x44\x41TA\x10\x01\x12\x07\n\x03\x43MD\x10\x02*}\n\x0cHandChipType\x12\x11\n\rESP32_S3_MINI\x10\x00\x12\x0b\n\x07\x42Q27427\x10\x01\x12\x0e\n\nKX132_1211\x10\x02\x12\x0b\n\x07VL53L1X\x10\x03\x12\t\n\x05\x43H101\x10\x04\x12\x0b\n\x07\x42OS1901\x10\x05\x12\n\n\x06\x42MI323\x10\x06\x12\x0c\n\x08TCA6408A\x10\x07*\xa9\x03\n\x10HandChipInstance\x12\x16\n\x12\x45SP32_S3_MINI_MAIN\x10\x00\x12\x13\n\x0f\x42Q27427_BATTERY\x10\x01\x12\x16\n\x12KX132_1211_SENSOR1\x10\x02\x12\x16\n\x12KX132_1211_SENSOR2\x10\x03\x12\x16\n\x12KX132_1211_SENSOR3\x10\x04\x12\x16\n\x12KX132_1211_SENSOR4\x10\x05\x12\x13\n\x0fVL53L1X_SENSOR1\x10\x06\x12\x13\n\x0fVL53L1X_SENSOR2\x10\x07\x12\x11\n\rCH101_SENSOR1\x10\x08\x12\x11\n\rCH101_SENSOR2\x10\t\x12\x11\n\rCH101_SENSOR3\x10\n\x12\x11\n\rCH101_SENSOR4\x10\x0b\x12\x15\n\x11\x42OS1901_ACTUATOR1\x10\x0c\x12\x15\n\x11\x42OS1901_ACTUATOR2\x10\r\x12\x15\n\x11\x42OS1901_ACTUATOR3\x10\x0e\x12\x15\n\x11\x42OS1901_ACTUATOR4\x10\x0f\x12\x0e\n\nBMI323_IMU\x10\x10\x12\x12\n\x0eTCA6408A_CH101\x10\x11\x12\x12\n\x0eTCA6408A_OTHER\x10\x12*d\n\x0cHandDataType\x12\t\n\x05UINT8\x10\x00\x12\n\n\x06UINT16\x10\x01\x12\t\n\x05INT32\x10\x02\x12\t\n\x05INT64\x10\x03\x12\t\n\x05\x46LOAT\x10\x04\x12\n\n\x06\x44OUBLE\x10\x05\x12\x10\n\x0c\x43H101_SIMPLE\x10\x06\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'hand_data_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_HANDMSGDIRECTION']._serialized_start=765
  _globals['_HANDMSGDIRECTION']._serialized_end=811
  _globals['_HANDMAINMSGTYPE']._serialized_start=813
  _globals['_HANDMAINMSGTYPE']._serialized_end=861
  _globals['_HANDCHIPTYPE']._serialized_start=863
  _globals['_HANDCHIPTYPE']._serialized_end=988
  _globals['_HANDCHIPINSTANCE']._serialized_start=991
  _globals['_HANDCHIPINSTANCE']._serialized_end=1416
  _globals['_HANDDATATYPE']._serialized_start=1418
  _globals['_HANDDATATYPE']._serialized_end=1518
  _globals['_HANDDATAMSG']._serialized_start=20
  _globals['_HANDDATAMSG']._serialized_end=194
  _globals['_HANDCONFIGMSG']._serialized_start=196
  _globals['_HANDCONFIGMSG']._serialized_end=262
  _globals['_HANDCMDMSG']._serialized_start=264
  _globals['_HANDCMDMSG']._serialized_end=324
  _globals['_HANDDATAWRAPPER']._serialized_start=326
  _globals['_HANDDATAWRAPPER']._serialized_end=376
  _globals['_HANDCONFIGWRAPPER']._serialized_start=378
  _globals['_HANDCONFIGWRAPPER']._serialized_end=434
  _globals['_HANDCMDWRAPPER']._serialized_start=436
  _globals['_HANDCMDWRAPPER']._serialized_end=483
  _globals['_HANDMSG']._serialized_start=486
  _globals['_HANDMSG']._serialized_end=763
# @@protoc_insertion_point(module_scope)
