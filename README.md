# HAND Firmware

This repo contains the firmware code for the HAND main board (esp32-s3-mini). The following repositories are related to making the HAND platform work:

- [HAND firmware (current repo)](https://github.com/Dennis40816/HAND)
- [HAND main board PCB files](https://github.com/Dennis40816/hand_main_pcb)
- [HAND FPC board PCB files](https://github.com/Dennis40816/hand_fpc_pcb)
- [HAND app (written in Flutter)](https://github.com/Dennis40816/hand_app)

## Install Dependencys

### Optional

- Local

  - [LLVM (clang-format)](https://llvm.org/builds/)

## Nanopb & ProtoBuf

- For nanopb and c output, do:

  ```bash
  # make sure you are in the root of project

  # open platformio CLI

  # generate by nanopb_generator.py
  python .pio\libdeps\hand_firmware\Nanopb\generator\nanopb_generator.py src\hand_modules\hand_data\proto\hand_data.proto
  ```

- For protobuf and python output, do

  - make sure you already install protoc and it's available cmd from your terminal

  ```bash
  # make sure you are in the root of project

  # open platformio CLI

  # make sure you have protobuf in python
  pip install protobuf

  # compile protobuf
  protoc -I=src/hand_modules/hand_data/proto --python_out=tools/tcp_server src/hand_modules/hand_data/proto/hand_data.proto
  ```

## Part test

- [CH-101 on ESP32-S3-MINI](https://github.com/Dennis40816/HAND/tree/port_ch101/doc/part_test/port_ch101)

## TODO list

### chore

- Add .clang-format-ignore file
- Add auto format when pio build

### lib

- TO CHECL: Can we use `GPIO_MODE_OUTPUT_INPUT` for ch101 lib?

## About build flags

- In platformio.ini, we can add build flags like this:

  - `-DLIB_PLATFORM=espidf`: This define specifies the target platform for all libraries in the lib folder. In the current version, all libraries are written targeting the espidf platform.

## Test

- [PIO test hierarchy](https://docs.platformio.org/en/stable/advanced/unit-testing/structure/hierarchy.html)
- Using `test_filter` when you only want to test specific dir:

  ```
  test_filter = test_tca6408a ;disable this if you want to go through all tests
  ```

- Subfolder must start with `test_...`
- how to run pio test?

  ```
  pio test -e hand_test -vv
  ```

## Platform dependent code

- When using python, please choose the correct platform io python env:

  - e.g., `C:\Users\USER\.platformio\penv\Scripts\python.exe`. You need to find your own path.
  - use pip to install `termcolor`

- Put an `extra_script.py` in lib folder to change `pio's src_filter`

## How to get build logs

- In powershell, do the following instruction

  ```ps1
  pio run -e hand_firmware -vv 2>&1 | Tee-Object -FilePath <log-name>
  ```

## Notes on Writing `extra_script.py` in the Library

- In the library, ensure that all required files (including `.c` files) have their parent folders included in either:

  1. The `build` flags in `library.json` (`-I` flags)
  2. Using `env.Append(CPPPATH=[absolute_path])` in `extra_script.py`

  Choose one of these methods. If dynamic adjustments are needed (e.g., through platformio.ini `build_flags`), it can be implemented in `extra_script.py`. For an example, refer to VL53L1X.

- Ensure that `src` is not used as the top-level folder name in the library. If `src` must be used as the top-level folder, ensure that it contains only `src` and `inc` folders. Adding other folders and referencing them from `src` can lead to compilation errors (e.g., undefined reference error). For an example, see the VL53L1X library; changing the `source` to `src` can cause compilation errors.

- srcFilter 的功能是決定要不要讓該資料夾裡面的內容進行編譯(推測是產生 .o)，而 -I 則決定其他文件是否在 include 階段能夠看見該文件。因此兩者都需要在 extra_script.py 進行配置(詳見 VL53L1X Lib)

## 如果 include path 沒有更新

- include path 的更新取決於兩點

  - 位於 extra_script.py 的 include path => 需要重新 build
  - 位於 library.json, CMakeLists.txt, platformio.ini 的 build flags `-I` => 需要偵測到 platformio.ini 更新時才會改動，可以簡單的刪除一個符號再重新加上後儲存即可

## 觀察

- 直接 include 能夠覆蓋，但是隔著檔案 include 則不能覆蓋 => 並不是這樣的，而是到底是在 header 還是 src 引入實作要用的 header 會影響 macro 內部是否能夠覆蓋

## Alias Name List

- PPB: Ping Pong Buffer
- HAND: Haptic Assitance Navigation Device
