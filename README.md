# HAND Firmware

This repo contains the firmware code for the HAND main board (esp32-s3-mini). The following repositories are related to making the HAND platform work:

- HAND firmware (current repo)
- HAND main board PCB files
- HAND FPC board PCB files
- HAND app (written in Flutter)

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
