# HAND Firmware

## Part test

- [CH-101 on ESP32-S3-MINI](https://github.com/Dennis40816/HAND/tree/port_ch101/doc/part_test/port_ch101)

## TODO list

### chore

- Add .clang-format-ignore file
- Add auto format when pio build

### lib

- Can we use `GPIO_MODE_OUTPUT_INPUT` for ch101 lib?

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
  pio test -e hand_test -vvv
  ```

## Platform dependent code

- Put an `extra_script.py` in lib folder to change `pio's src_filter`