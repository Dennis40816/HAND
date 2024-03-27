# sonclib v2


## Building

You must set the environment variable `ARM_GCC_PATH` to the path containing the arm-none-eabi installation. For example, if your arm-none-eabi-gcc executable is in C:\toolchains\arm-gnu\bin, set `ARM_GCC_PATH` to C:\toolchains\arm-gnu.

You must also set the processor specific toolchain file. This is passed as an option to the configure.sh script.

Use the configure.sh script to set up a build directory, e.g., `sh configure.sh -t CMakeToolchains/mytoolchain.cmake RelWithDebInfo "MinGW Makefiles"`.

## Inclusion

This project is intended to be included in a main application project. The easiest way to do that is to use the `add_subdirectory` CMake command from the main application's CMakeLists.txt file. Then use `target_link_libraries` to link this library into the main application. You will need to set up CMake variables pointing to the locations of the necessary header files, as described in the Dependencies section. You will also need to link the required external modules, which are also described
below.

## Dependencies

The following headers must be available at compile time.

- `chirp_board_config.h`, as required by soniclib. Set the CHIRP_BOARD_CONFIG_DIR CMake variable to the path that includes this file.

The following symbols are undefined by this library and must be defined by external modules at link time.

```
         U chbsp_delay_ms
         U chbsp_delay_us
         U chbsp_group_io_clear
         U chbsp_group_io_set
         U chbsp_group_pin_init
         U chbsp_group_set_io_dir_in
         U chbsp_group_set_io_dir_out
         U chbsp_i2c_get_info
         U chbsp_i2c_init
         U chbsp_i2c_mem_read
         U chbsp_i2c_mem_write
         U chbsp_i2c_read
         U chbsp_i2c_reset
         U chbsp_i2c_write
         U chbsp_program_disable
         U chbsp_program_enable
         U chbsp_reset_assert
         U chbsp_reset_release
         U memcpy
         U printf
```

The `chbsp_*` symbols are defined by the chirp board support implementation.


The remaining symbols are part of the standard libraries.
