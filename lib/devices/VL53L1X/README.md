# VL53L1X Driver

## Folder Structure

- `st`: Contains the official API provided by ST ([STSW-IMG007](https://www.st.com/en/embedded-software/stsw-img007.html))
- `platform`: Contains platform-specific code, with subfolders named according to the platform, such as `espidf`. This folder will provide default implementations for the platform (weak linkage, which will be overridden by implementations in the `bsp` folder). 

    Use following flags in section `build_flags` in `platformio.ini` 

    ```ini
    -DLIB_PLATFORM=espidf
    ;; for bsp config e.g.,VL53L1X
    -DLIB_BSP_BOARD=hand ; use hand main dev board (self made)
    ```

- `source`: Contains extended libraries derived from the official ST API to simplify the code users need to write.
- `bsp`: Since different boards may have slight variations in GPIO control for VL53L1X (e.g., using an IO expander to control XSHUT), this folder allows implementations to override those in the `platform` folder.
