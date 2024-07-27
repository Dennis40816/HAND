## CHX01 Group Test

- 如何配置 CHx01?

  1. 配置文件如下:
  
     a. `lib\devices\CH101\application\inc\app_config.h`: 該文件配置:

     - `CHIRP_SENSOR_FW_INIT_FUNC`: CHx01 的內部韌體 (e.g., `ch101_gpr_sr_narrow_init`)
     - `CHIRP_FIRST_SENSOR_MODE` && `CHIRP_OTHER_SENSOR_MODE`: CHx01 的 mode (e.g., `CH_MODE_TRIGGERED_TX_RX`, `CH_MODE_FREERUN`)
     - `CHIRP_SENSOR_MAX_RANGE_MM`: CHx01 的最大測距 (e.g., 500 (mm))，該選項與內部韌體也會有關係
     - `CHIRP_SENSOR_STATIC_RANGE`: CHx01 的靜態拒距，如果有要靜止不納入量測的距離，請將該選項配置為 0
     - `MEASUREMENT_INTERVAL_MS`: 量測間隔 (ms)
     - 剩下的可以自己看 (包括需要輸出格式...)

     b. `lib\devices\CH101\board\config\chirp_board_config.h`: 該文件配置:

     - `CHIRP_MAX_NUM_SENSORS`: CHx01 group 允許有最大的設備數量，預設為 4
     - `CHIRP_USE_NUM_SENSORS`: CHx01 group 實際會使用的數量，也就是陣列的前 n 個會實際通訊與擷取數據，所以如果有壞掉的，可以移到陣列(如 `CHIRP_PIN_PROG`)後方，並調整本參數 (<= `CHIRP_MAX_NUM_SENSORS`)
     - `CHIRP_NUM_I2C_BUSES`: CHx01 group 會用到的 I2C bus 總數量
     - `CHIRP_PIN_PROG`: CHx01 group 使用到的 PROG pin (在這裡是 ESP-IDF 的 GPIO_NUM_X)
     - `CHIRP_IO_PROG`: CHx01 group 使用到的中斷 pin (在這裡是 ESP-IDF 的 GPIO_NUM_X)
     - `CHIRP_PIN_LED`: CHx01 group 使用到的 LED (在這裡沒有用到)，此處設定為 CHIRP_DUMMY_PIN (1 << 6)。需要特別說明的是目前使用一個特大的值 (1 << 6) 表示 DUMMY_PIN，由於該值超過 esp32 s3 所擁有的腳位數量，因此不會產生任何效果，如果後續要實作，需要在 `gpio_isr_handler_add` 先初始化所有 LED 腳位，並修改相關的 LED function e.g., `chbsp_led_off()` (應該要確認腳位是否有效...)

     c. `lib\devices\CH101\board\config\chirp_smartsonic.h`: 該文件配置:

     - `CHIRP_I2C_ADDRS`: CHx01 group 實際的 I2C 地址 (7-bit address)，最大長度受 `CHIRP_MAX_NUM_SENSORS` 限制
     - `CHIRP_I2C_BUSES`: CHx01 group 的通信 I2C 總線編號 (如 `I2C_NUM_0` for ESP-IDF)

     d. `lib\devices\CH101\board\config\conf_hand_espidf_board.h`: 該文件依照 HAND 的硬體走線配置 (詳情參考 layout 文件)，應無需變動
