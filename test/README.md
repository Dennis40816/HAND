# HAND TEST

## How to run the test

- 在 platformio.ini 裡面透過 test_filter 指定目標
- 接著使用開啟 `platform io CLI` 執行 `pio test -e hand_test -vv`

## `test_rgb_led`

- 測試 RGB LED 是否正常
- 要求: 需要先完成對 hand_firmware 的建置(Full clean -> build),
  確保出現資料夾 `managed_components\espressif__led_strip` 才能正確編譯

## `test_bmi323_id`

- 測試 FPC 板子上的 BMI323 是否通訊正常
- 要求: 需要接上 HAND_FPC 板

## `test_bos1901_id`

- 測試主板上的 BOS1901 群組是否通訊正常
- 要求: 無

## `test_bq27427_id`

- 測試主板上的 BQ27427 (Fuel Gauge) 是否通訊正常
- 要求: 執行此測試前，請先接上電池以使 BQ27427 啟動，否則測試將失敗

## `test_kx132_1211_id`

- 測試 FPC 板子上的 KX132-1211 群組通訊是否正常
- 要求: 需要接上 HAND_FPC 板

## `test_vl53l1x_id`

- 測試 FPC 板子上的 VL53L1X 群組通訊是否正常
- 要求: 需要接上 HAND_FPC 板

## `test_wifi`

- 執行 test_wifi 之前，需要先打開一個監聽 port `6020` 的 tcp server，測試資料夾中 `test/test_wifi` 已經有一個範例
- 請使用另一個 terminal 先執行指令 `python test/test_wifi/tcp_server.py` 再開始測試
- 可以在 platformio.ini 中 env `hand_test` 的 build_flags 設定以下三個參數，避免在測試過程中手動輸入參數

  ```ini
    ;;; example: explicitly define test_wifi macros (if you don't want to manually set)
    -DTEST_HAND_TCP_ECHO_SERVER_IP=\"192.168.1.17\"
    -DTEST_HAND_WIFI_SSID=\"YourSSID\" ; must be 2.4G wifi
    -DTEST_HAND_WIFI_PASSWORD=\"YourPassWord\"
  ```

## `hand_test_hook.py`

- 這個 script 用來設定跟 test 有關的參數，允許使用者動態輸入諸如 `wifi SSID`, `PASSWORD`，如果使用者並未在 `platformio.ini` 中定義的情況下。
