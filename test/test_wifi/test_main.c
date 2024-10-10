#include "unity.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcp_client.h"

#define TEST_DATA "Hello, HAND Wi-Fi Test!"
#define BUF_SIZE  128

// Function to set up before each test case
void setUp(void)
{
  // Initialize NVS (necessary setup)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Connect to WiFi
  connect_to_wifi();
}

// Function to clean up after each test case
void tearDown(void)
{
  // Clean up resources
  nvs_flash_deinit();
}

// Test case function for TCP echo server connection and response
void test_tcp_echo_server_connection(void)
{
  char recv_buf[BUF_SIZE];

  // Send data and receive response
  tcp_client_send(TEST_DATA, recv_buf, BUF_SIZE);

  // Verify that the response matches the sent data
  TEST_ASSERT_EQUAL_STRING(TEST_DATA, recv_buf);
}

// Main test runner function
void app_main(void)
{
  UNITY_BEGIN();  // Initialize Unity test framework

  // Register and run the test case
  RUN_TEST(test_tcp_echo_server_connection);

  UNITY_END();  // Terminate Unity test framework
}
