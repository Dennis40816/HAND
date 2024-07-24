#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "esp_vfs.h"
#include "esp_vfs_eventfd.h"
#include "errno.h"
#include <inttypes.h>

// Buffer size for runtime stats
#define RUNTIME_STATS_BUFFER_SIZE 2048

// Select timeout settings
#define SELECT_TIMEOUT_SECONDS      5
#define SELECT_TIMEOUT_MICROSECONDS 0

// Task stack size
#define TASK_STACK_SIZE 2048

// Timer interval for printing runtime stats in milliseconds
#define STATS_TIMER_INTERVAL_MS 10000

// Task priority
#define TASK_PRIORITY tskIDLE_PRIORITY

// Event file descriptor
int event_fd;

// Test task handle
TaskHandle_t xTestTaskHandle = NULL;

// Message task handle
TaskHandle_t xMessageTaskHandle = NULL;

// Timer handle for printing runtime stats
TimerHandle_t xStatsTimerHandle = NULL;

// Buffer for runtime stats
char cRunTimeStatsBuffer[RUNTIME_STATS_BUFFER_SIZE];

static const char *TAG = "main";

/**
 * Initialize an event file descriptor
 * @return int: file descriptor, or -1 on failure
 */
int initialize_event_fd()
{
  esp_vfs_eventfd_config_t config = {.max_fds = 4};

  esp_vfs_eventfd_register(&config);
  int fd = eventfd(0, 0);
  if (fd == -1)
  {
    ESP_LOGE(TAG, "Failed to create eventfd: %s", strerror(errno));
    return -1;
  }
  return fd;
}

/**
 * Test task function
 * @param pvParameters: Task parameters (not used in this case)
 */
void vTestTask(void *pvParameters)
{
  while (1)
  {
    // Initialize the file descriptor set
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(event_fd, &readfds);  // Monitor the event file descriptor

    // Set timeout for select
    struct timeval tv;
    tv.tv_sec = SELECT_TIMEOUT_SECONDS;
    tv.tv_usec = SELECT_TIMEOUT_MICROSECONDS;

    int retval;

    // Monitor file descriptor changes using select
    retval = select(event_fd + 1, &readfds, NULL, NULL, &tv);

    if (retval == -1)
    {
      if (errno == EINTR)
      {
        // If select is interrupted by a signal, retry
        continue;
      }
      else
      {
        ESP_LOGE(TAG, "select() error: %s", strerror(errno));
        break;
      }
    }
    else if (retval)
    {
      ESP_LOGI(TAG, "Data is available now.");
      uint64_t buffer;
      ssize_t read_bytes = read(event_fd, &buffer, sizeof(buffer));
      if (read_bytes > 0)
      {
        ESP_LOGI(TAG, "Message: %" PRIu64, buffer);
      }
    }
    else
    {
      ESP_LOGI(TAG, "No data within the timeout period.");
    }

    // // Delay for a while before next iteration
    // vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Close the event file descriptor
  close(event_fd);
}

/**
 * Message task function
 * @param pvParameters: Task parameters (not used in this case)
 */
void vMessageTask(void *pvParameters)
{
  while (1)
  {
    const uint64_t message = 1;
    write(event_fd, &message, sizeof(message));
    ESP_LOGI(TAG, "Message sent to event FD.");

    // Delay for 6 seconds before sending the next message
    vTaskDelay(pdMS_TO_TICKS(6000));
  }

  // Close the event file descriptor
  close(event_fd);
}

/**
 * Timer callback function for printing runtime stats
 * @param xTimer: Timer handle
 */
void vStatsTimerCallback(TimerHandle_t xTimer)
{
  // Get and print runtime stats
  vTaskGetRunTimeStats(cRunTimeStatsBuffer);
  ESP_LOGI(TAG, "Run Time Stats:\n%s", cRunTimeStatsBuffer);
}

/**
 * Main application function
 */
void app_main(void)
{
  vTaskDelay(pdMS_TO_TICKS(3000));
  ESP_LOGI(TAG, "Starting...");
  // Initialize the event file descriptor
  event_fd = initialize_event_fd();
  if (event_fd == -1)
  {
    ESP_LOGE(TAG, "Failed to initialize event file descriptor.");
    return;
  }

  // Create the test task
  xTaskCreate(vTestTask, "TestTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY,
              &xTestTaskHandle);

  // Create the message task
  xTaskCreate(vMessageTask, "MessageTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY,
              &xMessageTaskHandle);

  // Create the timer for printing runtime stats every 10 seconds
  xStatsTimerHandle =
      xTimerCreate("StatsTimer", pdMS_TO_TICKS(STATS_TIMER_INTERVAL_MS), pdTRUE,
                   (void *)0, vStatsTimerCallback);

  if (xStatsTimerHandle != NULL)
  {
    // Start the timer
    xTimerStart(xStatsTimerHandle, 0);
  }
}
