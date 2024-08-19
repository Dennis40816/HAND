#include "hand_util.h"

#include <string.h>

size_t hand_util_to_bit_str(uint64_t number, char *buffer, size_t buf_size)
{
  // Determine the number of bits in the given number type
  size_t bit_length = 64;

  if (number <= UINT8_MAX)
  {
    bit_length = 8;
  }
  else if (number <= UINT16_MAX)
  {
    bit_length = 16;
  }
  else if (number <= UINT32_MAX)
  {
    bit_length = 32;
  }

  size_t binary_str_len =
      bit_length            // all bits
      + (bit_length / 4)    // space for every four bits
      + (bit_length / 8) +  // additional space for every eight bits
      1;                    // +1 for null terminator

  // Ensure buffer is large enough
  if (buf_size < binary_str_len)
  {
    return 0;  // Buffer too small
  }

  // Initialize buffer index and set pointer to the end of the buffer
  size_t index = 0;
  buffer[index] = '\0';  // Null-terminate the string

  // Loop over each bit
  for (int i = bit_length - 1; i >= 0; i--)
  {
    // Append '1' or '0' based on the current bit
    buffer[index++] = (number & (1ULL << i)) ? '1' : '0';

    // Insert a space every 4 bits
    if (i % 4 == 0 && i != 0)
    {
      buffer[index++] = ' ';
    }
    // Insert an extra space every 8 bits
    if (i % 8 == 0 && i != 0)
    {
      buffer[index++] = ' ';
    }
  }

  buffer[index] = '\0';  // Null-terminate the string
  return index;  // Return the length of the string (excluding null terminator)
}