/*
 * Copyright (c) 2024 Dennis Liu, dennis48161025@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef LIB_BOS1901_BOS1901_ERR_H_
#define LIB_BOS1901_BOS1901_ERR_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Unified error codes for BOS1901 across multiple platforms.
   * This enumeration includes common error codes that can be used to represent
   * various error conditions in a platform-independent manner.
   */
  typedef enum
  {
    BOS1901_OK,           ///< Operation successful
    BOS1901_INVALID_ARG,  ///< Invalid parameter
    BOS1901_ERR_BUS,      ///< Communication failure
    BOS1901_BUSY,         ///< Device or resource busy
    BOS1901_TIMEOUT,      ///< Operation timed out
    BOS1901_UNKNOWN       ///< Unknown error
  } bos1901_err_t;

#ifdef __cplusplus
}
#endif

#endif