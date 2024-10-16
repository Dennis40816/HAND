#pragma once

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

/* @USER: Wi-Fi config */
#ifndef HAND_WIFI_MODULE_DEFAULT_SSID
#define HAND_WIFI_MODULE_DEFAULT_SSID ("a52s")
// #define HAND_WIFI_MODULE_DEFAULT_SSID ("602lab")
// #define HAND_WIFI_MODULE_DEFAULT_SSID ("CHT061975")
#endif

/* @USER: server config */
#ifndef HAND_WIFI_MODULE_DEFAULT_PASSWORD
#define HAND_WIFI_MODULE_DEFAULT_PASSWORD ("11111111")
// #define HAND_WIFI_MODULE_DEFAULT_PASSWORD ("602mems206")
// #define HAND_WIFI_MODULE_DEFAULT_PASSWORD ("24577079")
#endif

/* @USER: server config */
#ifndef HAND_DEFAULT_LOG_SERVER_IP
#define HAND_DEFAULT_LOG_SERVER_IP ("192.168.25.151")
// #define HAND_DEFAULT_LOG_SERVER_IP ("192.168.0.170")
// #define HAND_DEFAULT_LOG_SERVER_IP ("192.168.1.17")
#endif

#ifndef HAND_DEFAULT_LOG_SERVER_PORT
#define HAND_DEFAULT_LOG_SERVER_PORT (12345)
#endif

// control server is a.k.a data server
#ifndef HAND_DEFAULT_CONTROL_SERVER_IP
#define HAND_DEFAULT_CONTROL_SERVER_IP HAND_DEFAULT_LOG_SERVER_IP
#endif

#ifndef HAND_DEFAULT_CONTROL_SERVER_PORT
#define HAND_DEFAULT_CONTROL_SERVER_PORT (8055)
#endif