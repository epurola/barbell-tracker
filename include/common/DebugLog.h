#pragma once

#ifndef VBT_DEBUG_SERIAL
#define VBT_DEBUG_SERIAL 0
#endif

#if VBT_DEBUG_SERIAL
    #include <Arduino.h>
    #define VBT_LOG_BEGIN(baud) do { Serial.begin(baud); delay(1000); } while (0)
    #define VBT_LOGLN(msg)      Serial.println(msg)
    #define VBT_LOGF(fmt, ...)  Serial.printf((fmt), ##__VA_ARGS__)
#else
    #define VBT_LOG_BEGIN(baud) ((void)0)
    #define VBT_LOGLN(msg)      ((void)0)
    #define VBT_LOGF(fmt, ...)  ((void)0)
#endif
