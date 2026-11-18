#pragma once

#include "sdkconfig.h"

// Shared scan preparation for LCD targets. Each SoC supplies its own transport.
#if !CONFIG_IDF_TARGET_ESP32S3 && !CONFIG_IDF_TARGET_ESP32S31
#error "this epdiy fork only supports ESP32-S3 and ESP32-S31 LCD rendering"
#endif
#define RENDER_METHOD_LCD 1

#ifdef __clang__
#define IRAM_ATTR
// define this if we're using clangd to make it accept the GCC builtin
void __assert_func(const char* file, int line, const char* func, const char* failedexpr);
#endif
