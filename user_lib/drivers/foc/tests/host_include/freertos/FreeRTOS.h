#ifndef FOC_HOST_FREERTOS_H
#define FOC_HOST_FREERTOS_H

#include <cstdint>

using BaseType_t = int32_t;
using UBaseType_t = uint32_t;
using TickType_t = uint32_t;

constexpr BaseType_t pdFALSE = 0;
constexpr BaseType_t pdTRUE = 1;
constexpr BaseType_t pdPASS = 1;

#define portYIELD_FROM_ISR(value) ((void)(value))

#endif
