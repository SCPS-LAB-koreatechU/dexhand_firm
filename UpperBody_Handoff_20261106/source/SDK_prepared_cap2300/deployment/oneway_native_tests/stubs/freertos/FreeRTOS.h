#pragma once
#include <mutex>
#include <condition_variable>
struct NativeTask {std::mutex mutex;std::condition_variable cv;unsigned notifications=0;};
using TaskHandle_t=NativeTask*;
using portMUX_TYPE=std::mutex;
using BaseType_t=int;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
#define pdPASS 1
#define pdTRUE 1
#define portMAX_DELAY 0xffffffffu
