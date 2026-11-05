#pragma once
#include "FreeRTOS.h"
#include <thread>
#include <chrono>
inline thread_local NativeTask* currentNativeTask=nullptr;
inline BaseType_t xTaskCreatePinnedToCore(void(*fn)(void*),const char*,unsigned,void* arg,unsigned,TaskHandle_t* out,int core){
 if(core!=0)return 0;*out=new NativeTask();auto task=*out;
 std::thread([=]{currentNativeTask=task;fn(arg);}).detach();return pdPASS;
}
inline void xTaskNotifyGive(TaskHandle_t task){std::lock_guard<std::mutex> lock(task->mutex);++task->notifications;task->cv.notify_all();}
inline unsigned ulTaskNotifyTake(int,unsigned){auto t=currentNativeTask;std::unique_lock<std::mutex> lock(t->mutex);t->cv.wait(lock,[&]{return t->notifications>0;});auto n=t->notifications;t->notifications=0;return n;}
inline void vTaskDelay(unsigned n){std::this_thread::sleep_for(std::chrono::milliseconds(n));}
