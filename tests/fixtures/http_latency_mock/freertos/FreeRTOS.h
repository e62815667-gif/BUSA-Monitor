#pragma once
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <cassert>
using BaseType_t=int;using TickType_t=unsigned;
constexpr BaseType_t pdTRUE=1,pdPASS=1;constexpr TickType_t portMAX_DELAY=~0u;
namespace fake_rtos {
struct Queue{std::mutex mutex;std::condition_variable cv;size_t size;bool full=false;unsigned char bytes[sizeof(void*)]={};explicit Queue(size_t n):size(n){assert(n<=sizeof bytes);}};
struct Semaphore{std::mutex mutex;std::condition_variable cv;bool ready=false;};
struct Stop{};
inline std::atomic<bool>& stop(){static std::atomic<bool> s{false};return s;}
inline std::thread& worker(){static std::thread t;return t;}
inline bool& failStart(){static bool b=false;return b;}
inline void join(){stop()=true;worker().join();stop()=false;}
}
using QueueHandle_t=fake_rtos::Queue*;using SemaphoreHandle_t=fake_rtos::Semaphore*;using TaskHandle_t=void*;
inline QueueHandle_t xQueueCreate(unsigned n,size_t size){assert(n==1);return new fake_rtos::Queue(size);}
inline void vQueueDelete(QueueHandle_t q){delete q;}
inline BaseType_t xQueueSend(QueueHandle_t q,const void* p,TickType_t){std::unique_lock<std::mutex> l(q->mutex);q->cv.wait(l,[&]{return !q->full;});memcpy(q->bytes,p,q->size);q->full=true;l.unlock();q->cv.notify_all();return pdTRUE;}
inline BaseType_t xQueueReceive(QueueHandle_t q,void* p,TickType_t timeout){std::unique_lock<std::mutex> l(q->mutex);if(timeout==0&&!q->full)return 0;q->cv.wait(l,[&]{return q->full;});memcpy(p,q->bytes,q->size);q->full=false;l.unlock();q->cv.notify_all();return pdTRUE;}
inline SemaphoreHandle_t xSemaphoreCreateBinary(){return new fake_rtos::Semaphore;}
inline void vSemaphoreDelete(SemaphoreHandle_t s){delete s;}
inline BaseType_t xSemaphoreTake(SemaphoreHandle_t s,TickType_t){std::unique_lock<std::mutex> l(s->mutex);s->cv.wait(l,[&]{return s->ready;});s->ready=false;return pdTRUE;}
inline BaseType_t xSemaphoreGive(SemaphoreHandle_t s){std::lock_guard<std::mutex> l(s->mutex);s->ready=true;s->cv.notify_one();return pdTRUE;}
inline TaskHandle_t xTaskGetCurrentTaskHandle(){static thread_local int marker;return &marker;}
inline BaseType_t xTaskCreatePinnedToCore(void(*fn)(void*),const char*,unsigned,void* arg,unsigned,TaskHandle_t*,int){if(fake_rtos::failStart())return 0;fake_rtos::worker()=std::thread([=]{try{fn(arg);}catch(const fake_rtos::Stop&){};});return pdPASS;}
inline void vTaskDelay(unsigned n){if(fake_rtos::stop())throw fake_rtos::Stop{};std::this_thread::sleep_for(std::chrono::milliseconds(n));}
