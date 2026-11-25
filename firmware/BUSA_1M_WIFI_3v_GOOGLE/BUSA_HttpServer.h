#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <atomic>
#include <cstring>

// Network parsing and socket writes belong to one worker. Existing route and
// upload callbacks still run exclusively on the Arduino application task.
// The queue and completion semaphore hand over the WebServer state; neither
// task touches that state while the other owns it. No retry of a command.
class BusaHttpServer final : public WebServer {
public:
    static constexpr size_t responseCapacity=512u*1024u;
    explicit BusaHttpServer(int port):WebServer(port){}

    void on(const Uri& uri, THandlerFunction fn){on(uri,HTTP_ANY,fn);}
    void on(const Uri& uri, HTTPMethod method,THandlerFunction fn){
        WebServer::on(uri,method,wrap(fn));
    }
    void on(const Uri& uri, HTTPMethod method,THandlerFunction fn,THandlerFunction upload){
        WebServer::on(uri,method,wrap(fn),wrap(upload));
    }
    void onNotFound(THandlerFunction fn){WebServer::onNotFound(wrap(fn));}
    void onFileUpload(THandlerFunction fn){WebServer::onFileUpload(wrap(fn));}

    bool startWorker(){
        if(enabled_)return true;
        application_=xTaskGetCurrentTaskHandle();
        buffer_=static_cast<uint8_t*>(heap_caps_malloc(responseCapacity,MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));
        requests_=xQueueCreate(1,sizeof(const THandlerFunction*));
        done_=xSemaphoreCreateBinary();
        if(buffer_&&requests_&&done_&&xTaskCreatePinnedToCore(taskEntry,"busa-http",12288,this,1,nullptr,0)==pdPASS){
            enabled_=true;return true;
        }
        if(requests_)vQueueDelete(requests_);
        if(done_)vSemaphoreDelete(done_);
        free(buffer_);buffer_=nullptr;requests_=nullptr;done_=nullptr;
        return false; // Preserve local API in explicit synchronous fallback.
    }
    bool workerEnabled()const{return enabled_;}
    uint32_t workerWorstMs()const{return workerWorst_.load(std::memory_order_relaxed);}
    uint32_t responseFailures()const{return responseFailures_.load(std::memory_order_relaxed);}
    void pollApplication(){
        if(!enabled_){WebServer::handleClient();return;}
        const THandlerFunction* fn=nullptr;
        if(xQueueReceive(requests_,&fn,0)!=pdTRUE)return;
        capturing_=true;
        (*fn)();
        capturing_=false;
        xSemaphoreGive(done_);
    }
protected:
    size_t _currentClientWrite(const char* data,size_t length) override {
        return capturing_?capture(data,length):WebServer::_currentClientWrite(data,length);
    }
    size_t _currentClientWrite_P(PGM_P data,size_t length) override {
        // ESP32 PROGMEM is memory mapped. The buffer owns an exact copy until
        // the socket write finishes, including dynamically generated headers.
        return capturing_?capture(data,length):WebServer::_currentClientWrite_P(data,length);
    }
private:
    THandlerFunction wrap(THandlerFunction fn){
        if(!fn)return fn;
        return [this,fn](){dispatch(fn);};
    }
    size_t capture(const char* data,size_t length){
        if(overflow_||length>responseCapacity-used_){overflow_=true;return 0;}
        if(length)memcpy(buffer_+used_,data,length);
        used_+=length;return length;
    }
    void dispatch(const THandlerFunction& fn){
        if(xTaskGetCurrentTaskHandle()==application_){fn();return;}
        used_=0;overflow_=false;
        const THandlerFunction* ptr=&fn;
        if(xQueueSend(requests_,&ptr,portMAX_DELAY)!=pdTRUE)return;
        // Do not abandon a queued callback: its arguments and upload buffer
        // stay owned by WebServer until the application acknowledges it.
        if(xSemaphoreTake(done_,portMAX_DELAY)!=pdTRUE)return;
        if(overflow_){
            ++responseFailures_;
            static constexpr char error[]="HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\nContent-Length: 0\r\n\r\n";
            WebServer::_currentClientWrite(error,sizeof(error)-1);
        }else if(used_&&WebServer::_currentClientWrite(reinterpret_cast<const char*>(buffer_),used_)!=used_){
            ++responseFailures_;
        }
    }
    static void taskEntry(void* arg){
        auto* self=static_cast<BusaHttpServer*>(arg);
        for(;;){
            const uint32_t start=millis();
            self->WebServer::handleClient();
            const uint32_t span=millis()-start;
            if(span>self->workerWorst_.load(std::memory_order_relaxed))self->workerWorst_.store(span,std::memory_order_relaxed);
            vTaskDelay(1);
        }
    }
    TaskHandle_t application_=nullptr;
    QueueHandle_t requests_=nullptr;
    SemaphoreHandle_t done_=nullptr;
    uint8_t* buffer_=nullptr;
    size_t used_=0;
    bool enabled_=false,capturing_=false,overflow_=false;
    std::atomic<uint32_t> workerWorst_{0},responseFailures_{0};
};
