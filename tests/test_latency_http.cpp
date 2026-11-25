#include "../firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_HttpServer.h"
#include <cassert>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
static void until(const std::function<bool()>& ready){auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);while(!ready()){assert(std::chrono::steady_clock::now()<deadline);std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
int main(){
    auto app=xTaskGetCurrentTaskHandle();
    // Lifetime matches the production global server. No handler executes on
    // the socket worker, including each streaming upload callback.
    auto* server=new BusaHttpServer(80);unsigned calls=0;std::vector<int> chunks;
    server->on("/test",HTTP_POST,[&]{assert(xTaskGetCurrentTaskHandle()==app);++calls;std::string transient="response";server->send(transient);},[&]{assert(xTaskGetCurrentTaskHandle()==app);chunks.push_back(server->chunk);});
    assert(server->startWorker());server->arm();until([&]{return server->reading.load();});
    for(int i=0;i<100;++i){server->pollApplication();}assert(calls==0&&chunks.empty());
    server->readReady=true;
    until([&]{server->pollApplication();return server->writing.load();});
    assert(calls==1&&(chunks==std::vector<int>{0,1,2}));
    // A deliberately blocked network write cannot hold the application.
    for(int i=0;i<100;++i){server->pollApplication();}assert(!server->completed&&calls==1);
    server->writeReady=true;until([&]{return server->completed.load();});assert(server->sent=="header:response");
    fake_rtos::join();
    // Resource failure keeps the previous synchronous service path explicit.
    fake_rtos::failStart()=true;BusaHttpServer fallback(80);fallback.on("/test",HTTP_GET,[&]{assert(xTaskGetCurrentTaskHandle()==app);fallback.send_P("fallback");});
    assert(!fallback.startWorker()&&!fallback.workerEnabled());fallback.arm();fallback.readReady=true;fallback.writeReady=true;fallback.pollApplication();assert(fallback.sent=="header:fallback");
    std::cout<<"PASS actual HTTP handoff: stalled read/write, single app owner, streaming callbacks, exact response, fallback\n";
}
