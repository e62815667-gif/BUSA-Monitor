#pragma once
#include "Arduino.h"
#include <functional>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <cassert>
using Uri=std::string;enum HTTPMethod{HTTP_ANY,HTTP_GET,HTTP_POST};
// Model only the native WebServer boundary: blocking network reads/writes,
// synchronous callbacks, and virtual response write hooks.
class WebServer {
public:
 using THandlerFunction=std::function<void(void)>;
 explicit WebServer(int){}
 virtual ~WebServer()=default;
 virtual void handleClient(){
   if(!request.exchange(false))return;
   reading=true;while(!readReady.load())std::this_thread::yield();reading=false;
   if(upload){for(int i=0;i<3;++i){chunk=i;upload();}}
   if(handler)handler();
   completed=true;
 }
 void on(const Uri&,HTTPMethod,THandlerFunction fn){handler=fn;upload={};}
 void on(const Uri&,HTTPMethod,THandlerFunction fn,THandlerFunction ufn){handler=fn;upload=ufn;}
 void onNotFound(THandlerFunction fn){handler=fn;}
 void onFileUpload(THandlerFunction fn){upload=fn;}
 void send(const std::string& body){const std::string header="header:";_currentClientWrite(header.data(),header.size());_currentClientWrite(body.data(),body.size());}
 void send_P(const char* body){const std::string header="header:";_currentClientWrite(header.data(),header.size());_currentClientWrite_P(body,std::char_traits<char>::length(body));}
 void arm(){completed=false;readReady=false;writeReady=false;request=true;}
 std::atomic<bool> request{false},reading{false},readReady{false},writing{false},writeReady{false},completed{false};
 std::string sent;int chunk=-1;void* writeTask=nullptr;
protected:
 virtual size_t _currentClientWrite(const char* b,size_t n){writing=true;while(!writeReady.load())std::this_thread::yield();sent.append(b,n);writing=false;return n;}
 virtual size_t _currentClientWrite_P(PGM_P b,size_t n){return _currentClientWrite(b,n);}
private:THandlerFunction handler,upload;
};
