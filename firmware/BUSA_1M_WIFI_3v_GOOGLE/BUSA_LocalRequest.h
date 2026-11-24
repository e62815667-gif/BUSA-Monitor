#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include <BUSA_DisplayApi.h>
bool apClient();
class BusaLocalRequest {
public:
    explicit BusaLocalRequest(WebServer& http):http_(&http){}
    explicit BusaLocalRequest(const busa_uart::ApiRequest& req):request_(&req){}
    bool local()const{return http_?::apClient():request_!=nullptr;}
    bool hasArg(const String& key){return http_?http_->hasArg(key):busa_uart::findArg(*request_,key.c_str())!=nullptr;}
    String arg(const String& key){
        if(http_)return http_->arg(key);
        const auto* a=busa_uart::findArg(*request_,key.c_str());
        if(!a)return "";
        String s;
        if(!s.reserve(a->valueSize+1))return "";
        for(size_t i=0;i<a->valueSize;++i)s+=char(a->value[i]);
        return s;
    }
    void sendHeader(const String& name,const String& value){if(http_)http_->sendHeader(name,value);}
    void send(int code,const char* mime,const String& body){
        if(http_){http_->send(code,mime,body);return;}
        code_=code;body_=body;
        if(body_.length()!=body.length()){code_=500;body_="Response allocation failed";}
    }
    int code()const{return code_;}
    const String& body()const{return body_;}
private:
    WebServer* http_=nullptr;
    const busa_uart::ApiRequest* request_=nullptr;
    int code_=500;
    String body_;
};
