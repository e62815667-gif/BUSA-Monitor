#pragma once
#include <stdint.h>
#include <stddef.h>
namespace busa_perf {
enum Stage : uint8_t {early, http, uart, services, peripherals, snapshot, cloud, tft, i2c, logging, stageCount};
class LoopProfile {
public:
    static const char* name(size_t i){
        static const char* names[]={"early","http","uart","services","peripherals","snapshot","cloud","tft","i2c","logging"};
        return i<stageCount?names[i]:"none";
    }
    void begin(uint32_t now){
        at_=now;
        for(auto& n:current_)n=0;
    }
    void mark(Stage stage,uint32_t now){
        current_[stage]+=uint32_t(now-at_);at_=now;
    }
    void finish(uint32_t loopMs){
        uint8_t largest=0;
        for(uint8_t i=0;i<stageCount;++i){
            if(current_[i]>worst_[i])worst_[i]=current_[i];
            if(current_[i]>current_[largest])largest=i;
        }
        if(loopMs>100){++slow_[largest];last_=largest;}
    }
    uint32_t worst(size_t i) const{return i<stageCount?worst_[i]:0;}
    uint32_t slow(size_t i) const{return i<stageCount?slow_[i]:0;}
    uint8_t last() const{return last_;}
private:
    uint32_t at_=0,current_[stageCount]={},worst_[stageCount]={},slow_[stageCount]={};
    uint8_t last_=stageCount;
};
}
