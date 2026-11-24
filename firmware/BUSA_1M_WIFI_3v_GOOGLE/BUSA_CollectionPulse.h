#pragma once
#include <stdint.h>
#include <cmath>

class BusaCollectionPulse {
    uint32_t at_=0,last_=0,interval_=0,delta_=0;
    uint64_t total_=0;
    float pulsesPerMl_=NAN;
public:
    void begin(uint32_t now,uint32_t count){at_=now;last_=count;interval_=delta_=0;total_=0;}
    static bool validCalibration(float value){return std::isfinite(value)&&value>0;}
    bool calibrate(float value){if(!validCalibration(value))return false;pulsesPerMl_=value;return true;}
    bool sample(uint32_t now,uint32_t count){
        const uint32_t dt=now-at_;
        if(dt<2000)return false;
        delta_=count-last_;total_+=delta_;last_=count;at_=now;interval_=dt;
        return true;
    }
    uint64_t pulses()const{return total_;}
    bool seen()const{return total_!=0;}
    bool calibrated()const{return validCalibration(pulsesPerMl_);}
    double rate()const{return seen()&&calibrated()&&interval_?double(delta_)*60000.0/interval_/pulsesPerMl_:NAN;}
    double volume()const{return seen()&&calibrated()?double(total_)/pulsesPerMl_:NAN;}
    float coefficient()const{return pulsesPerMl_;}
};
