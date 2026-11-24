#pragma once
#include <stdint.h>
#include <math.h>
namespace BusaCollection {
class Meter {
 uint64_t count=0;
 uint32_t at=0;
 double rawRate=NAN;
 float factor=NAN;
public:
 static bool validCalibration(float value){return isfinite(value)&&value>0;}
 void begin(uint32_t now){at=now;count=0;rawRate=NAN;}
 void sample(uint32_t now,uint64_t pulses){
  const uint32_t dt=now-at;
  if(dt<2000)return;
  rawRate=(double)(pulses-count)*60000.0/dt;
  count=pulses;at=now;
 }
 bool setCalibration(float value){if(!validCalibration(value))return false;factor=value;return true;}
 bool calibrated()const{return validCalibration(factor);}
 float calibration()const{return factor;}
 bool seen()const{return count!=0;}
 uint64_t pulses()const{return count;}
 double raw()const{return seen()?rawRate:NAN;}
 double convert(double pulseRate)const{return calibrated()&&isfinite(pulseRate)?pulseRate/factor:NAN;}
 double rate()const{return convert(raw());}
 double total()const{return seen()?convert((double)count):NAN;}
};
}
