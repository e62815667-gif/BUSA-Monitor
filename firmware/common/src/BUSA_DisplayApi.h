#pragma once
#include "BUSA_UartFrame.h"
#include <string.h>
namespace busa_uart {
constexpr uint8_t apiRequest=5, apiResponse=6;
constexpr size_t apiMaxRequest=1024, apiMaxArgs=12, apiMaxKey=32;
enum class ApiRoute:uint8_t {
    waterCoefficient=1,
    temperatureAlarms=2,
    wifiScan=3,
    wifiDisconnect=4,
    wifiConnect=5,
    theme=6,
    calibration=7,
    events=8,
    chartHistory=9,
    rainmakerEnable=10,
    rainmakerDisable=11,
    rainmakerStatus=12,
    googleConfig=13,
    googleTest=14,
    googleOff=15,
    keypadStart=16,keypadNext=17,keypadUndo=18,keypadCancel=19,geometryRead=20,geometrySave=21,referenceEdit=22,calibrationEdit=23,collectionCoefficient=24,resetVolume=25
};
struct ApiArg {const uint8_t* key;size_t keySize;const uint8_t* value;size_t valueSize;};
struct ApiRequest {ApiRoute route;size_t count;ApiArg args[apiMaxArgs];};
inline bool validRoute(uint8_t r){return r>=1&&r<=25;}
inline bool validKey(const uint8_t* p,size_t n){
    if(!p||!n||n>apiMaxKey)return false;
    for(size_t i=0;i<n;++i){uint8_t c=p[i];if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='_'))return false;}
    return true;
}
inline bool parseApiRequest(const uint8_t* p,size_t n,ApiRequest& out){
    if(!p||n<2||n>apiMaxRequest||!validRoute(p[0])||p[1]>apiMaxArgs)return false;
    ApiRequest q{};q.route=ApiRoute(p[0]);q.count=p[1];size_t at=2;
    for(size_t i=0;i<q.count;++i){
        if(n-at<3)return false;
        const size_t k=p[at],v=size_t(p[at+1])|(size_t(p[at+2])<<8);at+=3;
        if(k>n-at||!validKey(p+at,k)||v>n-at-k)return false;
        const uint8_t* key=p+at;const uint8_t* value=key+k;
        if(memchr(value,0,v))return false;
        for(size_t j=0;j<i;++j)if(q.args[j].keySize==k&&!memcmp(q.args[j].key,key,k))return false;
        q.args[i]=ApiArg{key,k,value,v};at+=k+v;
    }
    if(at!=n)return false;
    out=q;return true;
}
inline const ApiArg* findArg(const ApiRequest& q,const char* key){
    if(!key)return nullptr;
    const size_t n=strlen(key);
    for(size_t i=0;i<q.count;++i)if(q.args[i].keySize==n&&!memcmp(q.args[i].key,key,n))return &q.args[i];
    return nullptr;
}
class ApiBuilder {
public:
    ApiBuilder(uint8_t* p,size_t cap,ApiRoute route):p_(p),cap_(cap>apiMaxRequest?apiMaxRequest:cap){
        if(p_&&cap_>=2&&validRoute(uint8_t(route))){p_[0]=uint8_t(route);p_[1]=0;n_=2;}
    }
    bool add(const char* key,const char* value){
        if(!n_||!key||!value||p_[1]>=apiMaxArgs)return false;
        const size_t k=strlen(key),v=strlen(value);
        ApiRequest current{};
        if(!validKey(reinterpret_cast<const uint8_t*>(key),k)||v>cap_-n_||3+k+v>cap_-n_||
           !parseApiRequest(p_,n_,current)||findArg(current,key))return false;
        p_[n_++]=uint8_t(k);p_[n_++]=uint8_t(v);p_[n_++]=uint8_t(v>>8);
        memcpy(p_+n_,key,k);n_+=k;memcpy(p_+n_,value,v);n_+=v;++p_[1];return true;
    }
    size_t size()const{return n_;}
private:
    uint8_t* p_;size_t cap_,n_=0;
};
} // namespace busa_uart
