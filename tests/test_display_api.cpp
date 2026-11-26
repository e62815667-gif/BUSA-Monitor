#include "../firmware/common/src/BUSA_DisplayApi.h"
#include <assert.h>
using namespace busa_uart;
int main(){
    uint8_t p[apiMaxRequest]={};
    ApiBuilder b(p,sizeof(p),ApiRoute::wifiConnect);
    assert(b.add("ssid","Network"));assert(b.add("pass","12345678"));
    assert(!b.add("ssid","duplicate"));assert(!b.add("bad key","x"));
    ApiRequest q{};assert(parseApiRequest(p,b.size(),q));assert(q.route==ApiRoute::wifiConnect&&q.count==2);
    const ApiArg* a=findArg(q,"pass");assert(a&&a->valueSize==8&&!memcmp(a->value,"12345678",8));
    for(size_t n=0;n<b.size();++n)assert(!parseApiRequest(p,n,q));
    assert(!parseApiRequest(p,b.size()+1,q));
    p[0]=255;assert(!parseApiRequest(p,b.size(),q));p[0]=uint8_t(ApiRoute::wifiConnect);
    p[b.size()-1]=0;assert(!parseApiRequest(p,b.size(),q));
    uint8_t duplicate[]={uint8_t(ApiRoute::theme),2,1,0,0,'a',1,0,0,'a'};
    assert(!parseApiRequest(duplicate,sizeof(duplicate),q));
    uint8_t calibration[apiMaxRequest]={};
    ApiBuilder coll(calibration,sizeof(calibration),ApiRoute::collectionCoefficient);
    assert(coll.add("confirm","YES")&&coll.add("pulses_per_ml","4.71"));
    assert(parseApiRequest(calibration,coll.size(),q)&&uint8_t(q.route)==24);
    a=findArg(q,"pulses_per_ml");assert(a&&a->valueSize==4&&!memcmp(a->value,"4.71",4));
    assert(uint8_t(ApiRoute::waterCoefficient)==1&&uint8_t(ApiRoute::calibrationEdit)==23&&validRoute(25)&&!validRoute(26));
    uint8_t reset[apiMaxRequest]={};ApiBuilder rb(reset,sizeof(reset),ApiRoute::resetVolume);assert(rb.add("target","water")&&rb.add("confirm","YES")&&parseApiRequest(reset,rb.size(),q)&&q.route==ApiRoute::resetVolume);
    uint8_t tiny[2];ApiBuilder small(tiny,sizeof(tiny),ApiRoute::googleTest);
    assert(!small.add("confirm","YES"));
    assert(parseApiRequest(tiny,small.size(),q)&&q.count==0);
}
