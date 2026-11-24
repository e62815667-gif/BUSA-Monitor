#include "BUSA_DisplayOtaSender.h"
#include <cassert>
#include <vector>
using namespace busa_ota;
static std::vector<uint8_t> image(){
 std::vector<uint8_t> p(60001);p[0]=0xe9;p[12]=9;p[32]=0x32;p[33]=0x54;p[34]=0xcd;p[35]=0xab;
 memcpy(p.data()+100,displayFamily,sizeof(displayFamily));return p;
}
int main(){
 auto p=image();std::vector<uint8_t> storage(p.size());uint8_t hash[32]={};Sender s;
 assert(s.start(storage.data(),p.size(),123,100));assert(!s.start(storage.data(),p.size(),124,100));
 assert(s.append(p.data(),17,101));assert(!s.readyToSeal());assert(s.append(p.data()+17,p.size()-17,102));assert(s.seal(hash,103));
 assert(s.available());size_t n=0;assert(!s.block(124,0,n));assert(!s.block(123,1024,n));assert(!s.accept(123,bootOk,104));
 const auto* block=s.block(123,0,n);assert(block&&n==1024&&!memcmp(block,p.data(),n));s.sent(0,n,105);
 assert(s.block(123,0,n)&&n==1024);s.sent(0,n,106);assert(s.confirmed()==0);
 for(uint32_t offset=1024;offset<p.size();offset+=1024){assert(s.block(123,offset,n)&&n==chunkLength(p.size(),offset));s.sent(offset,n,107);assert(s.confirmed()==offset);}
 assert(s.accept(123,written,108)&&s.phase()==Phase::waitingBoot);assert(s.confirmed()==p.size());assert(!s.accept(124,bootOk,109));assert(s.accept(123,bootOk,110)&&s.phase()==Phase::success&&!s.busy());
 assert(s.start(storage.data(),p.size(),124,0xfffffff0u));s.tick(0xfffffff0u+29999u);assert(s.busy());s.tick(0xfffffff0u+30000u);assert(s.phase()==Phase::error);
 p=image();memcpy(p.data()+200,mainFamily,sizeof(mainFamily));assert(s.start(storage.data(),p.size(),125,1));assert(s.append(p.data(),p.size(),2));assert(!s.seal(hash,3)&&s.phase()==Phase::error);
 assert(s.start(storage.data(),p.size(),126,1));assert(!s.append(p.data(),p.size()+1,2)&&s.phase()==Phase::error);
 p=image();assert(s.start(storage.data(),p.size(),127,1));assert(s.append(p.data(),p.size(),2));assert(s.seal(hash,3));assert(s.accept(127,failed,4)&&s.phase()==Phase::error);
}
