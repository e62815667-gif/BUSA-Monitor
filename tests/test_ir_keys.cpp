#include "../firmware/BUSA_1M_WIFI_3v_GOOGLE/BUSA_IrKeys.h"
#include <cassert>
#include <vector>
#include <cstdio>
using namespace busa_ir;
int main(){
 Map m;uint64_t values[count];for(size_t i=0;i<count;++i)values[i]=identity(1,0x1234,uint16_t(50+i));
 assert(!m.ready()&&m.action(values[0])==none);
 m.start(0xfffffff0);assert(m.capture(values[0],true)==LearnResult::ignored);
 assert(m.capture(values[0],false)==LearnResult::accepted);
 assert(m.capture(values[0],false)==LearnResult::duplicate&&m.step()==1);
 for(size_t i=1;i<count;++i)assert(m.capture(values[i],false)==(i+1==count?LearnResult::complete:LearnResult::accepted));
 assert(m.finish(true)&&m.ready());
 const int expected[]={4,5,6,8,9,10,12,13,14,17,7,11,16,18,19,15,0};
 for(size_t i=0;i<count;++i)assert(m.action(values[i])==expected[i]);
 assert(m.action(identity(1,0x1235,50))==none&&m.action(identity(2,0x1234,50))==none);
 Map reboot;assert(reboot.load(values)&&reboot.action(values[16])==0);
 uint64_t broken[count]={};assert(!reboot.load(broken)&&reboot.action(values[0])==4);
 m.start(1000);for(size_t i=0;i<count;++i)m.capture(identity(2,1,uint16_t(i)),false);
 assert(!m.finish(false)&&m.action(values[0])==4);
 m.start(0xfffffff0);assert(m.expire(0xfffffff0+learnTimeoutMs)&&!m.learning());
 Press p;std::vector<int> out;auto emit=[&](int k){out.push_back(k);};
 p.frame(values[0],4,false,10,emit);for(uint32_t n=110;n<1000;n+=100)p.frame(values[0],4,true,n,emit);
 assert((out==std::vector<int>{4}));
 p.clear();out.clear();p.frame(values[9],17,false,1000,emit);p.expire(1351,emit);assert((out==std::vector<int>{17}));
 p.clear();out.clear();p.frame(values[9],17,false,2000,emit);
 for(uint32_t n=2100;n<=4500;n+=100)p.frame(values[9],17,true,n,emit);
 p.expire(4900,emit);assert((out==std::vector<int>{20}));
 p.clear();out.clear();p.frame(values[16],0,false,5000,emit);p.expire(5351,emit);assert((out==std::vector<int>{0}));
 p.clear();out.clear();p.frame(values[16],0,false,6000,emit);
 for(uint32_t n=6100;n<=8100;n+=100)p.frame(values[16],0,true,n,emit);
 p.expire(8500,emit);assert((out==std::vector<int>{1}));
 p.clear();out.clear();p.suppress(values[9],9000);p.frame(values[9],17,true,9100,emit);p.expire(9500,emit);assert(out.empty());
 p.frame(values[0],4,true,9600,emit);assert(out.empty());
 p.frame(values[9],17,false,0xfffffff0,emit);p.expire(0xfffffff0+351u,emit);assert((out==std::vector<int>{17}));
 std::puts("PASS IR learning, duplicate/foreign rejection, saved-map preservation, repeat suppression, 0/# hold, wake suppression and rollover");
}
